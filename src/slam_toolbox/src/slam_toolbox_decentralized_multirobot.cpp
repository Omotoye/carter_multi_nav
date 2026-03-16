/*
 * Decentralized multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2025, Achala Athukorala
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include "slam_toolbox/slam_toolbox_decentralized_multirobot.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
DecentralizedMultiRobotSlamToolbox::DecentralizedMultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  /** For decentralized multi-robot slam, each robot runs a slam_toolbox instance
   *  Each slam_toolbox instace should be run with a unique namespace
   *  This namespace acts as the 'identity' of the particular slam_toolbox instance
   */
  host_ns_ = this->get_namespace(); 
  // Remove namespace leading slash
  if (!host_ns_.empty() && host_ns_.front() == '/') host_ns_.erase(0,1); 
  if (host_ns_.empty()) {
    RCLCPP_ERROR(get_logger(), "This node must run in a non-root namespace (e.g., /robot1).");
    throw std::runtime_error("Namespace required");
  }

  if (!this->has_parameter("scan_share_topic")) {
    this->declare_parameter("scan_share_topic", "/localized_scan");
  }
  localized_scan_topic_ = this->get_parameter("scan_share_topic").as_string();
  RCLCPP_INFO(get_logger(), "Sharing scans on:  %s topic", localized_scan_topic_.c_str());

  localized_scan_pub_ = this->create_publisher<slam_toolbox::msg::LocalizedLaserScan>(
    localized_scan_topic_, 10);
  localized_scan_sub_ = this->create_subscription<slam_toolbox::msg::LocalizedLaserScan>(
    localized_scan_topic_, 10, std::bind(
      &DecentralizedMultiRobotSlamToolbox::localizedScanCallback,
      this, std::placeholders::_1));
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(
      get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // Note: When paused, only host scan processing will be paused.
  // Scan data from peers will still be processed -> refer to localizedScanCallback
  if (shouldProcessScan(scan, pose)) {
    LocalizedRangeScan * range_scan = addScan(laser, scan, pose);
    if (range_scan != nullptr) {
      Matrix3 covariance;
      covariance.SetToIdentity();
      publishLocalizedScan(
        scan, laser->GetOffsetPose(),
        range_scan->GetOdometricPose(), covariance, scan->header.stamp);
    }
  }
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::localizedScanCallback(
  slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan)
{
  std::string scan_ns = localized_scan->scan.header.frame_id.substr(
    0, localized_scan->scan.header.frame_id.find('/'));
  if (scan_ns == host_ns_) {
    return;  // Ignore callbacks from ourself
  }

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan =
    std::make_shared<sensor_msgs::msg::LaserScan>(localized_scan->scan);
  tf2::Quaternion quat_tf;
  tf2::convert(localized_scan->pose.pose.pose.orientation, quat_tf);
  Pose2 pose(localized_scan->pose.pose.pose.position.x,
    localized_scan->pose.pose.pose.position.y,
    tf2::getYaw(quat_tf));

  LaserRangeFinder * laser = getLaser(localized_scan);
  if (!laser) {
    RCLCPP_WARN(
      get_logger(), "Failed to create device for received localizedScanner"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }
  LocalizedRangeScan * range_scan = addExternalScan(laser, scan, pose);

  if (range_scan != nullptr) {
    // Publish transform
    pose = range_scan->GetCorrectedPose();
    tf2::Quaternion q(0., 0., 0., 1.0);
    geometry_msgs::msg::TransformStamped tf_msg;
    q.setRPY(0., 0., pose.GetHeading());
    tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
    tf2::toMsg(transform, tf_msg.transform);
    tf_msg.header.frame_id = map_frame_;
    tf_msg.header.stamp = localized_scan->pose.header.stamp;
    tf_msg.child_frame_id = localized_scan->scanner_offset.header.frame_id;
    tfB_->sendTransform(tf_msg);
  }
}

/*****************************************************************************/
LocalizedRangeScan * DecentralizedMultiRobotSlamToolbox::addExternalScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  // Get the localized range scan from the peer's reported pose
  LocalizedRangeScan * range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  boost::mutex::scoped_lock lock(smapper_mutex_);

  // ---------------------------------------------------------------
  // Map-only processing for peer scans:
  // We scan-match against nearby nodes to get a corrected pose (so the
  // peer's data renders correctly in the map), but we do NOT add
  // vertices, edges, or attempt loop closures.  This prevents the
  // peer scan from perturbing the host robot's pose graph, which is
  // the root cause of the map-to-odom TF micro-jitter and double-wall
  // artifacts in the shared map.
  // ---------------------------------------------------------------

  // The mapper must be initialized before we can use the scan matcher
  karto::LaserRangeFinder * pLaserRangeFinder = range_scan->GetLaserRangeFinder();
  if (pLaserRangeFinder == nullptr ||
      pLaserRangeFinder->Validate(range_scan) == false)
  {
    delete range_scan;
    return nullptr;
  }

  // Ensure the mapper is initialized (it auto-initializes on first Process,
  // but external scans may arrive before any host scan is processed)
  karto::ScanMatcher * pScanMatcher = smapper_->getMapper()->GetSequentialScanMatcher();
  if (pScanMatcher == nullptr) {
    // Mapper not yet initialized — initialize it now so scan matching works
    smapper_->getMapper()->Initialize(pLaserRangeFinder->GetRangeThreshold());
    pScanMatcher = smapper_->getMapper()->GetSequentialScanMatcher();
  }

  // Scan-match against the nearest existing node to refine the pose
  // without creating any graph constraints
  Matrix3 covariance;
  covariance.SetToIdentity();

  if (pScanMatcher != nullptr) {
    // Find the closest existing scan to match against
    karto::MapperGraph * pGraph = smapper_->getMapper()->GetGraph();
    if (pGraph != nullptr) {
      auto * closestVertex = pGraph->FindNearByScan(
        range_scan->GetSensorName(), range_scan->GetOdometricPose());

      if (closestVertex != nullptr) {
        // Build a minimal running-scan set from the nearby node
        LocalizedRangeScanVector nearbyScans;
        nearbyScans.push_back(closestVertex->GetObject());

        Pose2 bestPose;
        pScanMatcher->MatchScan(
          range_scan, nearbyScans, bestPose, covariance);
        range_scan->SetSensorPose(bestPose);
      }
    }
  }

  // Store the scan for map rendering (occupancy grid generation)
  // but do NOT add it to the graph or trigger loop closures
  if (enable_interactive_mode_) {
    scan_holder_->addScan(*scan);
  }

  // Add scan to the sensor manager so it appears in the occupancy grid,
  // but skip AddVertex/AddEdges/TryCloseLoop
  smapper_->getMapper()->GetMapperSensorManager()->AddScan(range_scan);
  dataset_->Add(range_scan);

  return range_scan;
}


/*****************************************************************************/
LaserRangeFinder * DecentralizedMultiRobotSlamToolbox::getLaser(
  const slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan)
/*****************************************************************************/
{
  const std::string & frame = localized_scan->scan.header.frame_id;
  if (lasers_.find(frame) == lasers_.end()) {
    try {
      lasers_[frame] = laser_assistant_->toLaserMetadata(
        localized_scan->scan, localized_scan->scanner_offset);
      dataset_->Add(lasers_[frame].getLaser(), true);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to compute laser pose[%s], "
        "aborting initialization (%s)", frame.c_str(), e.what());
      return nullptr;
    }
  }

  return lasers_[frame].getLaser();
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::publishLocalizedScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  const Pose2 & offset,
  const Pose2 & pose,
  const Matrix3 & cov,
  const rclcpp::Time & t)
/*****************************************************************************/
{
  slam_toolbox::msg::LocalizedLaserScan scan_msg;

  scan_msg.scan = *scan;

  tf2::Quaternion q_offset(0., 0., 0., 1.0);
  q_offset.setRPY(0., 0., offset.GetHeading());
  tf2::Transform scanner_offset(q_offset, tf2::Vector3(offset.GetX(), offset.GetY(), 0.0));
  tf2::toMsg(scanner_offset, scan_msg.scanner_offset.transform);
  scan_msg.scanner_offset.header.stamp = t;

  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, scan_msg.pose.pose.pose);

  scan_msg.pose.pose.covariance[0] = cov(0, 0) * position_covariance_scale_;  // x
  scan_msg.pose.pose.covariance[1] = cov(0, 1) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[6] = cov(1, 0) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[7] = cov(1, 1) * position_covariance_scale_;  // y
  scan_msg.pose.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw
  scan_msg.pose.header.stamp = t;

  // Prefix frame names with unique robot namespace identifier
  scan_msg.scan.header.frame_id = (*(scan->header.frame_id.cbegin()) == '/') ?
    host_ns_ + scan->header.frame_id :
    host_ns_ + "/" + scan->header.frame_id;

  scan_msg.pose.header.frame_id = (*(map_frame_.cbegin()) == '/') ?
    host_ns_ + map_frame_ :
    host_ns_ + "/" + map_frame_;

  scan_msg.scanner_offset.child_frame_id = scan_msg.scan.header.frame_id;

  scan_msg.scanner_offset.header.frame_id = (*(base_frame_.cbegin()) == '/') ?
    host_ns_ + base_frame_ :
    host_ns_ + "/" + base_frame_;

  localized_scan_pub_->publish(scan_msg);
}

/*****************************************************************************/
bool DecentralizedMultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_WARN(
      get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::DecentralizedMultiRobotSlamToolbox)