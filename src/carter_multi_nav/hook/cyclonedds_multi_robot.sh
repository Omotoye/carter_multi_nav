# Source this package's CycloneDDS profile so CLI tools match the launched graph.
_carter_multi_nav_share_dir="${COLCON_CURRENT_PREFIX}/share/carter_multi_nav"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_carter_multi_nav_share_dir}/config/cyclonedds_multi_robot.xml"
unset _carter_multi_nav_share_dir
