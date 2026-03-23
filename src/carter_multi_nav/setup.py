from glob import glob
import os

from setuptools import find_packages, setup


package_name = "carter_multi_nav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("bin", [os.path.join("scripts", "carter_ros2")]),
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "hook"), glob("hook/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omotoye",
    maintainer_email="adekoyaomotoye@gmail.com",
    description="Standalone multi-Carter Nav2 and decentralized slam_toolbox bringup.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "april_tag_detector = carter_multi_nav.april_tag_detector:main",
            "lifecycle_bringup_gate = carter_multi_nav.lifecycle_bringup_gate:main",
            "map_selector = carter_multi_nav.map_selector:main",
            "multi_robot_map_metrics = carter_multi_nav.multi_robot_map_metrics:main",
            "multi_robot_viz = carter_multi_nav.multi_robot_viz:main",
            "nav2_service_gate = carter_multi_nav.nav2_service_gate:main",
            "nav_benchmark_runner = carter_multi_nav.nav_benchmark_runner:main",
            "nav_ready_gate = carter_multi_nav.nav_ready_gate:main",
            "planning_map_clearer = carter_multi_nav.planning_map_clearer:main",
            "recommend_nav_slam_params = carter_multi_nav.recommend_nav_slam_params:main",
            "rviz_goal_router = carter_multi_nav.rviz_goal_router:main",
            "scan_peer_exclusion = carter_multi_nav.scan_peer_exclusion:main",
            "scan_motion_diagnostics = carter_multi_nav.scan_motion_diagnostics:main",
            "scan_motion_gate = carter_multi_nav.scan_motion_gate:main",
            "scan_relay = carter_multi_nav.scan_relay:main",
            "tf_aggregate_relay = carter_multi_nav.tf_aggregate_relay:main",
        ],
    },
)
