from glob import glob
import os

from setuptools import find_packages, setup


package_name = "carter_multi_nav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
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
            "map_selector = carter_multi_nav.map_selector:main",
            "scan_motion_diagnostics = carter_multi_nav.scan_motion_diagnostics:main",
            "scan_motion_gate = carter_multi_nav.scan_motion_gate:main",
            "scan_relay = carter_multi_nav.scan_relay:main",
            "tf_aggregate_relay = carter_multi_nav.tf_aggregate_relay:main",
        ],
    },
)
