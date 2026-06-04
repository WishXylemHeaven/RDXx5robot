from setuptools import setup
from glob import glob
import os

package_name = "digua_bpu_yolo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="wishxylemheaven@users.noreply.github.com",
    description="BPU YOLO wrapper package for Digua robot semantic mapping",
    license="MIT",
    entry_points={
        "console_scripts": [
            "realtime_bpu_yolo_node = digua_bpu_yolo.realtime_bpu_yolo_node:main",
            "offline_detections_publisher_node = digua_bpu_yolo.offline_detections_publisher_node:main",
        ],
    },
)
