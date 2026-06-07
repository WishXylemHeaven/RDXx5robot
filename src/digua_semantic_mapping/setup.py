from setuptools import setup
from glob import glob
import os

package_name = "digua_semantic_mapping"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="wishxylemheaven@users.noreply.github.com",
    description="Semantic mapping package for Digua robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "semantic_map_tool = digua_semantic_mapping.semantic_map_tool:main",
            "semantic_goto_node = digua_semantic_mapping.semantic_goto_node:main",
            "semantic_observer_node = digua_semantic_mapping.semantic_observer_node:main",
            "semantic_fusion_node = digua_semantic_mapping.semantic_fusion_node:main",
            "semantic_marker_node = digua_semantic_mapping.semantic_marker_node:main",
        ],
    },
)
