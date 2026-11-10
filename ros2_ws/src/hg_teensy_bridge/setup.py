# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup

package_name = "hg_teensy_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wynn Pham",
    maintainer_email="215331687+pkquy-vn@users.noreply.github.com",
    description="Home Guardian Pi-Teensy bridge: HEARTBEAT, MODE, CLEAR_FAULT out; STATUS in.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teensy_bridge = hg_teensy_bridge.teensy_bridge:main",
        ],
    },
)
