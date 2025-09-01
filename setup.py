from setuptools import setup

package_name = "spi_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/teleop_spi.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="SPI bridge node for Raspberry Pi ↔ Nucleo F446RE (Mbed CE).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "spi_bridge_node = spi_bridge.spi_bridge_node:main",
        ],
    },
)
