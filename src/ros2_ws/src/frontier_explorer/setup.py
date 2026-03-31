from setuptools import find_packages, setup

package_name = "frontier_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/frontier_explorer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="saif",
    maintainer_email="saif@example.com",
    description="Simple frontier-based autonomous exploration node for ROS 2 Nav2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_explorer_node = frontier_explorer.frontier_explorer_node:main",
        ],
    },
)
