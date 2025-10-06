from setuptools import setup

package_name = "ricoh_theta_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Falah Naufal",
    maintainer_email="falahnzk.dev@gmail.com",
    description="ROS 2 nodes that wrap RICOH THETA camera streams and utilities.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "capture = ricoh_theta_publisher.capture:main",
        ],
    },
)
