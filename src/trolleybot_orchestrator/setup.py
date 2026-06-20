from setuptools import find_packages, setup

package_name = "trolleybot_orchestrator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TrolleyBot maintainers",
    maintainer_email="bryanlnp@gmail.com",
    description="Mission orchestration for TrolleyBot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = trolleybot_orchestrator.mission_node:main",
        ],
    },
)
