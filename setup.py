from setuptools import find_packages, setup

package_name = "robot_nerf_launcher"

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
    maintainer="marco",
    maintainer_email="30201929+goldjunge91@users.noreply.github.com",
    description="Nerf launcher control and interfaces for my_steel robot",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "nerf_launcher_node=robot_nerf_launcher.nerf_launcher_node:main",
        ],
    },
)
