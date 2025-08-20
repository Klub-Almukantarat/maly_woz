from setuptools import setup

package_name = "maly_woz_hw"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michał Wójcik",
    maintainer_email="wojcikmichal98@gmail.com",
    description="Package for controlling the motors using Twist messages.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"ff_control = {package_name}.ff_control:main"],
    },
)
