from glob import glob

from setuptools import setup


package_name = "lunar_d455_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lunar",
    maintainer_email="lunar@example.com",
    description="D455 wide-FOV RGB-D and IMU bringup for lunar_slam.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "check_d455_topics = lunar_d455_bringup.check_d455_topics:main",
        ],
    },
)
