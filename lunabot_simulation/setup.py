from setuptools import setup

package_name = "lunabot_simulation"

setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grayson Arendt",
    maintainer_email="grayson.n.arendt@gmail.com",
    description="This package contains the URDF, worlds, and models for visualization",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = teleop.keyboard_teleop:main",
        ],
    },
)
