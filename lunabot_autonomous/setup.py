from setuptools import setup

package_name = 'lunabot_autonomous'

setup(
    name=package_name,
    version='0.0.0',
     packages=['simulated_robot'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grayson Arendt',
    maintainer_email='grayson.n.arendt@gmail.com',
    description='This package contains autonomous code.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = simulated_robot.teleop.keyboard_teleop:main',
        ],
    },
)
