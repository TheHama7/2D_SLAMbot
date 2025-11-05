from setuptools import setup

package_name = 'arduino_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Arduino serial interface for ROS2',
    entry_points={
        'console_scripts': [
            'arduino_bridge = arduino_bridge.serial_bridge:main'
        ],
    },
)
