from setuptools import setup

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS2 nodes to interface with ESP32: cmd_vel bridge and odometry publisher.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmdvel_to_esp32 = esp32_bridge.cmdvel_to_esp32:main',
            'esp32_odom_node = esp32_bridge.esp32_odom_node:main',
        ],
    },
)
