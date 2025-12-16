from setuptools import setup
import os
from glob import glob

package_name = 'fake_data_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publishes fake odom and encoder data for testing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_odom_publisher = fake_data_pkg.fake_odom_publisher:main',
            'fake_encoder_publisher = fake_data_pkg.fake_encoder_publisher:main',
        ],
    },
)
