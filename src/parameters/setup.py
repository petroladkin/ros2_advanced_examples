import os
from glob import glob
from setuptools import setup

package_name = 'parameters'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Petro Ladkin',
    maintainer_email='petro.ladkin@gmail.com',
    description='Package has node with examples use ROS2 parameters',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parameters_node = parameters.parameters_node:main'
        ],
    },
)
