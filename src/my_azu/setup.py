import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_azu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'configs'),
        glob('my_azu/configs/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
        glob('my_azu/rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'),
        glob('my_azu/urdf/*')),
        (os.path.join('share', package_name, 'meshes'),
        glob('my_azu/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azusa',
    maintainer_email='azusa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'realsense_yolo_node = my_azu.nodes.realsense_yolo_node:main',
            'rplidar_node = my_azu.nodes.rplidar_node:main',
        ],
    },
)
