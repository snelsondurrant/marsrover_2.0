from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

        # IMPORTANT! Add new custom models below so they show up in Gazebo sensors
        (os.path.join('share', package_name, 'models/urc_aruco_0'),
            glob('models/urc_aruco_0/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_0/meshes'),
            glob('models/urc_aruco_0/meshes/*')),
        (os.path.join('share', package_name, 'models/urc_aruco_1'),
            glob('models/urc_aruco_1/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_1/meshes'),
            glob('models/urc_aruco_1/meshes/*')),
        (os.path.join('share', package_name, 'models/urc_aruco_2'),
            glob('models/urc_aruco_2/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_2/meshes'),
            glob('models/urc_aruco_2/meshes/*')),
        (os.path.join('share', package_name, 'models/urc_aruco_3'),
            glob('models/urc_aruco_3/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_3/meshes'),
            glob('models/urc_aruco_3/meshes/*')),
        (os.path.join('share', package_name, 'models/urc_aruco_4'),
            glob('models/urc_aruco_4/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_4/meshes'),
            glob('models/urc_aruco_4/meshes/*')),
        (os.path.join('share', package_name, 'models/urc_aruco_5'),
            glob('models/urc_aruco_5/model*')),
        (os.path.join('share', package_name, 'models/urc_aruco_5/meshes'),
            glob('models/urc_aruco_5/meshes/*')),
        (os.path.join('share', package_name, 'models/coke_can'),
            glob('models/coke_can/model*')),
        (os.path.join('share', package_name, 'models/coke_can/meshes'),
            glob('models/coke_can/meshes/*')),
        (os.path.join('share', package_name, 'models/coke_can/materials/textures'),
            glob('models/coke_can/materials/textures/*')),
        (os.path.join('share', package_name, 'models/hammer'),
            glob('models/hammer/model*')),
        (os.path.join('share', package_name, 'models/hammer/meshes'),
            glob('models/hammer/meshes/*')),
        (os.path.join('share', package_name, 'models/hammer/materials/textures'),
            glob('models/hammer/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Gazebo simulation for the Mars Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_obj_detect = rover_gazebo.sim_obj_detect_node:main',
        ],
    },
)
