from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Autonomy capabilities (using Nav2) for the Mars Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = rover_navigation.state_machine_node:main',
        ],
    },
)
