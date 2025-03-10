from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Hardware controls for the Mars Rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_switch = rover_control.drive_switch:main',
            'mega_wrapper = rover_control.mega_wrapper:main',
            'nano_wrapper = rover_control.nano_wrapper:main',
        ],
    },
)
