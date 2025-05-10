from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'bb8_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yura',
    maintainer_email='i.podkorytov@innopolis.university',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_controller = bb8_controllers.head_controller:main',
            'head_pid_controller = bb8_controllers.head_pid_controller:main',
            'hamster_controller = bb8_controllers.hamster_controller:main',
            'wheels_odom = bb8_controllers.wheels_odom:main',
            'red_circle_follower = bb8_controllers.red_circle_follower:main',
        ],
    },
)
