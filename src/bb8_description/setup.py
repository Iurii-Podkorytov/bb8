from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'bb8_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
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
            'p3d_tf_broadcast = bb8_description.p3dbroadcaster:main',
        ],
    },
)
