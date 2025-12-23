from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_kf'

setup(
    name='drone_kf',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), ['simple_drone.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akindu',
    maintainer_email='akindu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_kf_gazebo = drone_kf.drone_kf_gazebo:main',
        ],
    },
)
