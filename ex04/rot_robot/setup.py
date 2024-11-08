from setuptools import find_packages, setup
import glob
import os

package_name = 'rot_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py')),
        (os.path.join('share/' + package_name + '/src/description'), glob.glob('src/description/*.urdf')),
        (os.path.join('share/' + package_name + '/rviz'), glob.glob('rviz/*.rviz')),
        (os.path.join('share/' + package_name + '/config'), glob.glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxim',
    maintainer_email='1',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rot_robot = rot_robot.circle_rot:main',
        ],
    },
)
