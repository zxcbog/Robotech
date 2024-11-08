from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
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
            'turtle_broadcaster = turtle_carrot.turtle_broadcaster:main',
            'turtle_listener = turtle_carrot.turtle_listener:main',
            'dynamic_frame_broadcaster = turtle_carrot.dynamic_frame_tf2_broadcaster:main',
        ],
    },
)
