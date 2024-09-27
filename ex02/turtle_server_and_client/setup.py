from setuptools import find_packages, setup

package_name = 'turtle_server_and_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxim',
    maintainer_email='e@e.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "action_turtle_server = turtle_server_and_client.action_turtle_server:main",
            "action_turtle_client = turtle_server_and_client.action_turtle_client:main"
        ],
    },
)
