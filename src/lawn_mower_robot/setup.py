from glob import glob

from setuptools import find_packages, setup

package_name = 'lawn_mower_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/robot', glob('config/robot/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muthana Ali',
    maintainer_email='mothana.alsaady@teebatkarbala.iq',
    description='Robot config, skid-steer kinematic and energy model, and Gazebo launch.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segment_runner = lawn_mower_robot.segment_runner:main',
            'robot_sdf = lawn_mower_robot.sdf_model:main',
        ],
    },
)
