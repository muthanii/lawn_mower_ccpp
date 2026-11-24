from glob import glob

from setuptools import find_packages, setup

package_name = 'lawn_mower_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muthana Ali',
    maintainer_email='mothana.alsaady@teebatkarbala.iq',
    description='Autonomous coverage: fuzzy path tracking, mission and simulation.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_pose = lawn_mower_navigation.sim_pose_node:main',
            'mission = lawn_mower_navigation.mission_node:main',
            'evaluator = lawn_mower_navigation.evaluator_node:main',
            'lawn_world = lawn_mower_navigation.world:main',
            'safety_check = lawn_mower_navigation.safety_check:main',
        ],
    },
)
