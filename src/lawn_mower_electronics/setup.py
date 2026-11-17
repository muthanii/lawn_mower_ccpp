from glob import glob

from setuptools import find_packages, setup

package_name = 'lawn_mower_electronics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muthana Ali',
    maintainer_email='mothana.alsaady@teebatkarbala.iq',
    description="Simulation models of the lawn mower robot's electronics.",
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'electronics = lawn_mower_electronics.nodes:main',
            'rc_transmitter = lawn_mower_electronics.transmitter_node:main',
        ],
    },
)
