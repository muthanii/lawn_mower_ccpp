from setuptools import find_packages, setup

package_name = 'lawn_mower_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muthana Ali',
    maintainer_email='mothana.alsaady@teebatkarbala.iq',
    description='Coverage pipeline (BCD, lanes, A*) and cell ordering, with the Xie et al. (2024) GA-ACO baseline.',
    license='MIT',
    extras_require={'test': ['pytest']},
)
