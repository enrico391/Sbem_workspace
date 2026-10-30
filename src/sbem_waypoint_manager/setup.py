from glob import glob
from setuptools import find_packages, setup

package_name = 'sbem_waypoint_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Enrico Moro',
    maintainer_email='enrimoro003@gmail.com',
    description='Map-aware persistent named robot locations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'waypoint_manager = sbem_waypoint_manager.node:main',
    ]},
)
