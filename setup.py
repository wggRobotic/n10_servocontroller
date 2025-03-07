from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'n10_servocontroller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'parameter'), glob('parameter/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christopher',
    maintainer_email='coroc@buzzmail.eu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'n10_servocontroller = n10_servocontroller.n10_servocontroller:main',
        ],
    },
    package_data={package_name: ['PCAController.py']}

)
