from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pursuit_evasion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Natanael Bimo Priambodo',
    maintainer_email='natanaelpriambodo@gmail.com',
    description='Pursuit and Evasion Case using Turtlesim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_tf2_broadcaster = pursuit_evasion.turtle_tf2_broadcaster:main',
                            'pursuer = pursuit_evasion.pursuer:main',
                            'evader = pursuit_evasion.evader:main',
                            'obstacle = pursuit_evasion.obstacle:main',
                            ],
    },
)
