from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nabi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='louis',
    maintainer_email='johnlouis.dante@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder = nabi.recorder:main',
            'writer = nabi.writer:main',
            'parser = nabi.parser:main',
            'talker = nabi.talker:main',
            'joint = nabi.joint:main',
            'canbus = nabi.canbus:main',
        ],
    },
)
