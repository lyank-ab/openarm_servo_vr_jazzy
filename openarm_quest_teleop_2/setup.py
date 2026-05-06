from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'openarm_quest_teleop_2'


def files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            files('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            files('config/*.yaml')),
        ('share/' + package_name + '/web',
            files('web/*')),
    ],
    install_requires=['setuptools', 'numpy', 'websockets'],
    zip_safe=True,
    maintainer='air-lab-ncsu',
    maintainer_email='air-lab-ncsu@todo.todo',
    description='OpenArm Quest 3 bimanual teleoperation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quest_bridge = openarm_quest_teleop_2.quest_bridge:main',
        ],
    },
)
