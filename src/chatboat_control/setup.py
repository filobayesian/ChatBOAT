from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chatboat_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chatboat',
    maintainer_email='todo@todo.com',
    description='Control interfaces and test nodes for BlueROV2 Heavy + Reach Alpha 5 UVMS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test_commander = chatboat_control.test_commander:main',
            'gripper_service = chatboat_control.gripper_service:main',
        ],
    },
)
