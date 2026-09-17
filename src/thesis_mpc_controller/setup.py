from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'thesis_mpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name='launch'), glob('launch/*.py')),
        (os.path.join('share', package_name='config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jfms3',
    maintainer_email='jfmsaul@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kalman_filter_node = thesis_mpc_controller.kalman_filter_node:main',
            'synthetic_source = thesis_mpc_controller.synthetic_source:main',
            'mpc_node = thesis_mpc_controller.mpc_node:main',
            'flight_control_node = thesis_mpc_controller.flight_control_node:main',
        ],
    },
)
