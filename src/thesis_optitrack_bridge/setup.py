from setuptools import find_packages, setup

package_name = 'thesis_optitrack_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'optitrack_bridge = thesis_optitrack_bridge.optitrack_bridge_node:main',
            'flight_node = thesis_optitrack_bridge.flight_node:main',
            'time_constant_classification_node = thesis_optitrack_bridge.time_constant_classification_node:main',
            'drag_classification_node = thesis_optitrack_bridge.drag_classification_node:main',
            'simple_hover_node = thesis_optitrack_bridge.simple_hover_node:main'
        ],
    },
)
