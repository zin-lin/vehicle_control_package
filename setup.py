from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vehicle_control_package'
dynamixel = 'vehicle_control_package/dynamixel'
components = 'vehicle_control_package/components'
parameters = 'vehicle_control_package/parameters'
launch = 'launch'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, dynamixel, components, parameters, launch],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/'+parameters, glob(os.path.join('vehicle_control_package/parameters', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zin',
    maintainer_email='zinlinhtun34@gmail.com',
    description='Control Vehicle Low Level Communication and movements',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_control=vehicle_control_package.vehicle_control:main',
            'dynamixel=vehicle_control_package.dynamixel.dynamixel_publisher_node:main'
        ],
    },
)
