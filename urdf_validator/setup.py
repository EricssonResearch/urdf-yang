import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'urdf_validator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'urdf_validator.urdf_validator_node',
        'urdf_validator.instance_validator_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcell Balogh',
    maintainer_email='balogh.marcell@edu.bme.hu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urdf_validator_node = urdf_validator.urdf_validator_node:main',
            'urdf_formatter_node = urdf_validator.urdf_formatter_node:main',
            'instance_validator = urdf_validator.instance_validator_node:main'
        ],
    },
)
