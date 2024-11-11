import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cable_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
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
            'cable_descriptor = cable_description.cable_descriptor:main'
        ],
    },
)
