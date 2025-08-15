from setuptools import find_packages, setup

package_name = 'marker_extension'

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
    maintainer='martzi',
    maintainer_email='balogh.marcell95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_connection = marker_extension.marker_connection:main',
            'interactive_and_simple_marker = marker_extension.interactive_and_simple_marker:main',
        ],
    },
)
