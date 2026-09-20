from setuptools import setup
import os
from glob import glob
from pathlib import Path

package_name = 'rosbot_plus_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'README.md', 'CONTRIBUTING.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        *[
            (os.path.join('share', package_name, str(directory)),
             [str(path) for path in directory.iterdir() if path.is_file()])
            for directory in [Path('docs'), Path('docs/media')]
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='k.abdrahamane22@gmail.com',
    description='ROSbot rear-drive Ackermann description with 2D lidar and RGB-D camera',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_physics = rosbot_plus_description.check_physics:main',
            'cloud_sanitize = rosbot_plus_description.cloud_sanitize:main',
            'elevation_map = rosbot_plus_description.elevation_map:main',
        ],
    },
)
