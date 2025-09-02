from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sparkie_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattsays',
    maintainer_email='mattsays@sparkie.com',
    description='Sparkie perception package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No executables in this package - only launch files
        ],
    },
)
