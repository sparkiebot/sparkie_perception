from glob import glob
from setuptools import find_packages, setup

package_name = 'sparkie_odom'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/params', glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattsays',
    maintainer_email='mattia.sicoli@gmail.com',
    description='Sparkie Odometry',
    license='MIT',
    tests_require=[],
    entry_points={
        'console_scripts': [
        ],
    },
)
