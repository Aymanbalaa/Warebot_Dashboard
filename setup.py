from setuptools import setup
import os
from glob import glob

package_name = 'nav_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
        'websockets',
    ],
    zip_safe=True,
    maintainer='ayman',
    maintainer_email='ayman@todo.com',
    description='Web-based navigation dashboard for ROS2 robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = nav_dashboard.main:main',
        ],
    },
)
