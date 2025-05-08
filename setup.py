import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dummy_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
         include=['dummy_controller', 'dummy_controller.*', 
                  'dummy_cli_tool', 'dummy_cli_tool.*']
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yeyiru1997@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_controller = dummy_controller.dummy_controller:main',
        ],
    },
)
