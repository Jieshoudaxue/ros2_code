import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'service_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycao',
    maintainer_email='1641395022@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = service_py.server:main',
            'client = service_py.client:main',
            'server_diy = service_py.server_diy:main',
            'client_diy = service_py.client_diy:main'            
        ],
    },
)
