import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'omron_moma'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('robot_description/urdf/*')),
        ('share/' + package_name, glob('robot_description/meshes/chassis/*')),
        ('share/' + package_name, glob('robot_description/config/*')),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gy',
    maintainer_email='guanyewtan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = omron_moma.demo:main',
            'teach_setup = omron_moma.teach_setup:main',
            'view_publisher = omron_moma.view_transform_publisher:main'
        ],
    },
)
