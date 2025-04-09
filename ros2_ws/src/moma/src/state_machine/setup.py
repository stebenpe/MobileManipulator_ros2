from setuptools import setup
import os
from glob import glob

package_name = 'state_machine'
submodules = "state_machine/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica-dev',
    maintainer_email='robotica-dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "get_drink_state_machine = state_machine.get_drink_state_machine:main",
            "demo = state_machine.demo:main",
            "moma_beckhoff = state_machine.moma_beckhoff:main",
            "website_integration = state_machine.website_integration:main",
            "moma_opc_ua = state_machine.moma_opc_ua:main"
        ],
    },
)