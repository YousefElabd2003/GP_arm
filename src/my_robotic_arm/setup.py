from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all URDF/XACRO files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        # Include all mesh files (STL)
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        # CRITICAL: Include the config folder for MoveItPy parameters
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yousef_elabd',
    maintainer_email='Yousef.Elabd@hotmail.com',
    description='Senior Graduation Project: 6-joint robotic arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Registering your IK mover node
            'arm_mover_node = my_robotic_arm.arm_mover:main',
            'point_extractor = my_robotic_arm.point_extractor:main',
            'arm_ik_solver = my_robotic_arm.arm_ik_solver:main',
            'door_sim = my_robotic_arm.door_routine_sim:main',
            'elevator_sim = my_robotic_arm.elevator_routine_sim:main',
        ],
    },
)