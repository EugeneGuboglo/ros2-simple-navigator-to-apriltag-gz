from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), 
         glob('worlds/*.sdf')),
        # Include model files
        (os.path.join('share', package_name, 'models', 'simple_robot'), 
         glob('models/simple_robot/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='AprilTag navigation robot',
    license='Apache-2.0',
    tests_require=['pytest'],
entry_points={
    'console_scripts': [
        'apriltag_navigator = apriltag_navigation.apriltag_navigator:main',
    ],
},
)
