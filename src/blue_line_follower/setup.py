from setuptools import setup
import os
from glob import glob

package_name = 'blue_line_follower'

# Function to get all files recursively in a directory
def get_data_files(directory, install_dir):
    data_files = []
    for root, dirs, files in os.walk(directory):
        if files:
            install_path = os.path.join(install_dir, os.path.relpath(root, directory))
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ] + get_data_files('models', os.path.join('share', package_name, 'models')) + get_data_files('aruco', os.path.join('share', package_name, 'aruco')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wayay',
    maintainer_email='wayay@todo.todo',
    description='Blue line follower robot with 4 wheels for Gazebo simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = blue_line_follower.line_follower_node:main',
            'aruco_navigation_server = blue_line_follower.aruco_navigation_server:main',
            'aruco_navigation_client = blue_line_follower.aruco_navigation_client:main',
            'aruco_diagnostic = blue_line_follower.aruco_diagnostic:main',
        ],
    },
)
