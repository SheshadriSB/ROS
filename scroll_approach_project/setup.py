from setuptools import find_packages, setup

package_name = 'scroll_approach_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS 2 resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch',
         ['launch/controller_launch.py']),
        # Configuration files
        ('share/' + package_name + '/config',
         ['config/params.yaml']),
   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='yourname@example.com',
    description='Scroll approach controller with serial bridge and service-based scroll detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scroll_approach_controller = scroll_approach_project.controller_node:main',
            'motor_bridge_node = scroll_approach_project.motor_bridge_node:main',
            'deadwheel_bridge_node = scroll_approach_project.deadwheel_bridge_node:main',
        ],
    },
)