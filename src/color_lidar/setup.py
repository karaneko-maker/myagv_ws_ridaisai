from setuptools import find_packages, setup

package_name = 'color_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yurina',
    maintainer_email='yurina@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_control_node =color_lidar.move_control_node:main",
            "test_node =color_lidar.test_node:main",
            "trans_command_node =color_lidar.trans_command_node:main",
            "lidar_node =color_lidar.lidar_node:main"
        ],
    },
)
