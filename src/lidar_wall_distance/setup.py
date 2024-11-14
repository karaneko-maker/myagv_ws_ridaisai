from setuptools import find_packages, setup

package_name = 'lidar_wall_distance'

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
            "scan_crop_node =lidar_wall_distance.scan_crop_node:main",
            "wall_distance_node =lidar_wall_distance.wall_distance_node:main",
            "distance_vel_node =lidar_wall_distance.distance_vel_node:main",
            "lidar_wall_find =lidar_wall_distance.lidar_wall_find:main"
        ],
    },
)
