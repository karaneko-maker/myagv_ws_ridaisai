from setuptools import find_packages, setup

package_name = 'lidar_check'

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
            "lidar_wall_find =lidar_check.lidar_wall_find:main",
            "lidar_wall_find_1 =lidar_check.lidar_wall_find_1:main",
            "lidar_wall_find_2 =lidar_check.lidar_wall_find_2:main",
            "lidar =lidar_check.lidar:main"
        ],
    },
)
