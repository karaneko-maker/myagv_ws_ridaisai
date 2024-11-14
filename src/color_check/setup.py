from setuptools import find_packages, setup

package_name = 'color_check'

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
            "color_read =color_check.color_read:main",
            "color_control =color_check.color_control:main",
            "color_find =color_check.color_find:main",
            "color_control_2 =color_check.color_control_2:main",
            "color_angle_test =color_check.color_angle_test:main",
            "angle_position_control =color_check.angle_position_control:main",
            "turn_left =color_check.turn_left:main",
            "turn_left_2 =color_check.turn_left_2:main",
            "color_find_blue =color_check.color_find_blue:main",
        ],
    },
)
