from setuptools import find_packages, setup

package_name = 'myagv_communication'

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
    maintainer_email='7521075@ed.tus.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "diff_drive_controller = myagv_communication.diff_drive_controller:main",
            "serial = myagv_communication.serial:main"
        ],
    },
)
