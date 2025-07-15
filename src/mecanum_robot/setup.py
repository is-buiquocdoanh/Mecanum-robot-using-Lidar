from setuptools import find_packages, setup

package_name = 'mecanum_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/mecanum_robot', ['package.xml']),
         ('share/' + package_name + '/description', [
            'description/robot.urdf',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/display.launch.py',
            'launch/slam_toolbox_launch.py',
            'launch/full_mapping.launch.py',
            'launch/test_slam.launch.py',
            'launch/navigation.launch.py',
            ]),
        ('share/' + package_name + '/config', [
            'config/test_slam.yaml',
            'config/slam_config.yaml',
        ]),

        ('share/' + package_name + '/maps', [
            'maps/my_map.yaml',
            'maps/my_map.pgm',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='doanh762003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_odom = mecanum_robot.static_odom:main',
        ],
    },
)
