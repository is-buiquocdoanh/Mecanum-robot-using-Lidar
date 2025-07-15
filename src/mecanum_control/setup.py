from setuptools import find_packages, setup

package_name = 'mecanum_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_control.launch.py',]),
        ('share/' + package_name + '/launch', ['launch/esp32_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_bridge = mecanum_control.esp32_serial_bridge:main',
            'joy_to_cmd_vel = mecanum_control.joy_to_cmd_vel:main',
        ],
    },
)
