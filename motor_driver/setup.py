from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joy_motor_driver_launch.py',
            'launch/joy_motor_driver_servo_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tusharchillal',
    maintainer_email='chillal.tushar@gmail.com',
    description='Motor driver ROS 2 package for joystick and servo control',
    license='MIT',  # ← Update this if you're using a different license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_motor_driver = motor_driver.twist_motor_driver:main',
            'joy_motor_driver = motor_driver.joy_motor_driver:main',
            'joy_motor_driver_servo = motor_driver.joy_motor_driver_servo:main',
        ],
    },
)
