from setuptools import find_packages, setup

package_name = 'turret_control_py'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turret_controller = turret_control_py.turret_controller:main',
            'gun_controller = turret_control_py.gun_controller:main',
            'ballistic_marker = turret_control_py.ballistic_marker:main',
            'servo_packets = turret_control_py.servo_packets:main',
        ],
    },
)
