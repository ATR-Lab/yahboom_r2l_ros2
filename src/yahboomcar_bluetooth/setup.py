from setuptools import find_packages, setup

package_name = 'yahboomcar_bluetooth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bluetooth_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='irvsteve@gmail.com',
    description='Bluetooth bridge for iPhone AR app to robot communication in multiplayer racing system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluetooth_bridge_node = yahboomcar_bluetooth.bluetooth_bridge_node:main',
            'bluetooth_ros2_bridge = yahboomcar_bluetooth.bluetooth_ros2_bridge:main',
        ],
    },
)
