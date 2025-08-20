from setuptools import find_packages, setup

package_name = 'yahboomcar_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yahboom_joy.launch.py',
                                               'launch/yahboom_keyboard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='irvsteve@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboom_joy = yahboomcar_ctrl.yahboom_joy:main',
            'yahboom_keyboard = yahboomcar_ctrl.yahboom_keyboard:main',
        ],
    },
)
