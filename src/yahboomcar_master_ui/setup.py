from setuptools import find_packages, setup

package_name = 'yahboomcar_master_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
        'numpy',
    ],
    zip_safe=True,
    maintainer='atr-lab',
    maintainer_email='irvsteve@gmail.com',
    description='Master monitoring and control UI for robot car racing system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_ui = yahboomcar_master_ui.master_ui:main'
        ],
    },
)
