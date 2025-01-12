from setuptools import setup

package_name = 'assignment1_rt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rubin',
    maintainer_email='S6558048@studenti.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_interface_node = assignment1_rt.user_interface_node:main',
            'distance_monitor_node = assignment1_rt.distance_monitor_node:main'
        ],
    },
)
