from setuptools import find_packages, setup

package_name = 'proc_planner_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config', ['config/proc_planner_config.yaml']),
    ],
    install_requires=['setuptools','quaternion', 'scipy'],
    zip_safe=True,
    maintainer='sonia2',
    maintainer_email='log.sonia.club@etsmtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proc_planner_ros2 = proc_planner_ros2.main:main'
        ],
    },
)
