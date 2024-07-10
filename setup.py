from setuptools import find_packages, setup

package_name = 'moving_robot_py'

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
    maintainer='cagolinux',
    maintainer_email='cagdas96vkp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moving_robot_server =moving_robot_py.moving_robot_server:main",
            "moving_robot_client =moving_robot_py.moving_robot_client:main"
        ],
    },
)
