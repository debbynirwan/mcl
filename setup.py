from setuptools import setup

package_name = 'mcl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/mcl_launch.py']),
        ('share/' + package_name + '/resource',
            ['resource/random_bounce.yaml']),
        ('share/' + package_name + '/resource',
            ['resource/motion_model.yaml']),
        ('share/' + package_name + '/resource',
            ['resource/epuck_world_map.pgm']),
        ('share/' + package_name + '/resource',
            ['resource/epuck_world_map.yaml']),
        ('share/' + package_name + '/resource',
            ['resource/configs.rviz']),
        ('share/' + package_name + '/resource',
            ['resource/sensor_model.yaml']),
        ('share/' + package_name + '/resource',
            ['resource/mcl.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Debby Nirwan',
    author_email='debby_nirwan@yahoo.com',
    maintainer='Debby Nirwan',
    maintainer_email='debby_nirwan@yahoo.com',
    keywords=['ROS2', 'Webots', 'Monte Carlo Localization',
              'MCL', 'Simulation', 'Examples', 'Particle Filter'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Monte Carlo Lozalizer for epuck on Webots Simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_bounce = mcl.random_bounce:main',
            'mission_controller = mcl.mission_controller:main',
            'monte_carlo_localizer = mcl.monte_carlo_localizer:main',
            'ros_to_serial = mcl.ros_to_serial:main',
        ],
    },
)
