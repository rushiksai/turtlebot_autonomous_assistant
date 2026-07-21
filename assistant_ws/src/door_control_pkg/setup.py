from setuptools import setup

package_name = 'door_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rushiksaiii',
    maintainer_email='rushik@example.com',
    description='Door control package for knock-knock logic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'doormonitornode = door_control_pkg.doormonitornode:main',
        'door_logic_node = door_control_pkg.door_logic_node:main',
        'door_lidar_node = door_control_pkg.door_lidar_node:main',  # ADD THIS
    ],
},

)

