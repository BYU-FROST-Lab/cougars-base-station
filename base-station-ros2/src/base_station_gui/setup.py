from setuptools import find_packages, setup

package_name = 'base_station_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['FRoSt_Lab.png'],
        'base_station_gui.temp_mission_control': ['*.json'],
        'base_station_gui.temp_waypoint_planner': ['*.py'],
        'base_station_gui.vehicles_calibrate': ['*.sh']
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'base_station_interfaces', 'frost_interfaces'],
    zip_safe=True,
    maintainer='frostlab',
    maintainer_email='frostlab@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		    'gui_node = base_station_gui.gui_ros_node:main'
        ],
    },
)