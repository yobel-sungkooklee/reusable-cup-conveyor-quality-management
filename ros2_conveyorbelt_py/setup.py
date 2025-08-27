from setuptools import setup

package_name = 'ros2_conveyorbelt_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yobellee',
    maintainer_email='yobellee@yourdomain.com',
    description='Python nodes for cup spawning',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cup_spawner = ros2_conveyorbelt_py.cup_spawner:main', 
            'monitor_swapper = ros2_conveyorbelt_py.monitor_swapper:main',
            'slider_box_mover = ros2_conveyorbelt_py.slider_box_mover:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/ros2_conveyorbelt_py']),
        ('share/ros2_conveyorbelt_py', ['package.xml']),
    ],
)
