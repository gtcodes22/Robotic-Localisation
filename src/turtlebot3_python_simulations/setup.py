from setuptools import setup

package_name = 'turtlebot3_python_simulations'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gideon',
    maintainer_email='your_email@example.com',
    description='A* pathfinding with DWA for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_pathplanning = turtlebot3_python_simulations.a_star_pathplanning:main',  # This should match the filename and function
        ],
    },
)
