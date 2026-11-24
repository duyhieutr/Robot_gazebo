from setuptools import find_packages, setup

package_name = 'wsl_exploration'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/exploration.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/exploration.launch.py', 'launch/extrude.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Azer Ben Ali',
    maintainer_email='azerbenali5017@gmail.com',
    description='Autonomous frontier exploration for WSL.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = wsl_exploration.frontier_explorer:main',
            'map_extruder = wsl_exploration.map_extruder:main',
            'save_cloud = wsl_exploration.save_cloud:main',
        ],
    },
)
