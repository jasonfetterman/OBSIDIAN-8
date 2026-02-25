from setuptools import find_packages, setup

package_name = 'obsidian_mission'

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
    maintainer='twitc',
    maintainer_email='twitc@todo.todo',
    description='Mission governance node for OBSIDIAN-8',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = obsidian_mission.mission_node:main',
        ],
    },
)
