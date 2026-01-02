from setuptools import find_packages, setup

package_name = 'reblade_tools'

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
    maintainer='faiq',
    maintainer_email='faiq@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ["wheel_joint_state_from_cmdvel = reblade_tools.wheel_joint_state_from_cmdvel:main",
         'reactive_avoidance = reblade_tools.avoidance:main',
           'odom_to_tf = reblade_tools.odom_to_tf:main',
           'goal_nav_avoid = reblade_tools.goal_nav_avoid:main',
           'scan_frame_fix = reblade_tools.scan_frame_fix:main',
           'tf_aliases = reblade_tools.tf_aliases:main',
        ],
    },
)
