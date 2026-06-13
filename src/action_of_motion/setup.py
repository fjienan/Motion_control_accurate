from setuptools import find_packages, setup

package_name = 'action_of_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/param.yaml']),
        ('share/' + package_name + '/launch', ['launch/motion_action.launch.py']),
        ('share/' + package_name + '/scripts', [
            'scripts/send_move_goal.sh',
            'scripts/plot_pid_debug.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fjienan',
    maintainer_email='fjienan@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_action_node = action_of_motion.motion_action_node:main',
        ],
    },
)
