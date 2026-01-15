from setuptools import find_packages, setup

package_name = 'ricketts_control'

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
    maintainer='ale',
    maintainer_email='puglisialessandro27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom2tf = ricketts_control.odom2tf:main',
            'ricketts_joint_states = ricketts_control.ricketts_joint_states:main',
            'ricketts_joint_states_simple = ricketts_control.ricketts_joint_states_simple:main',
            'ricketts_joy_teleop = ricketts_control.ricketts_joy_teleop:main',
        ],
    },
)
