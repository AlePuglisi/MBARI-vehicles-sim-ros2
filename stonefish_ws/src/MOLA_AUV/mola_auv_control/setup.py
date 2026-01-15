from setuptools import find_packages, setup

package_name = 'mola_auv_control'

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
            'odom2tf = mola_auv_control.odom2tf:main',
            'mola_auv_joint_states = mola_auv_control.mola_auv_joint_states:main',
            'mola_auv_joint_states_simple = mola_auv_control.mola_auv_joint_states_simple:main',
            'mola_auv_joy_teleop = mola_auv_control.mola_auv_joy_teleop:main',
        ],
    },
)
