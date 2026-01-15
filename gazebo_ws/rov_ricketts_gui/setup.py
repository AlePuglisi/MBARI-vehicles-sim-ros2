from setuptools import find_packages, setup

package_name = 'rov_ricketts_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', 
         ['resources/Rov_Ricketts.png', 'resources/thruster.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ale',
    maintainer_email='puglisialessandro27@gmail.com',
    description='Simple GUI to send commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_ricketts_gui = rov_ricketts_gui.rov_ricketts_gui:main'
        ],
    },
)
