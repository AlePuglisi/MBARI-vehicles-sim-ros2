import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mola_auv_estimation'

model_files = [ f for pattern in ('models/*.pt', 'models/*.onnx') for f in glob(pattern) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install everything in the models folder
        (os.path.join('share', package_name, 'models'), model_files),
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
            'mola_auv_apriltag = mola_auv_estimation.mola_auv_apriltag:main',
            'mola_auv_plot = mola_auv_estimation.mola_auv_plot:main',     
            'mola_auv_species_detection = mola_auv_estimation.mola_auv_species_detection:main',           
        ],
    },
)
