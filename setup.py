from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mjolnir_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='oktma',
    maintainer_email='martino@uia.no',
    description='Contol of Mjolnir arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = mjolnir_arm_control.main:main'
        ],
    },
)
