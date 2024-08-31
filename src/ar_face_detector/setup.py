import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ar_face_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selexin',
    maintainer_email='selexin@selexin.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ar_face_detector = ar_face_detector.ar_face_detector:main',
            'ar_face_detector_moveit = ar_face_detector.ar_face_detector_moveit:main',
            'ar_face_detector_state_publisher = ar_face_detector.ar_face_detector_state_publisher:main',
            'ar_nerf_firer = ar_face_detector.ar_nerf_firer:main',
            'esphome_leds = ar_face_detector.esphome_leds:main'
        ],
    },
)
