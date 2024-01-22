from setuptools import setup
from glob import glob

package_name = 'ros2_rpi_video_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pablo',
    maintainer_email='pawel.rawicki@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_rpi_video_recorder = ros2_rpi_video_recorder.ros2_rpi_video_recorder:main'
        ],
    },
)
