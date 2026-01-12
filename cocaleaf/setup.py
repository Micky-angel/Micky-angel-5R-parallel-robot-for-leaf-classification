from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocaleaf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabricio',
    maintainer_email='chhubyxd1627@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam = cocaleaf.camara:main',
            'cine = cocaleaf.cinematica:main',
            'yolo = cocaleaf.yolo:main',
            'visio = cocaleaf.vision:main',
            'cal = cocaleaf.calculo:main',
            'CPU = cocaleaf.cerebro:main',
            'tra = cocaleaf.trayectoria:main',
            'simu = cocaleaf.simulacion:main',
        ],
    },
)