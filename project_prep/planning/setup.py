from setuptools import find_packages, setup
from glob import glob

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (('share/' + package_name + '/launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee106a-acp',
    maintainer_email='rpicardo@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_master = planning.main_project_2:main',
            'ik = planning.ik:main',
            'main = planning.main:main',
            'tf = planning.static_tf_transform:main',
        ],
    },
)
