from glob import glob

from setuptools import setup

package_name = 'jetrover_sorting'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fuzesa',
    maintainer_email='fuzesa@users.noreply.github.com',
    description='Reworked colour sorting for the HiWonder JetRover',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sorting = jetrover_sorting.sorting_node:main',
        ],
    },
)
