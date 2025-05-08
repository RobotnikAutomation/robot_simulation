from setuptools import setup
from glob import glob
import os

package_name = 'webots_robotnik'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

# launch, worlds, protos
data_files.append(('share/' + package_name + '/launch', glob('launch/*.py')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*.wbt')))
data_files.append(('share/' + package_name + '/protos', glob('protos/*.proto')))

# resource con subdirectorios
for dirpath, dirnames, filenames in os.walk('resource'):
    if filenames:
        install_path = os.path.join('share', package_name, dirpath)
        file_paths = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((install_path, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelramosbeltran@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main'
        ],
    },
)

