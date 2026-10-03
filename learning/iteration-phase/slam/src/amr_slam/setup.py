from setuptools import setup
from glob import glob
import os

package_name = 'amr_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='amr@todo.todo',
    description='SLAM Toolbox configuration for AMR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
