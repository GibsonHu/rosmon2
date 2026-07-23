from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'rosmon2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         [path for path in glob('launch/*') if os.path.isfile(path)]),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Gibson',
    maintainer_email='gibson.hu@uts.edu.au',
    description='A rosmon-style terminal launcher and process monitor for ROS 2.',
    license='BSD-3-Clause',
    url='https://github.com/GibsonHu/rosmon2',
    project_urls={
        'Bug Tracker': 'https://github.com/GibsonHu/rosmon2/issues',
        'Source': 'https://github.com/GibsonHu/rosmon2',
    },
    entry_points={
        'console_scripts': [
            'mon2 = rosmon2.cli:main',
            'rosmon2 = rosmon2.cli:main',
            'rosmon2-mcp = rosmon2.mcp_server:main',
        ],
    },
)
