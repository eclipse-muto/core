import os
from glob import glob
from setuptools import setup

package_name = 'core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f"{package_name}.model"
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "log"), glob("log/*.log")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alp SARICA',
    maintainer_email='alp.sarica@eteration.com',
    description="Eclipse Muto Core Package",
    license='EPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twin = core.twin:main'
        ],
    },
)
