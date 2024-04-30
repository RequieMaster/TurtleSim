from setuptools import find_packages, setup

package_name = 'turtlesim_dibujito'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Sierra, Sandra Corral, Paula Morales',
    maintainer_email='carlos.sierra@students.salle.url.edu, sandra.corral@students.salle.url.edu, paula.morales@students.salle.url.edu',
    description='dibujito',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_dibujito = turtlesim_dibujito.turtlesim_dibujito:main'
        ],
    },
)
