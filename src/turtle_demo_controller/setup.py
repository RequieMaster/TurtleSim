from setuptools import find_packages, setup

package_name = 'turtle_demo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos, Sandra, Paula',
    maintainer_email='carlos.sierra@students.salle.url.edu, sandra.corral@students.salle.url.edu, paula.morales@students.salle.url.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turt_controller = turtle_demo_controller.turtle_controller:main'
        ],
    },
)
