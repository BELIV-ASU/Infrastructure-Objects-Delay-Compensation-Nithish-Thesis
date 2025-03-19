from setuptools import find_packages, setup

package_name = 'infrastructure_objects_delay_compensation'

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
    maintainer='nithish',
    maintainer_email='nithish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['object_delay_compensator = infrastructure_objects_delay_compensation.object_delay_compensation:main',
        'object_delay_compensator_version2 = infrastructure_objects_delay_compensation.object_delay_compensation_version2:main',
        'object_delay_compensator_and_output_regulator = infrastructure_objects_delay_compensation.object_delay_compensation_and_output_regulator:main',
        'object_delay_compensator_and_output_regulator_version2 = infrastructure_objects_delay_compensation.object_delay_compensation_and_output_regulator_version2:main'
        ],
    },
)
