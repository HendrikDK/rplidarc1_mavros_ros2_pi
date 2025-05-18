from setuptools import setup, find_packages

package_name = 'scan_to_mavlink'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scan_to_mavlink.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Convert RPLIDAR scan to OBSTACLE_DISTANCE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_to_mavlink_node = scan_to_mavlink.scan_to_mavlink_node:main',
        ],
    },
)
