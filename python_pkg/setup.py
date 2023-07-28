from setuptools import setup

package_name = 'python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryu',
    maintainer_email='xogns2079@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_capture_node = python_pkg.image_capture:main',
            'cmd_vel_pub_node = python_pkg.cmd_vel_pub:main',
            'laser_sub_node = python_pkg.laser_sub:main'
        ],
    },
)
