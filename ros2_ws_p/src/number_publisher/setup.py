from setuptools import find_packages, setup

package_name = 'number_publisher'

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
    maintainer='Ntiaju Chukwuebuka Eric',
    maintainer_email='ntiajubukason@gmail.com',
    description='A package to practice ROS2 concepts.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "num_pub = number_publisher.number_publisher:main",
            "number_counter = number_publisher.number_counter:main",
            "reset_counter_client = number_publisher.reset_counter_client:main"
        ],
    },
)
