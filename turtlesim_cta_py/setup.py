from setuptools import find_packages, setup

package_name = 'turtlesim_cta_py'

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
    maintainer='lateef',
    maintainer_email='akinolalateefolanrewaju@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawner = turtlesim_cta_py.turtle_spawner:main",
            "turtle_controller = turtlesim_cta_py.turtle_controller:main"
        ],
    },
)