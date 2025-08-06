from setuptools import find_packages, setup

package_name = 'giou_ps4'

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
    maintainer='ktl',
    maintainer_email='ktl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps4_control = giou_ps4.ps4_controll:main',
            'dynamixel_bridge = giou_ps4.move_dynamixel:main'
        ],
    },
)
