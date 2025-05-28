from setuptools import find_packages, setup

package_name = 'tbb_roomba'

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
    maintainer='ming',
    maintainer_email='saintshe@gmail.com',
    description='tbb roomba',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = tbb_roomba.service:main',
        ],
    },
)
