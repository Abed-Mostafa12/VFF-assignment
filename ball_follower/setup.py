from setuptools import find_packages, setup

package_name = 'ball_follower'

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
    maintainer='abed',
    maintainer_email='abedmostafaa94@gmail.com',
    description='Ball following robot using ROS2 and computer vision',
    license='TODO: License declaration',
    tests_require=['pytest'],
   entry_points={
    'console_scripts': [
        'ball_follower = ball_follower.ball_follower:main',  # Corrected entry point
    ],
},
    package_data={
        'ball_follower': ['scripts/*.py'],  # Make sure Python scripts are included
    },
)

