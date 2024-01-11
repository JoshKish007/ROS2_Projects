from setuptools import find_packages, setup

package_name = 'aisd_hearing'

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
    maintainer='aisd',
    maintainer_email='ayya0017@algonquinlive.com',
    description='Ros2 package for tasks',
    license='no license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'recording_publisher = aisd_hearing.recording_publisher:main',
        'words_publisher = aisd_hearing.words_publisher:main',
        'speak_client = aisd_hearing.speak_client:main',
        ],
    },
)
