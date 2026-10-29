from setuptools import find_packages, setup

package_name = 'recognizer'

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
    maintainer='june',
    maintainer_email='june@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognizer_node = recognizer.recognizer_node:main',
            'object_segmentation_node = recognizer.object_segmentation_node:main',
            'object_segcolor_publish_node = recognizer.object_segcolor_publish_node:main'
            ],
    },
)
