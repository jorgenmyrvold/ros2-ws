from setuptools import setup

package_name = 'joint_publisher'

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
    maintainer='jorgen',
    maintainer_email='myrvoldou@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_joint_publisher = joint_publisher.dummy_joint_publisher:main',
            'joint_jogger = joint_publisher.joint_jogger:main'
        ],
    },
)
