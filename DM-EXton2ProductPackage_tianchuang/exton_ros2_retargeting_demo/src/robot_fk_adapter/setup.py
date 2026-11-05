from setuptools import setup
import os
from glob import glob

package_name = 'robot_fk_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your@email.com',
    description='FK Adapter for Teleoperation Exoskeleton',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_to_pose_adapter = robot_fk_adapter.joint_to_pose_adapter:main',
            # 必须添加下面这一行，因为你的 Launch 文件用到了它
            'dummy_robot_driver = robot_fk_adapter.dummy_robot_driver:main',
        ],
    },
)