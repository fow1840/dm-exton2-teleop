from setuptools import setup
import os
from glob import glob

package_name = 'robot_ik_retargeting'

setup(
    name=package_name,
    version='0.0.1',
    # 确保包含包内的所有 Python 模块
    packages=[package_name],
    data_files=[
        # 1. 注册包索引，以便 ros2 run 能找到该包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2. 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 3. 安装 Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Pink-based IK Retargeting',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 核心配置：将脚本注册为可执行命令 
    entry_points={
        'console_scripts': [
            # 格式为 '可执行文件名 = 包名.文件名:入口函数'
            'ik_node = robot_ik_retargeting.ik_node:main',
            'ik_node_tracik = robot_ik_retargeting.ik_node_tracik:main',

        ],
    },
)