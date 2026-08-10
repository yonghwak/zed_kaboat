from setuptools import setup

package_name = 'zed_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='yonghwa',
    maintainer_email='kyh031106@gmail.com',
    description='매핑 + 카메라 색/모양 탐지 + 좌표 등록',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_node = zed_perception.mapping_node:main',
            'camera_node = zed_perception.camera_node:main',
            'perception_bridge_node = zed_perception.perception_bridge_node:main',
        ],
    },
)
