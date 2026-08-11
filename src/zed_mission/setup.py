from setuptools import setup

package_name = 'zed_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'opencv-python', 'numpy', 'Pillow', 'pyserial'],
    zip_safe=True,
    maintainer='yonghwa',
    maintainer_email='kyh031106@gmail.com',
    description='경로계획 + 조향 + 스러스터 구동 + 웹 대시보드',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_planner_node = zed_mission.path_planner_node:main',
            'helm_node = zed_mission.helm_node:main',
            'safety_supervisor_node = zed_mission.safety_supervisor_node:main',
            'mission_manager_node = zed_mission.mission_manager_node:main',
            'thruster_output = zed_mission.thruster_output:main',
            'dashboard_node = zed_mission.dashboard_node:main',
            'mission_params_node = zed_mission.mission_params_node:main',
        ],
    },
)
