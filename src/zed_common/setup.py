from setuptools import setup

package_name = 'zed_common'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yonghwa',
    maintainer_email='kyh031106@gmail.com',
    description='공용 설정/좌표변환/미션 데이터 라이브러리',
    license='MIT',
    entry_points={'console_scripts': []},
)
