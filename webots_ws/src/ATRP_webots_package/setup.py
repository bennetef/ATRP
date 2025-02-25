from setuptools import setup

package_name = 'ATRP_webots_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/ATRP_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/ATRP.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/ATRP.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ATRP_webots_driver = ATRP_webots_package.ATRP_webots_driver:main',
            'ATRP_webots_controller = ATRP_webots_package.ATRP_webots_controller:main'
        ],
    },
)
