from pathlib import Path

from setuptools import find_packages, setup

package_name = 'crsf_ros2'
submodules = 'crsf_ros2/submodules'
hil_source = Path(__file__).resolve().parents[2] / 'hil'
setup_dir = Path(__file__).resolve().parent


def hil_package_name(package):
    """Return the crsf_ros2.hil package name for an external HIL path."""
    relative = package.relative_to(hil_source)
    if relative == Path('.'):
        return f'{package_name}.hil'
    suffix = relative.as_posix().replace('/', '.')
    return f'{package_name}.hil.{suffix}'


hil_package_paths = [
    package
    for package in [hil_source, *hil_source.rglob('*')]
    if (package / '__init__.py').is_file()
]
hil_packages = [hil_package_name(package) for package in hil_package_paths]
package_dir = {
    hil_package_name(package): str(package)
    for package in hil_package_paths
}

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + hil_packages,
    package_dir=package_dir,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arunser',
    maintainer_email='stormbreaker.004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crsf_ros = crsf_ros2.ros2_crsf:main',
            'steering_test = crsf_ros2.steering_test:main',
            'throttle_test = crsf_ros2.throttle_test:main',
            'throttle_topic_test = crsf_ros2.throttle_topic_test:main',
            'teleop_test = crsf_ros2.teleop_test:main',
            'mpc_actuator = crsf_ros2.mpc_actuator:main',
            'aruco_track_follower = crsf_ros2.aruco_track_follower:main',
            'straight_static = crsf_ros2.hil.straight_static:main',
            'dynamic_straight = crsf_ros2.hil.dynamic_straight:main',
            'dyanmic_straight = crsf_ros2.hil.dynamic_straight:main',
            'mentor_demo_benchmark = crsf_ros2.hil.mentor_demo_benchmark:main',
            'straight_random_static_test = crsf_ros2.hil.random_static_test:main',
            'straght_static = crsf_ros2.hil.straght_static:main',
            'ellipse_static = crsf_ros2.hil.ellipse_static:main',
        ],
    },
)
