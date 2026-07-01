from pathlib import Path
import os

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop

package_name = 'crsf_ros2'
submodules = 'crsf_ros2/submodules'
setup_dir = Path(__file__).resolve().parent


def find_hil_source():
    """Locate the hardware hil source directory from build and source layouts."""
    path = setup_dir
    for parent in [path, *path.parents]:
        candidate = parent / 'hil'
        if (candidate / '__init__.py').is_file():
            return candidate
        candidate = parent / 'hardware' / 'hil'
        if (candidate / '__init__.py').is_file():
            return candidate
    raise FileNotFoundError('cannot locate hardware/hil source directory')


hil_source = find_hil_source()


class develop(_develop):
    """Accept colcon-specific develop flags for backward compatibility."""

    user_options = _develop.user_options + [
        ('editable', None, 'support colcon editable mode'),
        ('build-directory=', None, 'ignored build directory'),
        ('symlink-data', None, 'ignored symlink data'),
        ('force', None, 'ignored force flag'),
        ('uninstall', None, 'ignored uninstall flag'),
    ]
    boolean_options = _develop.boolean_options + [
        'editable',
        'symlink-data',
        'force',
        'uninstall',
    ]

    editable = False
    build_directory = None
    symlink_data = False
    force = False
    uninstall = False

    def initialize_options(self):
        super().initialize_options()
        self.editable = False
        self.build_directory = None
        self.symlink_data = False
        self.force = False
        self.uninstall = False

    def finalize_options(self):
        _develop.finalize_options(self)

    def run(self):
        if self.uninstall:
            return
        super().run()


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
    '': str(setup_dir),
    **{
        hil_package_name(package): str(package)
        for package in hil_package_paths
    }
}

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + hil_packages,
    package_dir=package_dir,
    cmdclass={
        'develop': develop,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'cvxpy'],
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
            'follow_the_gap_dynamic_straight = crsf_ros2.hil.dynamic_straight:main',
            'ftg_dynamic_straight = crsf_ros2.hil.dynamic_straight:main',
            'virtual_vehicle = crsf_ros2.hil.simulation.virtual_vehicle:main',
            'mentor_demo_benchmark = crsf_ros2.hil.mentor_demo_benchmark:main',
            'straight_random_static_test = crsf_ros2.hil.random_static_test:main',
            'straght_static = crsf_ros2.hil.straght_static:main',
            'ellipse_static = crsf_ros2.hil.ellipse_static:main',
        ],
    },
)
