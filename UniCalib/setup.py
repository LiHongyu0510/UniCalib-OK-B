import sys
import os
from glob import glob

# colcon 会传入 --editable 等 setuptools 不认识的选项，在 import setuptools 前过滤掉
def _strip_colcon_args(argv):
    single_arg = (
        "--build-directory", "--build-base", "--install-base",
        "--install-purelib", "--install-platlib", "--install-lib",
        "--install-headers", "--install-scripts", "--install-data",
        "--script-dir", "--exec-prefix", "--install-layout",
    )
    out = []
    skip_next = False
    for a in argv:
        if skip_next:
            skip_next = False
            continue
        if a in ("--editable", "-e") or a.startswith("--editable="):
            continue
        if any(a.startswith(opt + "=") for opt in single_arg):
            continue
        if a in single_arg:
            skip_next = True
            continue
        out.append(a)
    return out

sys.argv = _strip_colcon_args(sys.argv)

from setuptools import setup, find_packages

package_name = 'unicalib'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'sensor_templates'),
            glob('config/sensor_templates/*.yaml')),
        (os.path.join('share', package_name, 'config', 'vehicle_configs'),
            glob('config/vehicle_configs/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UniCalib Team',
    maintainer_email='calib@example.com',
    description='UniCalib: Unified Multi-Sensor Automatic Calibration System',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unicalib_node = unicalib.ros2_node:main',
            'run_calibration = scripts.run_calibration:main',
        ],
    },
)
