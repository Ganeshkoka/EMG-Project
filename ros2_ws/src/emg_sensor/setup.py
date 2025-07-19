from setuptools import setup

package_name = 'emg_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=['scripts.ble_bridge'],   # so ros2 run can find it
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ble_bridge = scripts.ble_bridge:main',
        ],
    },
)
