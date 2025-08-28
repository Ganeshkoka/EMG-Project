from setuptools import setup

package_name = 'emg_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=['scripts.ble_bridge',
                'scripts.hand_controller',
                'scripts.command_publisher',
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'ble_bridge = scripts.ble_bridge:main',
            'hand_controller = scripts.hand_controller:main',
            'command_publisher = scripts.command_publisher:main', 
        ],
    },
)
