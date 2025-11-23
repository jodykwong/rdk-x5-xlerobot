from setuptools import setup

package_name = 'xlerobot_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/xlerobot_camera.launch.py']),
        ('share/' + package_name + '/config', ['config/imx219_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XleRobot Team',
    maintainer_email='sunrise@xlerobot.com',
    description='XleRobot IMX219 camera driver for ROS2 with VIN interface support',
    license='MIT',
    python_requires='>=3.6',
    entry_points={
        'console_scripts': [
            'camera_node = xlerobot_camera.camera_node:main',
            'vin_camera_test = xlerobot_camera.vin_camera_driver:test_driver',
        ],
    },
)