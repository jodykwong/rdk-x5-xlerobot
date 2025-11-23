from setuptools import setup

package_name = 'xlerobot_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XleRobot Team',
    maintainer_email='developer@xlerobot.com',
    description='XleRobot视觉理解模块 - Story 1.6',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_llm_node = xlerobot_vision.vision_llm_node:main',
            'test_qwen_vl_client = xlerobot_vision.test_qwen_vl_client:main',
            'simple_api_test = xlerobot_vision.simple_api_test:main',
        ],
    },
)