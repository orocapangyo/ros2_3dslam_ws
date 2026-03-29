from setuptools import find_packages, setup

package_name = 'amr_motion_test_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/test_ui.launch.py',
            'launch/test_gui.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test_ui_node = amr_motion_test_ui.test_ui_node:main',
            'gui_node = amr_motion_test_ui.gui_node:main',
        ],
    },
)
