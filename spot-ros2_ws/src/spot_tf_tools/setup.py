from setuptools import setup

package_name = 'spot_tf_tools'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base_cam_calibration.launch.py']),
        ('share/' + package_name + '/config', ['config/apriltag_params.yaml']),
        ('share/' + package_name + '/config', ['config/robot_segmenter_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luiz F Dantas',
    maintainer_email='luiz.felipe.r.dantas@gmail.com',
    description='TF tools for Spot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'solve_base_cam = spot_tf_tools.solve_base_cam:main',
            'tf_calib_monitor = spot_tf_tools.tf_calib_monitor:main',
            'segmenter_supervisor = spot_tf_tools.segmenter_supervisor:main'
        ],
    },
)
