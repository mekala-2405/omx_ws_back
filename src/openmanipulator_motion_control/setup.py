from setuptools import setup

package_name = 'openmanipulator_motion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh',
    maintainer_email='skdhiksdhj',
    description='ROS2 package to move OpenManipulator-X in predefined poses',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'movement= openmanipulator_motion_control.movement:main',
            'clockwise = openmanipulator_motion_control.clockwise:main',
            'teleopkey = openmanipulator_motion_control.teleop:main',
            'service = openmanipulator_motion_control.move_arm_service:main',
            'client = openmanipulator_motion_control.move_arm_client:main',
            'SetPose_service = openmanipulator_motion_control.SetPose_service:main',
            'SetPose_client = openmanipulator_motion_control.SetPose_client:main',
            'PerformTask = openmanipulator_motion_control.PerformTask_service:main',
            'PerformTaskFeedback = openmanipulator_motion_control.PerformTask_service_feedback:main',
            'PerformTaskAction = openmanipulator_motion_control.PerformTask_action:main',
       
            
        ],
    },
)
#
