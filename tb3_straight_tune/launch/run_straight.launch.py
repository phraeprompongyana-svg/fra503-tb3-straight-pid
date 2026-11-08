"""รันโหนดวิ่งตรง 1 รอบผ่าน ros2 launch (ไม่มี stdin จึงไม่ถามระยะที่วัดจริง)

ใช้: ros2 launch tb3_straight_tune run_straight.launch.py v:=0.06
     ros2 launch tb3_straight_tune run_straight.launch.py v:=0.06 dist:=1.0 \
         profile_file:=/path/params_best_v0.06.yaml csv:=tune_log.csv

v และ dist ส่งเป็น launch argument (ทับค่าในไฟล์) ส่วน tau/kp/accel ให้ใส่ในไฟล์พารามิเตอร์
เช่น params_best_v0.06.yaml แล้วส่งผ่าน profile_file
ถ้าต้องการถามระยะที่วัดจริงด้วยตลับเมตร ให้ใช้ ros2 run หรือ tune_sweep แทน
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('tb3_straight_tune'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('v', description='ความเร็วเป้าหมาย m/s (จำเป็น)'),
        DeclareLaunchArgument('dist', default_value='1.2', description='ระยะเป้าหมาย m'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ไฟล์พารามิเตอร์หลัก'),
        DeclareLaunchArgument('profile_file', default_value=default_params,
                              description='ไฟล์พารามิเตอร์ที่จูนแล้ว (ทับไฟล์หลัก) เช่น params_best_v0.06.yaml'),
        DeclareLaunchArgument('csv', default_value='', description='ไฟล์บันทึกผล (ว่าง = ไม่บันทึก)'),
        Node(
            package='tb3_straight_tune',
            executable='run_straight_tuned',
            name='run_straight_tuned',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                LaunchConfiguration('profile_file'),
                {
                    'v': ParameterValue(LaunchConfiguration('v'), value_type=float),
                    'dist': ParameterValue(LaunchConfiguration('dist'), value_type=float),
                    'csv': ParameterValue(LaunchConfiguration('csv'), value_type=str),
                    'ask_real': False,
                },
            ],
        ),
    ])
