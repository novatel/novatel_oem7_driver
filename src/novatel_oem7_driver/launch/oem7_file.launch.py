
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PKG = "novatel_oem7_driver"

def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg )
    

def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)

def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))

def get_override_params():    
    try:
        oem7_param_override_path = os.environ['NOVATEL_OEM7_DRIVER_PARAM_OVERRIDES_PATH']
    except KeyError: # No overrides specified.
        return {}
        
    return load_yaml(oem7_param_override_path)

def arg(name, default_value, description):
    return DeclareLaunchArgument(name = name, description = description, default_value = default_value)

    
def generate_launch_description():

    node = Node(
        package=PKG,
        namespace='novatel/oem7',
        name='main',
        executable='novatel_oem7_driver_exe',
        
        parameters=[
                    get_params("std_msg_handlers.yaml"    ),
                    get_params("std_oem7_raw_msgs.yaml"   ),
                    get_params("std_msg_topics.yaml"      ),
                    get_params("oem7_supported_imus.yaml" ),
                    {
                    'oem7_msg_decoder'   : 'Oem7MessageDecoder',
                    'oem7_max_io_errors' : 1,
                    'oem7_if'            : 'Oem7ReceiverFile',
                    'oem7_file_name'     : LaunchConfiguration('oem7_file_name'),
                    'oem7_publish_delay' : 0.01
                    },
                    get_override_params() # Must be last to override
                    ],
    
        output='screen',
        arguments=[('--ros-args', '--log_level', 'DEBUG')]
    )
    
    return LaunchDescription([
                             arg('oem7_file_name', None,  'Path to input file'), 
                             node
                             ])

