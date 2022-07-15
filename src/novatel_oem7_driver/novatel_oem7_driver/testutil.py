################################################################################
# Copyright (c) 2021 NovAtel Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#################################################################################

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory

from launch.actions import OpaqueFunction
import launch.substitutions
import launch

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import launch_testing_ros


import os
import time

from launch.actions import DeclareLaunchArgument

import unittest
import launch_testing

from novatel_oem7_driver import rosbag_comparison


def launch_path(launch):
    """
    returns an absolute path to a launch file
    """
    
    return os.path.join(get_package_share_directory('novatel_oem7_driver'), "launch", launch )



def generate_test_description(name, topics):
    """ 
    Generates a launch_testing-compatible test description, which:
    - Sets up rosbag recording for the specified topic
    - Runs the driver using a file as input
    """
    
    oem7_file_name_arg = launch.substitutions.LaunchConfiguration('oem7_file_name')


    driver_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path('oem7_file.launch.py')),
        launch_arguments = {'oem7_file_name': oem7_file_name_arg}.items()
        )
    
    rosbag_desc = launch.actions.ExecuteProcess(
            cmd= ['ros2', 'bag', 'record', '-o', name] + topics,
            output='screen')
    
    
    return LaunchDescription([
        launch.actions.ExecuteProcess(cmd=["rm", "-rf", name], output='screen'),
        TimerAction(period=1.0, actions=[rosbag_desc]),
        TimerAction(period=3.0, actions=[driver_launch_desc]),
        launch_testing.actions.ReadyToTest()
        ])

    


class ConcurrentTestWorkaround(unittest.TestCase):
    def test_logging_output(self, proc_info, proc_output):
        print("Dummy concurrent test to supress spurious launch error")
        time.sleep(45)
        print("Dummy concurrent test finished")
        


class BagEquivalencyTest(unittest.TestCase):
    """
    Verifies that the content of reference and UUT bags are semantically identical.
    """
    
    def test_bag_equivalency(self,proc_info, proc_output, test_args):
        
        test_dir  = test_args['test_dir']
        test_name = test_args['test_name']

        uut_bag = os.getcwd() + '/' + test_name + '/' + test_name + '_0.db3'
        ref_bag = test_dir + '/' + test_name + '.db3'
        
        rosbag_comparison.verify_bag_equivalency(ref_bag, uut_bag)

