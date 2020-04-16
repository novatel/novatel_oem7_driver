#!/usr/bin/env python

################################################################################
# Copyright (c) 2020 NovAtel Inc.
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


import unittest
import rospy
import os, sys

import oem7_message_test

PKG = 'novatel_oem7_driver'
NAME = 'oem7_bist'

class Oem7BIST(unittest.TestCase):
    """
    Built-in Self-Test. Runs in parallel with a 'live' driver, supports correct recording of topics;
    and analyzes message rates.
    """
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        
        
    def test_1_recording(self):
        """
        Removes stale test .bag; delays execution of analysis until current recording is complete.
        """
        if os.path.exists(self.bist_bag):
            os.remove(self.bist_bag)
        
        delay_sec = float(rospy.get_param('~duration', 60.))
        print("Sleeping for {0} sec to allow topics to be recorded...".format(delay_sec))
        rospy.sleep(delay_sec)
        print("..done")
        
    def test_2_analysis(self):
        """
        Analyzes message rates.
        """
        oem7_message_test.analyze_hz(self.bist_bag, output_csv = False)
        # Set output_csv = True to dump topic statistics to .csv file.
        
        
if __name__ == '__main__':
    
    Oem7BIST.bist_bag = sys.argv[1]
        
    import rostest
    rostest.run(PKG, NAME, Oem7BIST, sys.argv)
        
