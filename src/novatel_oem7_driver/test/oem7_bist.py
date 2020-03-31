#!/usr/bin/env python

import unittest
import rospy
import os, sys

import oem7_message_test

PKG = 'novatel_oem7_driver'
NAME = 'oem7_bist'

class Oem7BIST(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        
        
    def test_1_recording(self):
        
        if os.path.exists(self.bist_bag):
            os.remove(self.bist_bag)
        
        delay_sec = float(rospy.get_param('~duration', 60.))
        print("Sleeping for {0} sec to allow topics to be recorded...".format(delay_sec))
        rospy.sleep(delay_sec)
        print("..done")
        
    def test_2_analysis(self):
        oem7_message_test.analyze_hz(self.bist_bag, output_csv = False)
        # Set output_csv = True to dump topic statistics to .csv file.
        
        
if __name__ == '__main__':
    
    Oem7BIST.bist_bag = sys.argv[1]
        
    import rostest
    rostest.run(PKG, NAME, Oem7BIST, sys.argv)
        
