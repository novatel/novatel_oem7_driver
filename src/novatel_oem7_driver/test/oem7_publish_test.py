#!/usr/bin/env python

import unittest
import rospy
import sys
import os

import oem7_decoder_test
#from __builtin__ import None

PKG = 'novatel_oem7_driver'
NAME = 'publishtest'

class Oem7PublishTest(unittest.TestCase):
    def __init__(self, *args):
            
        super(self.__class__, self).__init__(*args)
        rospy.init_node(NAME)
        
    def test_1_record_uut(self):
        """ 
        Purge existing bag, record new one.
        
        Fails if stale file cannot be purged.
        """
        
        if os.path.exists(self.uut_bag):
            os.remove(self.uut_bag)
        
        delay_sec = float(rospy.get_param('~delay_sec', 10.))
        rospy.sleep(delay_sec)
        
    def test_2_verify_recorded_topics(self):
        oem7_decoder_test.verify_bag_equivalency(self.ref_bag, self.uut_bag)     
        
        
if __name__ == '__main__':
    import rostest

    Oem7PublishTest.ref_bag = sys.argv[1] + ".bag"
    Oem7PublishTest.uut_bag = sys.argv[2] + "-test.bag"
    
    rostest.run(PKG, NAME, Oem7PublishTest, sysargs = sys.argv)
        
