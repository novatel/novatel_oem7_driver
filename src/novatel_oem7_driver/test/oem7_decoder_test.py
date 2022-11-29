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


import rosbag
import rospy

import os
import sys

import traceback
from docutils.nodes import topic


def get_topic_list(bag_name):
    """ Return a list of topics stored in this bag """
    bag = rosbag.Bag(bag_name, 'r')
    return bag.get_type_and_topic_info()[1].keys()

def make_msg_gen(bag_name, topic):
    """ Generates a sequence of messages in the topic from the bag """
    bag = rosbag.Bag(bag_name, 'r')
    for top, msg, t in bag.read_messages():
        if top == topic:
            yield msg

""" The functions below check for type of class objects in nested ROS messages """
primitive_types  = (int, float, bool, str)
collection_types = (list, tuple)
float_type = (float)
def is_primitive(item):  return isinstance(item, primitive_types)
def is_collection(item): return isinstance(item, collection_types)
def is_float_type(item): return isinstance(item, float_type)

def is_close(a, b, rel_tol=1e-9, abs_tol=0.0):
    """ Checks equivalency between two float numbers - Following rules of Python PEP485 - https://peps.python.org/pep-0485/#proposed-implementation"""
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def item_gen(item):
    """ Generates values of various types (int, bool, str, float) from a nested bag object """
    for member in dir(item):
        if member.startswith("_"):  # exclude private stuff, it's irrelevant and blows up getattr.
            continue
        value = getattr(item, member)
        if (is_primitive(value)):
            yield value
        elif (repr(type(value)).startswith("<class")):
            for c in item_gen(value):
                yield c
        elif (is_collection(value)):
            for elt in value:
                if is_primitive(elt):
                    continue
                else:
                    for c in item_gen(value):
                        yield c
            pass  # methods, classmethods, etc.

def compare(ref_msg, uut_msg):
    """
    Compares contents of two bags; fails if the contents are not identical (except for ROS seqno, timestamp).
    """
    # Supress seqno, timestamp
    ref_msg.header.seq = 0
    uut_msg.header.seq = 0
    ref_msg.header.stamp = None
    uut_msg.header.stamp = None

    if (ref_msg != uut_msg):
        ref_value_gen = item_gen(ref_msg)
        uut_value_gen = item_gen(uut_msg)
        for ref_value in ref_value_gen:
            uut_value = next(uut_value_gen)
            if is_float_type(ref_value):
                if not is_close(ref_value, uut_value):
                    rospy.logerr("Messages do not match:")
                    rospy.logerr("Ref:\r\n" + str(ref_msg))
                    rospy.logerr("UUT:\r\n" + str(uut_msg))
                    return False;
            else:
                if not(ref_value == uut_value):
                    rospy.logerr("Messages do not match:")
                    rospy.logerr("Ref:\r\n" + str(ref_msg))
                    rospy.logerr("UUT:\r\n" + str(uut_msg))
                    return False;
    return True
    
def verify_bag_equivalency(ref_bag, uut_bag):
  """
  Verifies that two bags contain semantically identical sequence of messages.
  """

  ref_topics = get_topic_list(ref_bag)
  print(ref_topics)
  for topic in ref_topics:
      msgno = 0
      uut_gen = make_msg_gen(uut_bag, topic)
      ref_gen = make_msg_gen(ref_bag, topic)
      for ref_msg in ref_gen:
        uut_msg = next(uut_gen)
        if not compare(ref_msg, uut_msg):
            rospy.logerr("Topic: {} Msg No: {}".format(topic, msgno))
            assert False
            
        msgno += 1
      
      rospy.loginfo("Verified {} '{}' messages".format(msgno, topic))
      # Check for presence of unexpected messages
      unexpected_messages = 0
      try:
          while True:
              uut_top, uut_msg, uut_t = next(uut_gen)
              rospy.logerr("Unexpected message")
              rospy.logerr(uut_msg)
              unexpected_messages += 1
  
      except StopIteration:
        pass # Normal
      
      except:
        traceback.print_exc()
        assert(False)
        
      assert(unexpected_messages == 0)
  
if __name__ == '__main__':
    mgen = make_msg_gen(sys.argv[1], sys.argv[2])
    for m in mgen:
        print(m)
        inspect_item(m)
 
