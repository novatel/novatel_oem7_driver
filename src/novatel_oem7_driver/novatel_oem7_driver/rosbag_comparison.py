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



import os
import sys

import traceback


import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def get_topic_list(bag_name):
    """ Return a list of topics stored in this bag """
    
    conn = sqlite3.connect(bag_name)
    cursor = conn.cursor()

    topics_data = cursor.execute("SELECT id, name, type FROM topics").fetchall()
    topic_info = {name_of:(id_of, type_of) for id_of,name_of,type_of in topics_data}
    
    return topic_info
     


def get_topic_messages(bag_name, topic):
    """
    Returns a list of messages for a specific topic in this bag
    """
    
    conn = sqlite3.connect(bag_name)
    cursor = conn.cursor()

    # Get from the db
    rows = cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic[0])).fetchall()
    data =  [deserialize_message(data, get_message(topic[1])) for timestamp,data in rows]
    return (m for m in data) 


def compare_messages(ref_msg, uut_msg):
    """
    Compares a reference message to a UUT message: 
    fails if the fields are not identical (except for ROS timestamp).
    """
    # Supress seqno, timestamp
    t = type(ref_msg.header.stamp)
    ref_msg.header.stamp = t()
    uut_msg.header.stamp = t()
    
    if(ref_msg != uut_msg):
        print("Messages do not match:")
        print("Ref:\r\n" + str(ref_msg))
        print("UUT:\r\n" + str(uut_msg))
        return False
    
    return True
    


def verify_bag_equivalency(ref_bag, uut_bag):
  """
  Verifies that two bags contain semantically identical sequence of messages.
  """

  ref_topics = get_topic_list(ref_bag)
  for topic in ref_topics.keys():
      
      print(topic)
      ref_msgs = get_topic_messages(ref_bag, ref_topics[topic])
      uut_msgs = get_topic_messages(uut_bag, ref_topics[topic])
      
      msgno = 0
      for ref_msg in ref_msgs:
        uut_msg = next(uut_msgs)
        if not compare_messages(ref_msg, uut_msg):
            print("Topic: {} Msg No: {}".format(topic, msgno))
            assert False
            
        msgno += 1
      
      print("Verified {} '{}' messages".format(msgno, topic))
      # Check for presence of unexpected messages
      unexpected_messages = 0
      try:
          while True:
              uut_msg = next(uut_msgs)
              print("Unexpected message")
              print(uut_msg)
              unexpected_messages += 1
  
      except StopIteration:
        pass # Normal
      
      except:
        traceback.print_exc()
        assert(False)
        
      assert(unexpected_messages == 0)

