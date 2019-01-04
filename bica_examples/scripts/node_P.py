#!/usr/bin/env python

import rospy
from bica.bica import Component

class TestP(Component):
  def step(self):
    rospy.loginfo("[" +  rospy.get_name() + "] step")

if __name__ == '__main__':
  rospy.init_node('node_P', anonymous=False)
  rate = rospy.Rate(10) # 10hz

  test_p = TestP(10)

  while test_p.ok():
    pass
