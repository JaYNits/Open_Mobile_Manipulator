#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Int16MultiArray

class simple_class:

  def __init__(self):
    self.sub = rospy.Subscriber("servo_cmd",Int16MultiArray,self.callback)

  def callback(self,data):
    print(data.data)



def main(args):
  obc = simple_class()
  rospy.init_node('adafruit_servo', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)