#!/usr/bin/env python

import rospy
import roslib
from math import sin, cos, pi

from std_msgs.msg import Float32

#############################################################################
class Stats:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("stats")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',30.0)  # the rate at which to publish the transform
        self.ticks_meter = 33000 #float(rospy.get_param('~ticks_meter', 36000))  
        self.base_width = 0.32
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        self.prev_vel_lf = 0
        self.prev_vel_rf = 0
        self.prev_vel_lb = 0
        self.prev_vel_rb = 0
        self.curr_vel_lf = 0
        self.curr_vel_rf = 0
        self.curr_vel_lb = 0
        self.curr_vel_rb = 0
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("lfspeed", Float32, self.lfaccelCallback)
        rospy.Subscriber("rfspeed", Float32, self.rfaccelCallback)
        rospy.Subscriber("lbspeed", Float32, self.lbaccelCallback)
        rospy.Subscriber("rbspeed", Float32, self.rbaccelCallback)
        self.acclfPub = rospy.Publisher("lfaccel", Float32, queue_size=10)
        self.accrfPub = rospy.Publisher("lbaccel", Float32, queue_size=10)
        self.acclbPub = rospy.Publisher("rfaccel", Float32, queue_size=10)
        self.accrbPub = rospy.Publisher("rbaccel", Float32, queue_size=10)
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            accel_lf = Float32()
            accel_rf = Float32()
            accel_lb = Float32()
            accel_rb = Float32()
            accel_lf.data = (self.curr_vel_lf - self.prev_vel_lf) / elapsed
            accel_rf.data = (self.curr_vel_rf - self.prev_vel_rf) / elapsed
            accel_lb.data = (self.curr_vel_lb - self.prev_vel_lb) / elapsed
            accel_rb.data = (self.curr_vel_rb - self.prev_vel_rb) / elapsed

            self.acclfPub.publish(accel_lf)
            self.accrfPub.publish(accel_rf)
            self.acclbPub.publish(accel_lb)
            self.accrbPub.publish(accel_rb)
            self.prev_vel_lf = self.curr_vel_lf
            self.prev_vel_rf = self.curr_vel_rf
            self.prev_vel_lb = self.curr_vel_lb
            self.prev_vel_rb = self.curr_vel_rb
            


    #############################################################################
    def lfaccelCallback(self, msg):
    #############################################################################
        vel = msg.data
        self.curr_vel_lf = vel
        
    #############################################################################
    def rfaccelCallback(self, msg):
    #############################################################################
        vel = msg.data
        self.curr_vel_rf = vel

    #############################################################################
    def lbaccelCallback(self, msg):
    #############################################################################
        vel = msg.data
        self.curr_vel_lb = vel
        
    #############################################################################
    def rbaccelCallback(self, msg):
    #############################################################################
        vel = msg.data
        self.curr_vel_rb = vel
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        stats = Stats()
        stats.spin()
    except rospy.ROSInterruptException:
        pass
