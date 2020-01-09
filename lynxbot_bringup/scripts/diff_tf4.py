#!/usr/bin/env python


import rospy
import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32

#############################################################################
class DiffTf:
#############################################################################

    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 36000))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.25)) # The wheel base width in meters
        self.publish_tf = int(rospy.get_param('~publish_tf', 0))
        self.base_frame_id = rospy.get_param('~base_frame_id','robot_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483647)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_fleft = None        # wheel encoder readings
        self.enc_fright = None
        self.enc_bleft = None        # wheel encoder readings
        self.enc_bright = None
        self.fleft = 0               # actual values coming back from robot
        self.fright = 0
        self.bleft = 0
        self.bright = 0
        self.lfmult = 0
        self.rfmult = 0
        self.lbmult = 0
        self.rbmult = 0
        self.prev_lfencoder = 0
        self.prev_rfencoder = 0
        self.prev_lbencoder = 0
        self.prev_rbencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("lfwheel", Int32, self.lfwheelCallback)
        rospy.Subscriber("rfwheel", Int32, self.rfwheelCallback)
        rospy.Subscriber("lbwheel", Int32, self.lbwheelCallback)
        rospy.Subscriber("rbwheel", Int32, self.rbwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
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
            
            # calculate odometry
            if self.enc_fleft == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (int((self.fleft + self.bleft)*0.5) - int((self.enc_fleft + self.enc_bleft)*0.5)) / self.ticks_meter
                d_right = (int((self.fright + self.bright)*0.5) - int((self.enc_fright + self.enc_bright)*0.5)) / self.ticks_meter
            self.enc_fleft = self.fleft
            self.enc_fright = self.fright
            self.enc_bleft = self.bleft
            self.enc_bright = self.bright

            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           
             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            if (self.publish_tf == 1):
                self.odomBroadcaster.sendTransform(
                    (self.x, self.y, 0),
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    self.base_frame_id,
                    self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
            


    #############################################################################
    def lfwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lfencoder > self.encoder_high_wrap):
            self.lfmult = self.lfmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lfencoder < self.encoder_low_wrap):
            self.lfmult = self.lfmult - 1
            
        self.fleft = 1.0 * (enc + self.lfmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lfencoder = enc
        
    #############################################################################
    def rfwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rfencoder > self.encoder_high_wrap):
            self.rfmult = self.rfmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rfencoder < self.encoder_low_wrap):
            self.rfmult = self.rfmult - 1
            
        self.fright = 1.0 * (enc + self.rfmult * (self.encoder_max - self.encoder_min))
        self.prev_rfencoder = enc

    #############################################################################
    def lbwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lbencoder > self.encoder_high_wrap):
            self.lbmult = self.lbmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lbencoder < self.encoder_low_wrap):
            self.lbmult = self.lbmult - 1
            
        self.bleft = 1.0 * (enc + self.lbmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lbencoder = enc
        
    #############################################################################
    def rbwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rbencoder > self.encoder_high_wrap):
            self.rbmult = self.rbmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rbencoder < self.encoder_low_wrap):
            self.rbmult = self.rbmult - 1
            
        self.bright = 1.0 * (enc + self.rbmult * (self.encoder_max - self.encoder_min))
        self.prev_rbencoder = enc
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
