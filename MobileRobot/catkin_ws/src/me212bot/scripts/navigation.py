#!/usr/bin/python


import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from me212base.msg import WheelVelCmd
import helper

class ObjectNavigator():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.object_sub = rospy.Subscriber("object2camera", Pose, self.object_callback, queue_size=1)

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size=1)

        self.thread = threading.Thread(target = self.navi_loop)

        self.thread.start()

        rospy.sleep(1)

    def object_callback(self, msg):
        # pose of object w.r.t base
        object2base = helper.poseTransform(helper.pose2list(msg), homeFrame='/camera', targetFrame='base_link', listener=self.listener)
        pubFrame(self.br, pose=object2base, frame_id='/object', parent_frame_id='/base_link', npub=1)

    def navi_loop(self):
        robot_pos2d = [0, 0]
        thres_dist = 0.1
        thres_angle = 0.2
        k = 0.5
        r = 0.037
        b = 0.225

        wv = WheelVelCmd()

        while not rospy.is_shutdown():

            object_pose3d = helper.lookupTransformList('/base_link', '/object', self.listener)

            if object_pose3d is None:
                print 'object not in view'
                wv.desiredWV_R = 0 
                wv.desiredWV_L = 0
                self.velcmd_pub.publish(wv)
                continue

            object_pos2d = np.array(robot_pose3d[0:2])

            dist = np.linalg.norm(object_pos2d - robot_pos2d)
            angle = np.arctan2(object_pos2d[1] - robot_pos2d[1], object_pos2d[0] - robot_pos2d[0])

            if dist <= thres_dist and abs(angle) <= thres_angle:
                print 'Get close enough to the object'
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0
            elif dist > thres_dist and abs(angle) <= thres_angle:
                wv.desiredWV_R = 0.5
                wv.desiredWV_L = 0.5
            elif dist <= thres_dist and abs(angle) > thres_angle:
                wv.desiredWV_R = 0.2*np.sign(angle)
                wv.desiredWV_L = -0.2*np.sign(angle)
            else:
                w = k*angle
                vel = 0.5
                wv.desiredWV_R = (vel+w*b)/r
                wv.desiredWV_L = (vel-w*b)/r 

            self.velcmd_pub.publish(wv)
    
def main():
    rospy.init_node('me212_robot', anonymous=True)
    april_navi = ObjectNavigator()
    rospy.spin()

if __name__=='__main__':
main()