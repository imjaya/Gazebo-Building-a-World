#! /usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def img_callback(ros_image):
    global bridge

    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print e

    resized = cv2.resize(cv_image, (0,0), fx = 0.60, fy = 0.60)
    cv2.imshow("Turtlebot's View", resized)

    cv2.waitKey(1)

#def depth_img_callback(ros_depth_image):
#    global bridge
#
#    try:
#        cv_depth_image = bridge.imgmsg_to_cv2(ros_depth_image)
#    except CvBridgeError as e:
#        print e
#
#    resized_depth_img = cv2.resize(cv_depth_image, (0,0), fx = 0.60, fy = 0.60)
#    cv2.imshow("Turtlebot's Depth View", resized_depth_img)
#
#    cv2.waitKey(1)

if __name__ == '__main__':

    try:
        rospy.init_node('camera_stream_node', anonymous = True)

        cam_topic = "/camera/rgb/image_raw"
        image_sub = rospy.Subscriber(cam_topic, Image, img_callback)

        #depth_topic = "/camera/depth/image_raw"
        #depth_img = rospy.Subscriber(depth_topic, Image, depth_img_callback)

        print("------------------Started Camera------------------")
        rospy.spin()

        if rospy.is_shutdown:
            print("------------------Shutting down Camera------------------")
            cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        rospy.loginfo("Camera Stream is Terminated")