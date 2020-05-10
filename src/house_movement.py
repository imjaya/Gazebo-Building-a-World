#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
yaw = 0

def poseCallback(pose_message):
    global x, y, yaw

    x = pose_message.pose.pose.position.x
    y = pose_message.pose.pose.position.y
    q_x = pose_message.pose.pose.orientation.x
    q_y = pose_message.pose.pose.orientation.y
    q_z = pose_message.pose.pose.orientation.z
    q_w = pose_message.pose.pose.orientation.w
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

def move(speed, distance, forward):
    velocity_message = Twist()
    global x, y

    x0 = x
    y0 = y

    if forward:
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    pub_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(pub_topic, Twist, queue_size = 10)

    while True:
        rospy.loginfo('Turtlebot is moving')
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        distance_moved += abs(math.sqrt(((x - x0)**2) + ((y - y0)**2)))

        if not (distance_moved < distance):
            rospy.loginfo("Turtlebot has reached")
            break

    print("Distance Moved = {}".format(distance_moved))
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)
    print("Current Position and Orientation:")
    print("X = {}".format(x))
    print("Y = {}".format(y))
    print("YAW = {}".format(yaw))

def rotate(angular_speed, angle_to_rotate, clockwise):
    velocity_message = Twist()
    global yaw

    theta0 = yaw

    if clockwise:
        velocity_message.angular.z = -math.radians(abs(angular_speed))
    else:
        velocity_message.angular.z = math.radians(abs(angular_speed))

    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    pub_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(pub_topic, Twist, queue_size = 10)

    tn = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo('Turtlebot is rotating')
        velocity_publisher.publish(velocity_message)

        tnp1 = rospy.Time.now().to_sec()
        loop_rate.sleep()

        current_angle = abs((tnp1 - tn) * math.radians(angular_speed))

        if not (current_angle < math.radians(angle_to_rotate)):
            rospy.loginfo("turtlebot has rotated")
            break
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    print("Angle rotated = {}".format(math.degrees(abs(current_angle))))
    print("Current Position and Orientation:")
    print("X = {}".format(x))
    print("Y = {}".format(y))
    print("YAW = {}".format(yaw))

def arc(speed, angular_speed, angle_to_rotate, clockwise):
    velocity_message = Twist()
    global x, y, yaw

    theta0 = yaw

    if clockwise:
        velocity_message.linear.x = abs(speed)
        velocity_message.angular.z = -abs(math.radians(abs(angular_speed)))
    else:
        velocity_message.linear.x = abs(speed)
        velocity_message.angular.z = abs(math.radians(abs(angular_speed)))

    angle_moved = 0.0
    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    pub_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(pub_topic, Twist, queue_size = 10)

    tn = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo('Turtlebot is making an arc')
        velocity_publisher.publish(velocity_message)

        tnp1 = rospy.Time.now().to_sec()
        loop_rate.sleep()

        current_angle = abs((tnp1 - tn) * math.radians(angular_speed))

        if not (current_angle < math.radians(angle_to_rotate)):
            rospy.loginfo("turtlebot has made an arc")
            break
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)
    print("Arc angle rotated = {}".format(math.degrees(abs(current_angle))))
    print("Current Position and Orientation:")
    print("X = {}".format(x))
    print("Y = {}".format(y))
    print("YAW = {}".format(yaw))

def travel_the_house():

    print("------------------Starting to travel the house------------------")

    rotation_speed = 20
    movement_speed = 0.4
    
    rotate(rotation_speed, 90, False)
    move(movement_speed, 150, True)
    print("------------------Entered the house------------------")
    rotate(rotation_speed, 88, True)
    move(movement_speed, 30, True)
    print("------------------Going to the far end of the house------------------")
    rotate(rotation_speed, 90, False)
    move(movement_speed, 2350, True)
    print("------------------Reached the far end of the house------------------")
    rotate(rotation_speed, 200, False)
    
    print("------------------Finished travelling the house------------------")
    pass

def come_to_the_livingroom():

    print("------------------Coming to the living room------------------")

    rotation_speed = 20
    movement_speed = 0.4
    
    move(movement_speed, 2200, True)
    rotate(rotation_speed, 90, False)

    print("------------------Came to the living room------------------")
    pass

def make_initials():

    rotation_speed = 10
    movement_speed = 0.1
    rotation_speed_multiplier = 3
    arc_move_speed_multiplier = 3
    arc_rotate_speed_multiplier = 2

    print ("Starting the Drawing of initials - ( SV )")

    # Drawing S
    rotate(rotation_speed, 90, False)
    arc(movement_speed * arc_move_speed_multiplier, rotation_speed * arc_rotate_speed_multiplier, 190, False)
    rotate(rotation_speed, 40, False)
    move(movement_speed, 50.0, True)
    rotate(rotation_speed, 40, True)
    arc(movement_speed * arc_move_speed_multiplier, rotation_speed * arc_rotate_speed_multiplier, 190, True)
    # Moving to the base of the S
    rotate(rotation_speed * rotation_speed_multiplier, 180, False)
    arc(movement_speed * arc_move_speed_multiplier, rotation_speed * arc_rotate_speed_multiplier, 90, False)
    # Moving to draw V
    move(movement_speed, 40.0, True)
    # Drawing V
    rotate(rotation_speed * rotation_speed_multiplier, 100, False)
    move(movement_speed, 30.0, True)
    rotate(rotation_speed * rotation_speed_multiplier, 180, False)
    move(movement_speed, 30.0, True)
    rotate(rotation_speed * rotation_speed_multiplier, 115, False)
    move(movement_speed, 30.0, True)
    # Moving tho the base of V
    rotate(rotation_speed * rotation_speed_multiplier, 180, False)
    move(movement_speed, 30.0, True)
    rotate(rotation_speed * rotation_speed_multiplier, 65, True)
    # Moving to the center of the initials
    move(movement_speed, 8.0, True)
    rotate(rotation_speed, 85, True)

    print ("Finished the Drawing of initials - ( SV )")

def go_around_the_damn_house_and_dance():

    time.sleep(10)
    travel_the_house()
    time.sleep(2)
    come_to_the_livingroom()
    time.sleep(2)
    make_initials()
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('house_movement', anonymous = True)

        pub_topic = '/cmd_vel'
        velocity_publisher = rospy.Publisher(pub_topic, Twist, queue_size = 10)

        pos_topic = '/odom'
        pose_subscriber = rospy.Subscriber(pos_topic, Odometry, poseCallback)

        print("------------------Here we Gooooo !!------------------")

        go_around_the_damn_house_and_dance()

    except rospy.ROSInterruptException:
        print("------------------So... We are Done------------------")
        rospy.loginfo("Main House Movement Node Terminated")