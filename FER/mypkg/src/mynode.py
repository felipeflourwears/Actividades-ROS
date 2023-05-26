#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def radians_to_degrees(radians):
    return radians * 180 / 3.141592

def callback(msg, publisher):
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    
    roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
    grados = radians_to_degrees(yaw)
    
    publisher.publish(grados)  # Publicar la variable "grados" en el nuevo nodo
    
    rospy.loginfo("GRADOS: %f, RADIANES: %f", grados, yaw)

def listener():
    rospy.init_node('subscritor', anonymous=True)

    pub = rospy.Publisher('att_turtlebot3', Float64, queue_size=10)  # Crear publicador en el nuevo nodo
    
    rospy.Subscriber("odom", Odometry, callback, pub)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass