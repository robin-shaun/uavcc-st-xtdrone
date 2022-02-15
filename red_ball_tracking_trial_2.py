import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16, Float32, String

Kp_x = 1.5
Kp_yaw = 0.01
Kp_z = 1
target_center_u = 640 / 2 
offset = 0.3
target_distance = 1 - offset
target_height = 0.5 

def local_pose_callback(msg):
    global twist
    twist.linear.z = Kp_z * (target_height - msg.pose.position.z)    

def center_u_callback(msg):
    global twist
    twist.angular.z = Kp_yaw * (target_center_u - msg.data)

def center_depth_callback(msg):
    global twist
    twist.linear.x = Kp_x * (msg.data - target_distance)

if __name__ == "__main__":
    rospy.init_node('red_ball_tracking')
    cmd_vel_flu_pub = rospy.Publisher('/xtdrone/iris_0/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/iris_0/cmd',String, queue_size=3)
    rospy.Subscriber("/iris_0/ball_center_u", Int16,callback=center_u_callback, queue_size=1)   
    rospy.Subscriber("/iris_0/ball_center_depth", Float32, callback=center_depth_callback, queue_size=1)
    rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, local_pose_callback, queue_size=1)
                                
    twist = Twist()
    rate = rospy.Rate(50)
    arm_times = 0
    while not rospy.is_shutdown():
        cmd_vel_flu_pub.publish(twist)
        if(twist.linear.z > 0.3 and arm_times < 10):
            cmd_pub.publish("OFFBOARD")
            cmd_pub.publish("ARM")
            arm_times = arm_times + 1
        rate.sleep()