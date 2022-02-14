from cmath import sqrt
import rospy
from gazebo_msgs.msg import ModelStates

uavStr = "iris_0"
carStr = "rover_moving"

def callback(msg):
    global total_score
    name = msg.name
    if (uavStr in name) and (carStr in name):
        uav_ind = name.index(uavStr)
        car_ind = name.index(carStr)
        uav_pos = msg.pose[uav_ind].position
        car_pos = msg.pose[car_ind].position
        distance = ((uav_pos.x-car_pos.x)**2+
               (uav_pos.y-car_pos.y)**2+
               (uav_pos.z-car_pos.z)**2 )**0.5
        if distance < 5.0:
            score = 1 - abs(distance-1)/4
        else:
            score = 0
        total_score = total_score + score
        print("[DEBUG]distance:{:f},score:{:f}".format(distance,total_score))
    else:
        print("[ERR]no ("+ carStr+") and ("+uavStr+") has been found.")

rospy.init_node("ScoreingSystemNode")
total_score = 0
while not rospy.is_shutdown():
    msg = rospy.wait_for_message("gazebo/model_states", ModelStates, timeout=None)
    callback(msg)
    rospy.sleep(0.5)