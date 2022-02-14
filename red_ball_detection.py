# import the necessary packages
import imutils
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int16
from cv_bridge import CvBridge
bridge = CvBridge()

# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points
redLower = (0, 43, 43)
redUpper = (10, 255, 255)

def detection(color_img, depth_img):
    blurred = cv2.GaussianBlur(color_img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "red", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, redLower, redUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    center_depth = np.sum(mask * depth_img) / np.count_nonzero(mask) / 255
    # cv2.imshow("mask",mask)
    # cv2.waitKey(1)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 20:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(color_img, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(color_img, center, 5, (0, 0, 255), -1)
            print(center)
            return color_img, center[0], center_depth  


def color_img_callback(msg):
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_img_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth_img = np.nan_to_num(depth_img)

if __name__ == "__main__":
    rospy.init_node("red_ball_detection")
    color_img_sub = rospy.Subscriber("/iris_0/realsense/depth_camera/color/image_raw", Image, callback=color_img_callback, queue_size=1)
    depth_img_sub = rospy.Subscriber("/iris_0/realsense/depth_camera/depth/image_raw", Image, callback=depth_img_callback, queue_size=1)
    img_processed_pub = rospy.Publisher("/iris_0/image_with_circle", Image, queue_size=1)
    center_u_pub = rospy.Publisher("/iris_0/ball_center_u",Int16, queue_size=1)
    center_depth_pub = rospy.Publisher("/iris_0/ball_center_depth",Float32, queue_size=1)
    img_processed = Image()
    center_u = Int16()
    center_depth = Float32()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            cv_image, center_u.data, center_depth.data = detection(color_img, depth_img)
            img_processed = bridge.cv2_to_imgmsg(cv_image, "bgr8")
            img_processed.header.stamp = rospy.Time.now()
        except:
            continue
        img_processed_pub.publish(img_processed)
        center_u_pub.publish(center_u)
        center_depth_pub.publish(center_depth)
        rate.sleep()
        