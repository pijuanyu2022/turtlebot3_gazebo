#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 
                   
        except CvBridgeError as e:
            print(e)

        new_image = self.rescale_frame(cv_image)

        # BRG to HSV
        hsv = cv2.cvtColor(new_image, cv2.COLOR_BGR2HSV)

        # Set threshold
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 190])

        # add mask
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # create a circle in the center of the line
        h, w, d = new_image.shape
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(new_image, (cx, cy), 15, (0,0,255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("New Image window", new_image)
        cv2.waitKey(3)

    # smaller the window
    def rescale_frame(self, frame, scale=0.4):
        height = int(frame.shape[0]*scale)
        width = int(frame.shape[1]*scale)
        dimensions = (width, height)
        return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
    	rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)