#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        #ROSのイメージメッセージをbgrのMat型に変換
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv_img = bridge.imgmsg_to_cv2(msg)  #ROSのイメージメッセージをMat型に変換

#######　処理を追加してみよう
        gray_frame = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        n,binary_frame = cv2.threshold(gray_frame, 128, 255, cv2.THRESH_BINARY)
        
        #cv2.imshow("hogehoge", cv_img)
        cv2.imshow("window",binary_frame)
        cv2.waitKey(1)
        
    except Exception as e:
        print(e)

def main():
    rospy.init_node("camera_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
