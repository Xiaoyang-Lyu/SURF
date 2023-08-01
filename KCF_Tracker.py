#!/usr/bin/env/python

import os
import rospy
import rospkg
import threading
from astra_common import *
from geometry_msgs.msg import Twist
from yahboomcar_msgs.msg import Position
from sensor_msgs.msg import CompressedImage, Image
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from yahboomcar_astra.cfg import ColorHSVConfig
# TODO: import PID
class Image_Converter:
# Rect selectRect 这是鼠标框选相关的变量
# Point origin;
# Rect result;
# bool select_flag = false;
# bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
# bool bBeginKCF = false;
# Mat rgbimage;
# Mat depthimage;

#下列是color的框选方法
    # def onMouse(self, event, x, y, flags, param):
    #         if event == 1:
    #             self.Track_state = 'init'
    #             self.select_flags = True
    #             self.Mouse_XY = (x, y)
    #         if event == 4:
    #             self.select_flags = False
    #             self.Track_state = 'mouse'
    #         if self.select_flags == True:
    #             self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
    #             self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
    #             self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])


    RGB_WINDOW = "rgb_img"
    DEPTH_WINDOW = "depth_img"
    min_Dist = 1.0
    linear_speed = 0.0
    rotation_speed = 0.0
    enable_get_depth = False
    dist_val[5]
    HOG = True
    FIXED_WINDOW = False
    MULTISCALE = True
    LAB = False
    center_x = 0
#   dynamic_reconfigure::Server<yahboomcar_astra::KCFTrackerPIDConfig> server;
#     dynamic_reconfigure::Server<yahboomcar_astra::KCFTrackerPIDConfig>::CallbackType f;
# TODO: 动态配置服务节点

# PID* linear_PID;
#     pid* angular_pid;
# TODO: PID    

    def __init__(self):
         # TODO: KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
        nodeName = "KCF_Tracker"
        rospy.init_node(nodeName,anonymous=False)
        rospy.on_shutdown(self.cancel)

        self.linear_KP = 0.9
        self.linear_KI = 0.0
        self.linear_KD = 0.1
        self.angular_KP = 0.5
        self.angular_KI = 0.0
        self.angular_KD = 0.2
#  this->linear_PID = new PID(linear_KP, linear_KI, linear_KD);
#     this->angular_PID = new PID(angular_KP, angular_KI, angular_KD);
# TODO: PID init
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", imageCb, queue_size = 1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image", depthCb, queue_size = 1)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
# TODO:   f = boost::bind(&ImageConverter::PIDcallback, this, _1, _2);
        pub.publish(Twist())

# TODO: server.setCallback(f);
        cv.nameWindow(RRB_WINDOW, cv.WINDOW_AUTOSIZE)
    
# TODO:  this->linear_PID->Set_PID(linear_KP, linear_KI, linear_KD);
    # this->angular_PID->Set_PID(angular_KP, angular_KI, angular_KD);

    # def __del__(self):
    #     # Close ROS node
    #     n.shutdown()
    #     # Close publisher
    #     pub.shutdown()
    #     # Close subscribers
    #     image_sub_.shutdown()
    #     depth_sub_.shutdown()
    #     # Delete PID objects
    #     del self.linear_PID
    #     del self.angular_PID
    #     # Destroy RGB window
    # color里没有析构方法,也可能是在color的cancel里

    def imageCb(self, msg):
        # Convert ROS image message to OpenCV image object
        try:
            cv_ptr = CvBridge().cv2_to_imgmsg(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return
        # Copy the converted image to rgbimage variable
        rgbimage = cv_ptr.image.copy()
        # Set a mouse callback function to select target area
        setMouseCallback(RGB_WINDOW, self.onMouse, 0)
        # If bRenewROI is true, initialize the tracker with the selected area
        if self.bRenewROI:
            if self.selectRect.width <= 0 or self.selectRect.height <= 0:
                self.bRenewROI = False
                return
            tracker.init(self.selectRect, rgbimage)
            self.bBeginKCF = True
            self.bRenewROI = False
            self.enable_get_depth = False
        # If bBeginKCF is true, update the tracking result and draw a rectangle and a circle on the image
        if self.bBeginKCF:
            result = tracker.update(rgbimage)
            rectangle(rgbimage, result, (0, 255, 255), 1, 8)
            circle(rgbimage, (result.x + result.width / 2, result.y + result.height / 2), 3, (0, 0, 255), -1)
        # If bBeginKCF is false, draw a rectangle on the image to indicate the selected area
        else:
            rectangle(rgbimage, self.selectRect, (255, 0, 0), 2, 8, 0)
        # Convert OpenCV image object to ROS image message and publish it to /KCF_image topic
        try:
            kcf_imagemsg = CvBridge().cv2_to_imgmsg(rgbimage, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return
        image_pub_.publish(kcf_imagemsg)
        # Show the image on RGB_WINDOW window
        imshow(RGB_WINDOW, rgbimage)
        # Get the user input and perform different actions according to the key pressed
        action = waitKey(1) & 0xFF
        if action == ord('q') or action == ACTION_ESC:
            self.Cancel()
        elif action == ord('r'):
            self.Reset()
        elif action == ACTION_SPACE:
            self.enable_get_depth = True

    def depthCb(self, msg):
        # Convert ROS image message to OpenCV image object
        try:
            cv_ptr = CvBridge().cv2_to_imgmsg(msg, "32FC1")
            depthimage = cv_ptr.image.copy()
        except CvBridgeError as e:
            rospy.logerr("Could not convert from '%s' to '32FC1'.", msg.encoding)
            return
        # If enable_get_depth is true, get the depth information and calculate the speed
        if self.enable_get_depth:
            # Calculate the center point coordinates of the tracking result
            center_x = int(result.x + result.width / 2)
            center_y = int(result.y + result.height / 2)
            # Get the five depth values around the center point and store them in an array
            dist_val = [0] * 5
            dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5)
            dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5)
            dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5)
            dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5)
            dist_val[4] = depthimage.at<float>(center_y, center_x)
            # Calculate the average depth value and exclude the unreasonable values
            distance = 0
            num_depth_points = 5
            for i in range(5):
                if dist_val[i] > 0.4 and dist_val[i] < 10.0:
                    distance += dist_val[i]
                else:
                    num_depth_points -= 1
            distance /= num_depth_points
            # If there are valid depth values, calculate the linear speed according to the distance and minDist
            if num_depth_points != 0:
                if abs(distance - minDist) < 0.1:
                    linear_speed = 0
                else:
                    linear_speed = -self.linear_PID.compute(minDist, distance)
            # Calculate the angular speed according to the center_x and 320
            rotation_speed = self.angular_PID.compute(320 / 100.0, center_x / 100.0)
            # If the angular speed is too small, set it to zero
            if abs(rotation_speed) < 0.1:
                rotation_speed = 0
            # Create a Twist message object and assign the linear speed and angular speed to it
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = rotation_speed
            # Publish the twist message to /cmd_vel topic to control the robot movement
            pub.publish(twist)
            # Print the speed, distance and center point information
            rospy.logwarn("linear = %.3f,angular =  %.3f", linear_speed, rotation_speed)
            rospy.logerr("distance = %.3f,center_x =  %d", distance, center_x)
        # Wait for user input and refresh the window
        waitKey(1)

# A helper function of Cancel()
    def Reset(self):
        self.bRenewROI = False
        self.bBeginKCF = False
        # 框选reset
        # self.selectRect.x = 0
        # self.selectRect.y = 0
        # self.selectRect.width = 0
        # self.selectRect.height = 0
        self.linear_speed = 0
        self.rotation_speed = 0
        self.enable_get_depth = False
        self.linear_PID.reset()
        self.angular_PID.reset()
        self.pub.publish(geometry_msgs.Twist())

    def Cancel(self):
        self.Reset()
        ros.sleep(0.5)
        # 感觉下边的没啥用
        # del RGB_WINDOW
        # del DEPTH_WINDOW
        # del self.linear_PID
        # del self.angular_PID
        # n.shutdown()
        # pub.shutdown()
        # image_sub_.shutdown()
        # depth_sub_.shutdown()
        cv2.destroyWindow(RGB_WINDOW)

    def PIDcallback(self, config, level):
        rospy.loginfo("linear_PID: %f %f %f", config.linear_Kp, config.linear_Ki, config.linear_Kd)
        rospy.loginfo("angular_PID: %f %f %f", config.angular_Kp, config.angular_Ki, config.angular_Kd)
        self.minDist = config.minDist
        # self.linear_PID.Set_PID(float(config.linear_Kp), float(config.linear_Ki), float(config.linear_Kd))
        # self.angular_PID.Set_PID(float(config.angular_Kp), float(config.angular_Ki), float(config.angular_Kd))
        # self.linear_PID.reset()
        # self.angular_PID.reset()
        # TODO: PID相关

if __name__ == '__main__':
    nodeName = "KCF_Tracker"
    rospy.init_node(nodeName, anonymous = False)
    rospy.on_shutdown(self.Cancel)
    imageConverter = Image_Converter()
    rospy.spin()