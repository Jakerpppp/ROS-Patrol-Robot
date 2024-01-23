#!/usr/bin/env python3
# coding=utf-8
import rospy
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from yolov4 import Detector
import smach
from geometry_msgs.msg import Point
from std_msgs.msg import String

'''Parent Class for the specific YOLO detection to each room - Each Room YOLO Inherits this class with specific 
details about the room and what it should detect '''


class YOLOState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['result', 'feedback'],
                             output_keys=['result', 'feedback'])
        self.cv_image = None
        self.bridge = CvBridge()
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k22001853/Documents/Robotics/ros_ws/src/second_coursework/config/coco.data')
        self.tfb = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfb)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)

    # ud is userdata
    def execute(self, ud):
        while not self.preempt_requested():
            if self.cv_image is not None:
                img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
                detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
                self.process_detections(ud, detections)
        self.service_preempt()
        self.reset_room()
        return 'succeeded'

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process_detections(self, ud, detections):
        for detection in detections:
            print(detection.class_name)
            if detection.class_name == 'cat':
                self.cat_found = True
            if detection.class_name == 'dog':
                self.dog_found = True

        if self.cat_found and self.dog_found:
            rule_broken = 2
            self.get_current_coordinates()
            coords = Point(self.x, self.y, self.z)
            ud.feedback.append([coords, rule_broken])
            ud.result[1] += 1
            self.tts_pub.publish("Could the Cat or the Dog please leave immediately")
            self.cat_found = False
            self.dog_found = False

    def get_current_coordinates(self):
        transformation = self.tfb.lookup_transform('map', 'base_link', rospy.Time(0))
        self.x = transformation.transform.translation.x
        self.y = transformation.transform.translation.y
        self.z = transformation.transform.translation.z

    def reset_room(self):
        self.cat_found = False
        self.dog_found = False
