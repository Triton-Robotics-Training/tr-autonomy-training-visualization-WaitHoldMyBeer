import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from tr_messages.msg import DetWithImg
from tr_messages.msg import SimGroundTruth


class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.det_subscription = self.create_subscription(
            DetWithImg,
            '/detections',
            self.handle_detection, 
            1
        )
        self.camera_subscription = self.create_subscription(
           SimGroundTruth,
           '/simulation/ground_truth',
           self.ground_truth_callback,
           1
        )

        self.det_subscription

    def handle_detection(self, msg):
        #the detected_panel transform initialization
        detected_panel = TransformStamped()

        #create header
            #timestamp
            #target camera_frame
            #source detected_panel
        detected_panel.header.stamp = self.get_clock().now().to_msg()
        detected_panel.header.frame_id = 'camera_frame'
        detected_panel.child_frame_id = 'detected_panel'

        #Send the translation position
        detected_panel.transform.translation.x = msg.detection_info.detections[0].results[0].pose.pose.position.x
        detected_panel.transform.translation.y = msg.detection_info.detections[0].results[0].pose.pose.position.y
        detected_panel.transform.translation.z = msg.detection_info.detections[0].results[0].pose.pose.position.z
        
        #Send the rotation position
        detected_panel.transform.rotation.x = msg.detection_info.detections[0].results[0].pose.pose.orientation.x
        detected_panel.transform.rotation.y = msg.detection_info.detections[0].results[0].pose.pose.orientation.y
        detected_panel.transform.rotation.z = msg.detection_info.detections[0].results[0].pose.pose.orientation.z
        detected_panel.transform.rotation.w = msg.detection_info.detections[0].results[0].pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(detected_panel)



        #Read content and assign it with corresponding tf variables
    
    def ground_truth_callback(self, msg):
        #assign to transform stamped types
        camera_frame = TransformStamped()
        panel_0 = TransformStamped()
        panel_1= TransformStamped()
        panel_2= TransformStamped()
        panel_3= TransformStamped()
        panels = [panel_0, panel_1, panel_2, panel_3]

        #timestamp
        camera_frame.header.stamp = panel_0.header.stamp = panel_1.header.stamp = panel_2.header.stamp = panel_3.header.stamp = self.get_clock().now().to_msg()
        camera_frame.header.frame_id = panel_0.header.frame_id = panel_1.header.frame_id = panel_2.header.frame_id = panel_3.header.frame_id = 'map'
        camera_frame.child_frame_id = 'camera_frame'
        for i in range(4):
            panels[i].child_frame_id = f"panel_{i}"
        
        camera_frame.transform.translation.x = msg.primary_robot.camera_pose.position.x
        camera_frame.transform.translation.y = msg.primary_robot.camera_pose.position.y
        camera_frame.transform.translation.z = msg.primary_robot.camera_pose.position.z

        camera_frame.transform.rotation.x = msg.primary_robot.camera_pose.orientation.x
        camera_frame.transform.rotation.y = msg.primary_robot.camera_pose.orientation.y
        camera_frame.transform.rotation.z = msg.primary_robot.camera_pose.orientation.z
        camera_frame.transform.rotation.w = msg.primary_robot.camera_pose.orientation.w

        self.tf_broadcaster.sendTransform(camera_frame)

        for i in range(4):
            panels[i].transform.translation.x = msg.secondary_robot.armor_panel_poses[i].position.x
            panels[i].transform.translation.y = msg.secondary_robot.armor_panel_poses[i].position.y
            panels[i].transform.translation.z = msg.secondary_robot.armor_panel_poses[i].position.z

            panels[i].transform.rotation.x = msg.secondary_robot.armor_panel_poses[i].orientation.x
            panels[i].transform.rotation.y = msg.secondary_robot.armor_panel_poses[i].orientation.y
            panels[i].transform.rotation.z = msg.secondary_robot.armor_panel_poses[i].orientation.z
            panels[i].transform.rotation.w = msg.secondary_robot.armor_panel_poses[i].orientation.w

            self.tf_broadcaster.sendTransform(panels[i])

        



def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()