from geometry_msgs.msg import TransformStamped

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException, LookupException, ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tr_messages.msg import DetWithImg
from tr_messages.msg import SimGroundTruth
from std_msgs.msg import Float64


class ErrorPublisher(Node):

    def __init__(self):
        super().__init__('calc_error')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #creates the publishers we will use
        self.x_err_publisher = self.create_publisher(Float64, 'x_err', 10)
        self.y_err_publisher = self.create_publisher(Float64, 'y_err', 10)
        self.z_err_publisher = self.create_publisher(Float64, 'z_err', 10)
        #prevents unused error
        self.x_err_publisher
        self.y_err_publisher
        self.z_err_publisher

        
        #depends on det for publishing cue
        self.det_subscription = self.create_subscription(
            DetWithImg,
            '/detections',
            self.calc_error,
            1
        )
        self.det_subscription

    def calc_error(self, msg):
        when = self.get_clock().now() - Duration(seconds=1)

        try: 
            det_t = self.tf_buffer.lookup_transform(
                "map",
                "detected_panel",
                when,
                #Duration(seconds=0.5)
            )
            p1_t = self.tf_buffer.lookup_transform(
                "map",
                "panel_1",
                when,
                #Duration(seconds=0.5)
            )
            p2_t = self.tf_buffer.lookup_transform(
                "map",
                "panel_2",
                when,
                #Duration(seconds=0.5)
            )
            p3_t = self.tf_buffer.lookup_transform(
                "map",
                "panel_3",
                when,
                #Duration(seconds=0.5)
            )
            p4_t = self.tf_buffer.lookup_transform(
                "map",
                "panel_3",
                when,
                #Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info(
                f'transform not ready'
            )
            return
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform properly: {ex}'
            )
            return
        
        panels = [p1_t, p2_t, p3_t, p4_t]
        min_error_panel = None
        min_error = None
        for panel in panels:
            error = math.sqrt(
                (det_t.transform.translation.x-panel.transform.translation.x)**2 +
                (det_t.transform.translation.y-panel.transform.translation.y)**2 +
                (det_t.transform.translation.z-panel.transform.translation.z)**2
                )
            if min_error is None:
                min_error = error
                min_error_panel = panel
            elif (error < min_error):
                min_error_panel = panel
                min_error = error
        self.get_logger().info(
            f'The current panel is {min_error_panel.child_frame_id}'
        )
        x_err, y_err, z_err = Float64(),Float64(),Float64()
        x_err.data = min_error_panel.transform.translation.x-det_t.transform.translation.x
        y_err.data = min_error_panel.transform.translation.y-det_t.transform.translation.y
        z_err.data = min_error_panel.transform.translation.z-det_t.transform.translation.z

        self.x_err_publisher.publish(x_err)
        self.y_err_publisher.publish(y_err)
        self.z_err_publisher.publish(z_err)


def main():
    rclpy.init()
    node = ErrorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()