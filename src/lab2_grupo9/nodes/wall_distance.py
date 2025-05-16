#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
# from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class wall_distance(Node):
    def __init__(self):
        super().__init__("wall_distance")
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.image_sub = self.create_subscription(
            Image,
            "camera/depth/image_raw",
            self.image_callback,
            10
        )
        self.obstacle_pub = self.create_publisher(
            Float64,
            "state",
            10
        )
        self.get_logger().info("Nodo detector de obstáculos iniciado")

        self.diferencia = Float64()

    def image_callback(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        altura, ancho = self.current_cv_depth_image.shape

        tercio = ancho // 3

        zona_izquierda = self.current_cv_depth_image[:, :tercio]
        zona_central = self.current_cv_depth_image[:, tercio:2*tercio]
        zona_derecha = self.current_cv_depth_image[:, 2*tercio:]

        def minimo(zona):
            dist = np.nanmin(zona)
            return dist if not np.isnan(dist) else 0.0
        
        izquierda = minimo(zona_izquierda)
        centro = minimo(zona_central)
        derecha = minimo(zona_derecha)

        self.diferencia.data = float(izquierda - derecha)

        self.obstacle_pub.publish(self.diferencia)
        # self.get_logger().info(f"diferencia={self.diferencia.data}")

if __name__ == '__main__':
    rclpy.init()
    kinect_node = wall_distance()
    rclpy.spin(kinect_node)
