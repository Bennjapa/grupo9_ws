#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

class wall_distance(Node):
    def __init__(self):
        super().__init__("wall_distance")
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.image_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10
        )
        self.obstacle_pub = self.create_publisher(
            Vector3,
            "state",
            10
        )
        self.get_logger().info("Nodo detector de obstáculos iniciado")

        self.distancias = Vector3()

    def laser_callback(self, data):

        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        
        # noveno = len(self.ranges) // 9

        zona_derecha = self.ranges[60:65]
        zona_central = self.ranges[65:115]
        zona_izquierda = self.ranges[115:120]

        def minimo(zona):
            zona_np = np.array(zona)
            dist = np.nanmin(zona_np)
            return dist if not np.isnan(dist) else 0.0
        
        izquierda = minimo(zona_izquierda)
        centro = minimo(zona_central)
        derecha = minimo(zona_derecha)

        
        
        self.distancias.y = float(centro)
        self.distancias.x = float(izquierda)
        self.distancias.z = float(derecha)

        self.obstacle_pub.publish(self.distancias)
        # self.get_logger().info(f"distancia_muro={self.distancia_muro.data}")


if __name__ == '__main__':
    rclpy.init()
    kinect_node = wall_distance()
    rclpy.spin(kinect_node)