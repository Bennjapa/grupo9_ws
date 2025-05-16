#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Timer
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class p_navigation_node(Node):
    def __init__(self):
        super().__init__("p_navigation_node")
        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular
        self.t = 0.0
        
        #Creamos nuestro publisher de velocidad
        self.enviar_velocidad = self.create_publisher(
            Twist,
            "/cmd_vel_mux/input/navigation",
            10
        )

        self.recibir_accion_control = self.create_subscription(
            Float64,
            "/control_action",
            self.accion_control_cb,
            10
        )

        #Creamos nuestro mensaje de velocidad
        self.speed = Twist()
        self.speed.linear.x = self.max_v
        self.speed.angular.z = 0.0

        #Creamos un timer que estara enviando velocidades cada 0.1 segundos
        self.timer_mover = self.create_timer(0.1, self.move)


    def move(self):
        self.enviar_velocidad.publish(self.speed)

    def accion_control_cb(self, accion: Float64):
        if accion.data > self.max_w:
            accion.data = self.max_w
        else:
            self.speed.angular.z = accion.data
        # self.get_logger().info(f"accion_control_cb: {accion.data}")


if __name__ == "__main__":
    rclpy.init()
    nodo = p_navigation_node()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
