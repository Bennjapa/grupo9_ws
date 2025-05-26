#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path


class TurtleBot(Node):
    def __init__(self):
        Node.__init__(self, "follower_node")
        # Posición robotin
        self.x = 0
        self.y = 0

        # Variables para la velocidad
        self.velocidad = Twist()
        self.velocidad.linear.x = 0.1

        # Leemos la velocidad angular
        self.w_reader = self.create_subscription(
            Twist,
            "control_effort",
            self.recibir_w,
            1
        )

        # Publicar la velocidad
        self.pub_velocidad = self.create_publisher(
            Twist,
            "/cmd_vel_mux/input/navigation",
            10
        )

        self.timer_velocidad = self.create_timer(0.1, self.mover)

        # Nos suscribimos a nav_plan para graficar el path ideal
        self.recibir_path = self.create_subscription(
            Path,
            "nav_plan",
            self.path_cb,
            10
        )
        # Creamos una variable para graficar solo una vez el path
        self.path_leido = False

        # Nos suscribimos a las poses reales del bot para graficarlas
        self.pose_sub = self.create_subscription(
            Pose,
            "/real_pose",
            self.registro_realpose,
            10
        )

    def recibir_w(self, w : Twist):
        self.velocidad.angular.z = w.angular.z

    def mover(self):
        self.pub_velocidad.publish(self.velocidad)

    def registro_realpose(self, pose : Pose):
        self.x = pose.position.x
        self.y = pose.position.y
    
    def path_cb(self, path : Path):
        if not self.path_leido:
            # path: .poses, .poses[0].pose, poses[0].pose.position.x
            path_recibido = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]

            if len(path_recibido) > 20: # Confirmamos que se leyó algo y no un mensaje vacío
                self.path_leido = True # Decimos que se leyó para evitar sobreescribir path_por_recorrer


if __name__ == '__main__':
    
    rclpy.init()
    robotin = TurtleBot()
    rclpy.spin(robotin)
    robotin.destroy_node()
    rclpy.shutdown()
