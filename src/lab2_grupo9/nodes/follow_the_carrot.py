#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


class FollowTheCarrot(Node):
    """
    Objetivo: calcular el error entre el ángulo del robot (odometría) y el
    ángulo desde el robot a la zanahoria.
    Recibe: Odometría nav_msgs/Odometry
            Trayectoria de nav_plan nav_msg/Path
    Envía: error entre ángulos
    """
    def __init__(self):
        super().__init__("follow_the_carrot")
        # Creamos las variables principales
        self.path = []

        self.look_ahead_dist = 10 # numero de coordenadas más adelante en la lista del path
        self.path_leido = False # Confirma la lectura del path

        self.x = 0
        self.y = 0
        self.theta = 0

        # Nos suscribimos al a odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odometry_cb,
            10
        )

        # Nos suscribimos al tópico de la ruta
        self.recibir_path = self.create_subscription(
            Path,
            "nav_plan",
            self.path_cb,
            10
        )

        # Creamos un archivo vacío para el registro de la odometría
        with open("odometria.txt", "w") as file:
            file.write("X,Y\n")

        # Creamos el tópico para publicar
        self.publisher = self.create_publisher(
            Float64,
            "follow_the_carrot_topic",
            10
        )
        self.timer_registro_odometria = self.create_timer(0.3, self.registrar_odo)
        self.timer_publicador = self.create_timer(0.3, self.publicar_angulo)

    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion( ( odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w ) )
        # De aquí nos importan X, Y y YAW
        pos = (x, y, yaw)
        self.x, self.y, self.theta = pos

    def registrar_odo(self):
        with open("odometria.txt", "a") as file:
            file.write(str(self.x) + "," + str(self.y) + "\n")

    def path_cb(self, path):
        if not self.path_leido:
            # path: .poses, .poses[0].pose, poses[0].pose.position.x
            self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
            self.get_logger().info(f"Se leyó un path: (largo {len(self.path)}). Existe {bool(self.path)}")
            if len(self.path) > 20: # Confirmamos que se leyó algo y no un mensaje vacío
                self.path_leido = True # Decimos que se leyó

    def pto_a_dist_min(self):
        self.get_logger().info(f"Robot en: {(self.x, self.y)} y yaw {self.theta}")
        min_dist = 1000
        self.pos_cercano = 0
        for i, par in enumerate(self.path):
            dist = abs(par[0] - self.x) + abs(par[1] - self.y)
            if dist < min_dist:
                min_dist = dist
                self.pos_cercano = i
        self.get_logger().info(f"Pos cercano {self.pos_cercano}, de coordenadas {self.path[self.pos_cercano]} y distancia {min_dist}")

    def calculo_theta_robot_carrot(self):
        """
        Método que toma el punto que se encuentra 8 posiciones más adelante y
        en base a él calcula el error angular del robot.
        """
        if self.path_leido: # Solo calcular el error si se ha leido algun path
            self.pto_a_dist_min()
            try:
                carrot = self.path[self.pos_cercano + self.look_ahead_dist]
            except Exception:
                carrot = self.path[-1]
            c_x, c_y = carrot
            self.get_logger().info(f"Carrot en {carrot}")
            angulo_robot_carrot = Float64()
            angulo_robot_carrot.data = np.arctan2(c_y - self.y, c_x - self.x)
            return angulo_robot_carrot

    def publicar_angulo(self):
        angulo = self.calculo_theta_robot_carrot()
        if angulo is not None:
            self.get_logger().info(f"Angulo publicado: {angulo}")
            self.publisher.publish(angulo)


rclpy.init()
f_t_c = FollowTheCarrot()
rclpy.spin(f_t_c)
f_t_c.destroy_node()
rclpy.shutdown()
