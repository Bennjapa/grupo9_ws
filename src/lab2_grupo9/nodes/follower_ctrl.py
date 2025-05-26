#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class FollowerCtrl(Node):
    
    def __init__(self):
        super().__init__("follower_node")
        # Variables para limitar la salida del controlador
        self.w_max = 1.0

        self.nuevo_angulo_recibido = False
        # Variables del robot
        self.x = 0
        self.y = 0
        self.yaw = 0

        # Variables del controlador
        self.kp = 0.6
        self.ki = 0.0
        self.setpoint = None
        self.state = None
        self.error_integral = 0

        self.tiempo_anterior = 0

        # Leemos los ángulos publicados por follow_the_carrot
        self.angulos_sub = self.create_subscription(
            Float64,
            "follow_the_carrot_topic",
            self.recibir_setpoint,
            1
        )

        # Leemos la odometría
        self.odom_read = self.create_subscription(
            Odometry,
            "/odom",
            self.recibir_estado,
            10
        )

        # Publicamos la velocidad angular conseguida
        self.velocidad_pub = self.create_publisher(
            Twist,
            "control_effort",
            1
        )

    def enviar_w(self, w : Twist):
        self.velocidad_pub.publish(w)
        self.nuevo_angulo_recibido = False

    def recibir_setpoint(self, angulo_carrot : Float64):
        """
        Método que recibe el ángulo entre el robot y la carrot.
        """
        self.get_logger().info(f"Recibido theta_robot_carrot {angulo_carrot.data}")
        self.get_clock()
        self.tiempo_anterior = (self.get_clock().now().nanoseconds) * 1e-9
        self.reset()
        self.setpoint = angulo_carrot.data
        self.error_integral = 0
        self.nuevo_angulo_recibido = True

    def recibir_estado(self, odometria : Odometry):
        if self.nuevo_angulo_recibido:
            tiempo_actual = (self.get_clock().now().nanoseconds) * 1e-9

            if self.setpoint == None:
                return

            self.x = odometria.pose.pose.position.x
            self.y = odometria.pose.pose.position.y
            _, _, self.yaw = euler_from_quaternion( ( odometria.pose.pose.orientation.x,
                                                    odometria.pose.pose.orientation.y,
                                                    odometria.pose.pose.orientation.z,
                                                    odometria.pose.pose.orientation.w ) )
            self.state = self.yaw

            dt = tiempo_actual - self.tiempo_anterior
            if dt == 0:
                return

            # Definimos nuestro error como:
            error = self.calculo_error()

            self.error_integral += error * dt
            self.tiempo_anterior = tiempo_actual

            p_actuation = self.kp * error
            i_actuation = self.ki * self.error_integral

            actuation = p_actuation + i_actuation
            self.get_logger().info(f"Calculado un error de {error} y una actuacion de {actuation}")

            if actuation > self.w_max:
                actuation = self.w_max
            elif actuation < -self.w_max:
                actuation = -self.w_max

            w = Twist()
            w.angular.z = actuation
            self.get_logger().info(f"Aplicando w de {actuation}")
            self.enviar_w(w)
    
    def calculo_error(self):
        """
        Método que usa el angulo entre el robot y la carrot (theta_rc) y el
        ángulo del robot según odometría (theta_r).
        """
        diferencia = self.state - self.setpoint
        # Normalizamos el resultado para que esté entre -pi y pi
        error = -np.arctan2(np.sin(diferencia), np.cos(diferencia))
        return error

    def reset(self):
        self.setpoint = None
        self.state = None

if __name__ == '__main__':
  
    rclpy.init()
    follower_ctrl = FollowerCtrl()
    rclpy.spin( follower_ctrl )
    follower_ctrl.destroy_node()
    rclpy.shutdown()