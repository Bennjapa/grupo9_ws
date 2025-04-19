#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Timer
from nav_msgs.msg import Odometry

class dead_reckoning_nav(Node):
    def __init__(self):
        super().__init__("dead_reckoning_nav")
        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular
        self.t = 0.0
        self.hay_obstaculo = False
        
        #Creamos nuestro publisher de velocidad
        self.enviar_velocidad = self.create_publisher(
            Twist,
            "/cmd_vel_mux/input/navigation",
            10
        )

        self.recibir_poses = self.create_subscription(
            PoseArray,
            "goal_list",
            self.accion_mover_cb,
            10
        )
        self.recibir_obstaculos = self.create_subscription(
            Vector3,
            "/occupancy_state",
            self.accion_obstaculo_cb,
            10
        )

        self.pose_sub = self.create_subscription(Pose, "/real_pose", self.registro_realpose, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.registro_odometry, 10)

        #Creamos nuestro mensaje de velocidad
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        with open(f"odom_poses.txt", "w") as file:
            file.write("")
        with open(f"real_poses.txt", "w") as file:
            file.write("")

        #Creamos un timer que estara enviando velocidades cada 0.1 segundos
        self.timer_mover = self.create_timer(0.1, self.move)

    def aplicar_velocidad(self, speed_command_list):
        #Speed_command_list = (v,w,t) (velocidad, velocidad angular, tiempo)
        for i in  speed_command_list:
            timer = Timer(self.t, self.asignar_velocidad, args=(i[0], i[1]))
            timer.start()
            self.t += i[2]
        timer = Timer(self.t, self.asignar_velocidad, args=(0.0, 0.0))
        timer.start()
    
    def mover_robot_a_destino(self, goal_pose):
        #goal_pose = (x,y,theta) es la posicion a la que se quiere llegar 
        x = float(goal_pose[0])
        y = float(goal_pose[1])
        theta = float(goal_pose[2])
        t1 = (x/self.max_v) 
        t2 = (theta/self.max_w)
        t3 = (y/self.max_v) 
        #1.105 esta bueno
        factor_correct = 1.105
        t2_c = t2 * factor_correct
        
        if x < 0:
            x_v = -self.max_v
        else: 
            x_v = self.max_v
        if y < 0:
            y_v = -self.max_v
        else: 
            y_v = self.max_v
        if theta < 0:
            w = -self.max_w
        else:
            w = self.max_w
        self.get_logger().info(f"t1: {t1}, t2: {t2}, t3: {t3}")
        lista_de_comandos = [
            (x_v, 0.0, abs(t1)),
            (0.0, w, abs(t2_c)),
            (y_v, 0.0, abs(t3))
        ]
        self.aplicar_velocidad(lista_de_comandos)

    def accion_mover_cb(self, goal_list: PoseArray):
        for i in goal_list.poses:
            x = i.position.x
            y = i.position.y
            pose_quaternion = (i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w)
            roll, pitch, yaw = euler_from_quaternion(pose_quaternion)
            theta = yaw
            self.mover_robot_a_destino((x,y,theta))

    def move(self):
        self.enviar_velocidad.publish(self.speed)

    def asignar_velocidad(self, v, w):
        if not self.hay_obstaculo:
            self.get_logger().info(f"Asignando velocidad: v={v}, w={w}")
            self.speed.linear.x = v
            self.speed.angular.z = w

    def accion_obstaculo_cb(self, vector: Vector3):
        if vector.x == 1.0 or vector.y == 1.0 or vector.z == 1.0:
            self.get_logger().warn("Obstáculo detectado")
            self.hay_obstaculo = True
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
        else:
            self.hay_obstaculo = False
    
    def registro_realpose(self, real_pose : Pose):
        with open(f"real_poses.txt", "a") as file:
            file.write(f"{real_pose.position.x},{real_pose.position.y}\n")

    def registro_odometry(self, odom : Odometry):
        with open(f"odom_poses.txt", "a") as file:
            file.write(f"{odom.pose.pose.position.x},{odom.pose.pose.position.y}\n")


if __name__ == "__main__":
    rclpy.init()
    nodo = dead_reckoning_nav()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
