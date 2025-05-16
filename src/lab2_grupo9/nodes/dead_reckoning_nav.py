#!/usr/bin/env python3

import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Thread, Event
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Empty

class dead_reckoning_nav(Node):
    def __init__(self):
        super().__init__("dead_reckoning_nav")
        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular
        self.hay_obstaculo = False
        #Estado de la coordenada x
        self.x_state = 0.0
        #Estado de la coordenada y
        self.y_state = 0.0
        #Estado de el Angulo
        self.a_state = 0.0
        
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

        #Publishers y subscribers de controlador lineal
        self.state_pub = self.create_publisher(Float64, "state", 1)
        self.setpoint_pub = self.create_publisher(Float64, "setpoint", 1)
        self.control_effort_sub = self.create_subscription(Float64, "control_effort", self.asignar_velocidad_lineal, 1)

        #Publishers y subscribers de controlador angular
        self.state_w_pub = self.create_publisher(Float64, "state_w", 1)
        self.setpoint_w_pub = self.create_publisher(Float64, "setpoint_w", 1)
        self.control_effort_w_sub = self.create_subscription(Float64, "control_effort_w", self.asignar_velocidad_angular, 1)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.leer_odometria, 10)

        #Creamos un publisher de setpoint, publisher del estado
        """""
        self.pose_sub = self.create_subscription(Pose, "/real_pose", self.registro_realpose, 10)
        self.odom_reg_sub = self.create_subscription(Odometry, "/odom", self.registro_odometry, 10)"""


        #Creamos nuestro mensaje de velocidad
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        """
        with open(f"odom_poses.txt", "w") as file:
            file.write("")
        with open(f"real_poses.txt", "w") as file:
            file.write("")"""

        #Creamos un timer que estara enviando velocidades cada 0.1 segundos
        self.timer_mover = self.create_timer(0.1, self.move)

    def aplicar_velocidad(self, displacement_list):
        #Speed_command_list = (v,w,t) (velocidad, velocidad angular, tiempo)

        #Se hacen threads para que cada ejecución de comandos espere a que termine la anterior
        self.primer_controlador = Thread(target= self.controlar_x, args= (displacement_list[0],), daemon=True)
        self.segundo_controlador = Thread(target= self.controlar_w, args= (displacement_list[1],), daemon=True)
        self.tercer_controlador = Thread(target=self.controlar_y, args= (displacement_list[2],), daemon=True)
        self.primer_controlador.start()
        self.primer_controlador.join()
        self.segundo_controlador.start()
        self.segundo_controlador.join()
        self.tercer_controlador.start()
        self.tercer_controlador.join()

    
    def mover_robot_a_destino(self, goal_pose):
        #goal_pose = (x,y,theta) es la posicion a la que se quiere llegar 
        x = float(goal_pose[0])
        y = float(goal_pose[1])
        theta = float(goal_pose[2])

        displacement_list = [
            (x, 0.0),
            (0.0, theta),
            (y, 0.0)
        ]
        self.get_logger().info(f"Enviando Displacement {x},{theta},{y}")
        self.aplicar_velocidad(displacement_list)

    def accion_mover_cb(self, goal_list: PoseArray):
        #Aqui se hace un thread para poder recorrer la lista de posiciones sin detener el hilo principal
        self.accion_mover_cb_thread = Thread(target= self.enlistar_poses, args=(goal_list,))
        self.accion_mover_cb_thread.start()

    def enlistar_poses(self, goal_list: PoseArray):
        for i in goal_list.poses:
            x = i.position.x
            y = i.position.y
            pose_quaternion = (i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w)
            roll, pitch, yaw = euler_from_quaternion(pose_quaternion)
            theta = yaw
            self.thread_mover_robot_a_destino = Thread(target=self.mover_robot_a_destino, args=((x,y,theta),), daemon=True)
            self.thread_mover_robot_a_destino.start()
            self.thread_mover_robot_a_destino.join()

    def move(self):
        self.enviar_velocidad.publish(self.speed)

    def asignar_velocidad_lineal(self, data:Float64):
        if not self.hay_obstaculo:
            #self.get_logger().info(f"Asignando velocidad: v={data.data}")
            self.speed.linear.x = data.data

    def asignar_velocidad_angular(self, data:Float64):
        if not self.hay_obstaculo:
            #self.get_logger().info(f"Asignando velocidad: w={data.data}")
            self.speed.angular.z = data.data

    def accion_obstaculo_cb(self, vector: Vector3):
        if vector.x == 1.0 or vector.y == 1.0 or vector.z == 1.0:
            self.get_logger().warn("Obstáculo detectado")
            self.hay_obstaculo = True
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
        else:
            self.hay_obstaculo = False

    def leer_odometria(self, odom: Odometry):
        pose_quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(pose_quaternion)
        self.a_state = yaw

        #Filtramos los valores de x e y, porque dependiendo del angulo que tenga nuestro robot estamos invirtiendos nuestros ejes de coordenadas
        if self.a_state <= np.pi/2 and self.a_state >= -np.pi/2:
            self.x_state = odom.pose.pose.position.x
        else:
            self.x_state = -odom.pose.pose.position.x

        if self.a_state >= 0:
            self.y_state = odom.pose.pose.position.y
        else:
            self.y_state = -odom.pose.pose.position.y
    

    #Estas 3 son las funciones para controlar los movimiento en x,y,theta
    def controlar_x(self, pose:tuple):
        x_ref = pose[0] + self.x_state
        
        setpoint = Float64()
        setpoint.data = x_ref
        self.setpoint_pub.publish( setpoint )
        while round(self.x_state, 4) != round(x_ref, 4):
            msg = Float64()
            msg.data = self.x_state
            self.state_pub.publish( msg )
            self.get_logger().info(f"{self.x_state} != {x_ref}")

    def controlar_y(self, pose:tuple):
        y_ref = pose[0] + self.y_state

        setpoint = Float64()
        setpoint.data = y_ref
        self.setpoint_pub.publish( setpoint )
        while round(self.y_state, 4) != round(y_ref, 4):
            msg = Float64()
            msg.data = self.y_state
            self.state_pub.publish( msg )
            self.get_logger().info(f"{self.y_state} != {y_ref}")

    def controlar_w(self, pose:tuple):
        w_ref = pose[1] + self.a_state

        setpoint = Float64()
        setpoint.data = w_ref
        self.setpoint_w_pub.publish( setpoint )
        #Para esta transformamos el angulo porque la odometria mide de [-180, 180], necesitamos hacer la conversión para que el cambio de angulo sea sutil
        #Es decir que nuestro controlador entienda que 180 y -180 son valores cercanos y no lejanos.
        #Esta trasformacion tambien se hace dentro del controlador pid de la velocidad angular para evitar problemas de igual forma
        while round((self.a_state % (2*np.pi)), 4) != round((w_ref % (2*np.pi)), 4):
            msg = Float64()
            msg.data = self.a_state
            self.state_w_pub.publish( msg ) 
            self.get_logger().info(f"{self.a_state} != {w_ref}")
        
    






















    """
    def registro_realpose(self, real_pose : Pose):
        with open(f"real_poses.txt", "a") as file:
            file.write(f"{real_pose.position.x},{real_pose.position.y}\n")

    def registro_odometry(self, odom : Odometry):
        with open(f"odom_poses.txt", "a") as file:
            file.write(f"{odom.pose.pose.position.x},{odom.pose.pose.position.y}\n")"""


if __name__ == "__main__":
    rclpy.init()
    nodo = dead_reckoning_nav()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
