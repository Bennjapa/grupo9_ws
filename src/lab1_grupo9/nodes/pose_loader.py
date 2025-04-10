#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
import math
import sys

class pose_loader(Node):
    
    def __init__(self):
        super().__init__("pose_loader")

        self.enviar_poses = self.create_publisher(
            PoseArray,
            "goal_list",
            10
        )
        self.leer_poses()
    
    def leer_poses(self):
        with open("posiciones.txt", "r") as file:
            posiciones = file.readlines()
            posiciones_procesadas = []

            for i in posiciones:
                a = i.strip("\n")
                posicion = a.split(",")
                posiciones_procesadas.append(posicion)
            
            #posiciones procesadas va a quedar con las posiciones en orden
            #(x, y, theta)
        posiciones_Pose = PoseArray()
        for i in posiciones_procesadas:
            pose = Pose()
            pose.position.x = float(i[0])
            pose.position.y = float(i[1])
            pose.position.z = 0.0
            x, y, z, w = quaternion_from_euler(0.0, 0.0, math.radians(float(i[2])))
            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w
            posiciones_Pose.poses.append(pose)

        self.enviar_poses.publish(posiciones_Pose)

if __name__ == "__main__":
    rclpy.init()
    ldr = pose_loader()
    rclpy.spin(ldr)
    ldr.destroy_node()
    rclpy.shutdown()
    