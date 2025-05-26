#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
import os


class PathReader(Node):
    """
    Lee los paths de los archivos de texto
    """
    def __init__(self):
        super().__init__("path_reader")

        # Creamos el mensaje que será enviado
        self.path = Path()
        # Creamos el tópico en donde publicaremos el path
        self.nav_plan = self.create_publisher(
            Path,
            "nav_plan",
            10
        )

        ruta_base = get_package_share_directory('lab2_grupo9')
        # ------ ESTA LINEA SE MODIFICA POR CADA PRUEBA --------
        archivo = "path_" + "sqrt" + ".txt"
        # ------------------------------------------------------
        ruta_txt = os.path.join(ruta_base, archivo)
                
        with open(ruta_txt, "r") as file:
            self.get_logger().info("Leyendo el archivo " + archivo)
            posiciones = file.readlines()
            posiciones = [pose.strip().split(",") for pose in posiciones]

        # Posiciones es una lista de pares ordenados [x, y]
        for pos in posiciones:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(pos[0])
            pose.pose.position.y = float(pos[1])
            self.path.poses.append(pose)

        # Publicamos el path periodicamente para evitar la no lectura
        self.timer = self.create_timer(0.2, self.publicar_path)
        
    def publicar_path(self):
        self.nav_plan.publish(self.path)

rclpy.init()
pth_rdr = PathReader()
rclpy.spin(pth_rdr)
pth_rdr.destroy_node()
rclpy.shutdown()
