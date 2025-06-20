#!/usr/bin/env python3

import rclpy
import time
from scipy.stats import norm
import numpy as np
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, Vector3
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Thread, Event
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from std_msgs.msg import String
from math import atan2, sin, cos

class sensorModel(Node):

    def __init__(self):
        super().__init__("sensor_model_node")

        self.sigma = 0.02 #desviación estándar de la distribución normal
        self.zmax = 4.0 #[m], las lecturas validas estan en un rango de 57°

        ##Recibimos la información de los nodos
        # self.map_sub = self.create_subscription(
        #     OccupancyGrid,
        #     "map",
        #     self.recibir_mapa,
        #     10
        # )

        self.laser = self.create_subscription(
            LaserScan,
            "/scan",
            self.recibir_laser,
            1
        )
        self.origin = [0.0, 0.0, 0.0]
        self.res = 0.006
        self.occupied_tresh = 0.65
        self.free_tresh = 0.196
        self.likelihood_field("mapa.pgm")


    # def recibir_mapa(self, data : OccupancyGrid):
    #     self.get_logger().info("HOLAAAAAAAAAAAAAa")

    def likelihood_field(self, mapa:String):
        img = cv2.imread(mapa, cv2.IMREAD_GRAYSCALE)
        alto, ancho = img.shape
        posiciones_ocupadas = []
        for y in range(alto):
            for x in range(ancho):
                if img[y,x] == 0:
                    posiciones_ocupadas.append((y,x))
        posiciones_ocupadas = np.array(posiciones_ocupadas)
        self.get_logger().info("Terminamos la primera parte :D")

        field = np.zeros((alto, ancho))
        largo = len(posiciones_ocupadas)
        contador = 0

        for i in posiciones_ocupadas:
            distancias = np.zeros((alto, ancho))
            n = 10
            for y in range(i[0] - n, i[0]+ n):
                for x in range(i[1] - n, i[1]+ n):
                    if y < alto and y >= 0 and x < ancho and x >= 0:
                        dist = np.sqrt(((y-i[0])*self.res)**2 + ((x-i[1])*self.res)**2)
                        distancias[y,x] = dist
            normal_distribution = norm.pdf(distancias, loc = 0, scale=self.sigma)
            for y in range(i[0] - n, i[0]+ n):
                for x in range(i[1] - n, i[1]+ n):
                    if y < alto and y >= 0 and x < ancho and x >= 0:
                        field[y,x] += normal_distribution[y,x]
            self.get_logger().info(f"Completadas {contador} de {largo}")
            contador += 1
            
        #ahora visualizamos el mapa de densidades y ver si es que funciona :D
        field = field/field.max() 
        self.get_logger().info("Estamos listos :D")
        self.get_logger().info(str(field))
        field_norm = (field*255).astype(np.uint8)
        cv2.imshow('Field', field_norm)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def recibir_laser(self, data : LaserScan):
        #NOTA: cada indice de la lista de mediciones es un angulo
        #posee 181 datos
        #cada indice es un angulo en grados
        #Medición en angulo 0° asumo que es el indice 90

        mediciones = data.ranges
        
        #self.get_logger().info(str(mediciones[90]))

if __name__ == "__main__":
    rclpy.init()
    nodo = sensorModel()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()