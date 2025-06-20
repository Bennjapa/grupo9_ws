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
from math import sin, cos

class sensorModel(Node):

    def __init__(self):
        super().__init__("sensor_model_node")

        self.sigma = 0.02 #desviación estándar de la distribución normal, con 0.02 funciona decente
        self.zmax = 4.0 #[m], las lecturas validas estan en un rango de 57°
        self.zhit = 0.2
        self.field_ready = False #Hacemos esto para que no se active el range finder, antes del likelihood field
        self.posiciones_desocupadas = []

        #Definimos la posición del sensor, respecto al robot
        self.sens_x = 0.0 #[m]
        self.sens_y = 0.0 #[m]

        ##Recibimos la información de los nodos
        ##leer linea 57 
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

    ############################################################
    #no me funciona el mapa, el tópico map como que no envia nada
    #############################################################
    # def recibir_mapa(self, data : OccupancyGrid):
    #     self.get_logger().info("HOLAAAAAAAAAAAAAa")

    def likelihood_field(self, mapa:String):
        img = cv2.imread(mapa, cv2.IMREAD_GRAYSCALE) #Leemos la imagen
        alto, ancho = img.shape #Obtenemos las dimensiones de la imagen
        posiciones_ocupadas = [] #Creamos una lista vacia para ingresar posiciones ocupadas
        for y in range(alto): #Recorremos la imagen, para ver las posiciones con obstaculos
            for x in range(ancho):
                if img[y,x] == 0: #Ingresamos posiciones con obstaculos
                    posiciones_ocupadas.append((y,x))
                elif img[y,x] == 255: #Ingresamos posiciones desocupadas
                    self.posiciones_desocupadas.append((y,x))
        #Convertimos en arrays de numpy ambas listas
        posiciones_ocupadas = np.array(posiciones_ocupadas)
        self.posiciones_desocupadas = np.array(self.posiciones_desocupadas)
        self.get_logger().info("Terminamos la primera parte :D") 

        #Hacemos un array vacio, para hacer nuestro campo
        field = np.zeros((alto, ancho))

        #Esto es solo para visualizar el progreso del programa
        largo = len(posiciones_ocupadas)
        contador = 1

        for i in posiciones_ocupadas: #Recorremos las posiciones ocupadas
            distancias = np.zeros((alto, ancho)) #Hacemos un arreglo vacio para las distancias, desde cada pixel
            n = 30 #Definimos cuantos pixeles recorremos en cada dirección en este caso es n*2 x n*2

            for y in range(i[0] - n, i[0]+ n):
                for x in range(i[1] - n, i[1]+ n):
                    if y < alto and y >= 0 and x < ancho and x >= 0:
                        dist = np.sqrt(((y-i[0])*self.res)**2 + ((x-i[1])*self.res)**2) #Obtenemos la distancia al obstaculo desde cada pixel
                        distancias[y,x] = dist

            normal_distribution = norm.pdf(distancias, loc = 0, scale=self.sigma) #Calculamos la probabilidad de todas las distancias cercanas al obstaculo

            for y in range(i[0] - n, i[0]+ n): #Recorremos de nuevo para sumar al campo las probabilidades
                for x in range(i[1] - n, i[1]+ n):
                    if y < alto and y >= 0 and x < ancho and x >= 0:
                        field[y,x] += normal_distribution[y,x]

            self.get_logger().info(f"Completadas {contador} de {largo}")
            contador += 1
            
        #ahora visualizamos el mapa de densidades y ver si es que funciona :D
        self.alto = alto
        self.ancho = ancho
        self.field = field/field.max() #Normalizamos el campo para que el máximo de probabilidad sea 1
        self.field_ready = True #Indicamos que el campo esta listo
        self.get_logger().info("Estamos listos con el mapa :D")
        field_norm = (self.field*255).astype(np.uint8)
        #Mostramos el mapa
        cv2.imshow('Field', field_norm)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def recibir_laser(self, data : LaserScan):
        mediciones = data.ranges #Listado de mediciones
        self.angle_min = data.angle_min #ángulo minimio en el que mide el sensor
        self.increment = data.angle_increment #cada cuanto incrementa el angulo por medición

        valores_verosimilitud = np.zeros((self.alto, self.ancho)) #array vacio, con donde puede estar el robot

        if self.field_ready: #Si esta listo el campo de distancias
            for i in self.posiciones_desocupadas: #Probamos con todas las posiciones x,y desocupadas
                q = self.likelihood_field_range_finder_model(mediciones, i, self.field) #obtenemos la probabilidad de la posición
                valores_verosimilitud[i[0], i[1]] = q #Aqui guardamos en el mapa de verosimilitud la probabilidad de la posición (x,y)
            a = (valores_verosimilitud * 255).astype(np.uint8) #Aqui transformamos a escala de grises
            cv2.imshow('Pose?', a)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    
    def likelihood_field_range_finder_model(self, mediciones, pose, mapa):
        theta = 0
        q = 1 #Inicio de la productoria
        angulo = self.angle_min #Angulo mínimo que mide el sensor
        for i in mediciones:
            if i != self.zmax:
                x_real = pose[1]*self.res #Transformamos x a [m]
                y_real = pose[0]*self.res #Transformamos y a [m]
                #Aplicamos formulas de clases slide 38
                x = x_real + self.sens_x*cos(theta) - self.sens_y*sin(theta) + i*cos(theta + angulo)
                y = y_real + self.sens_y*cos(theta) + self.sens_x*sin(theta) + i*sin(theta + angulo)
                if int(x/self.res) >= 0 and int(x/self.res) < 270 and int(y/self.res) >= 0 and int(y/self.res) < 270:
                    prob = mapa[int(y/self.res), int(x/self.res)]
                    q = q*(self.zhit * prob)
            angulo += self.increment #Sumamos el incremento para el siguiente ángulo
        return q


if __name__ == "__main__":
    rclpy.init()
    nodo = sensorModel()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()