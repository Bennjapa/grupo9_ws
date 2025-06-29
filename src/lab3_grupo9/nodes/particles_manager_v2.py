#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
from particle import Particle
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import cv2
from scipy.stats import norm
from std_msgs.msg import String
from math import sin, cos

import numpy as np
from random import uniform

class ParticlesManager(Node):
  
  def __init__( self, num_particles ):
    super().__init__('particles_manager')
    self.num_particles = num_particles
    self.sigma = 0.01
    self.particles = []
    self.pub_particles = self.create_publisher(PoseArray, 'particles', 10)
    self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
    self.map_sub = self.create_subscription(OccupancyGrid, "/world_map", self.recibir_mapa, 10)
    self.sens_x = 0.0 #[m]
    self.sens_y = 0.0 #[m]
    
    # For testing only:
    # self.create_timer( 1.0, self.rotate_particles )

  def create_particles( self, range_x, range_y ):
    for i in range( 0, self.num_particles ):
      x = uniform( range_x[0], range_x[1] )
      y = uniform( range_y[0], range_y[1] )
      ang = uniform( -np.pi, np.pi )
      new_particle = Particle( x, y, ang, sigma = self.sigma )
      self.particles.append( new_particle )
    self.publish_particles()

  def update_particles( self, delta_x, delta_y, delta_ang ):
    for particle in self.particles:
      particle.move( delta_x, delta_y, delta_ang )
    self.publish_particles()

  def publish_particles(self):
    pose_array_msg = PoseArray()
    pose_array_msg.header = Header()
    pose_array_msg.header.frame_id = "base_link"

    for part in self.particles:
      part_pose = Pose()
      part_pose.position.x, part_pose.position.y = part.x, part.y
      quat = quaternion_from_euler(0,0, part.ang)

      part_pose.orientation.x = quat[0]
      part_pose.orientation.y = quat[1]
      part_pose.orientation.z = quat[2]
      part_pose.orientation.w = quat[3]

      pose_array_msg.poses.append(part_pose)

    self.pub_particles.publish(pose_array_msg)

  # For testing only:
#   def rotate_particles( self ):
#     self.update_particles( 0, 0, (30*np.pi/180) )
  def odom_callback(self, msg: Odometry):
        
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, theta  = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))

    x = pos.x
    y = pos.y

    if self.ultima_odom is not None:
        x_prev, y_prev, theta_prev = self.ultima_odom

        dx = x - x_prev
        dy = y - y_prev
        dtheta = theta - theta_prev

        self.update_particles(dx, dy, dtheta)

    self.ultima_odom = (x, y, theta)

  def recibir_mapa(self, data : OccupancyGrid):
    self.ancho = data.info.width
    self.alto = data.info.height
    self.res = data.info.resolution
    self.origin = [data.info.origin.position.x, data.info.origin.position.y, 0.0]
    self.likelihood_field("mapa.pgm")

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
        n = 10 #Definimos cuantos pixeles recorremos en cada dirección en este caso es n*2 x n*2

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
    self.field = field/field.max() #Normalizamos el campo para que el máximo de probabilidad sea 1
    self.field_ready = True #Indicamos que el campo esta listo
    self.get_logger().info("Estamos listos con el mapa :D")
    # field_norm = (self.field * 255).astype(np.uint8)
    #Mostramos el mapa
    # cv2.imshow('Field', field_norm)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
  
  def likelihood_field_range_finder_model(self, mediciones, pose, mapa):
    theta = 0.0
    q = 1 #Inicio de la productoria
    angulo = self.angle_min #Angulo mínimo que mide el sensor
    # print(angulo)      
    rayos_validos = 0
    FOV = 57 * np.pi / 180
    for i in mediciones:
        if i != self.zmax and not np.isnan(i) and angulo < FOV/2 and angulo > -FOV/2:
            rayos_validos += 1

            x_real = pose[1]*self.res #Transformamos x a [m]
            y_real = (self.alto - pose[0] - 1)*self.res #Transformamos y a [m] y lo ponemos en coordenadas reales
            #Aplicamos formulas de clases slide 38
            x = x_real + self.sens_x*cos(theta) - self.sens_y*sin(theta) + i*cos(theta + angulo)
            y = y_real + self.sens_y*cos(theta) + self.sens_x*sin(theta) + i*sin(theta + angulo)

            pixel_x = int(x / self.res) #Pasamos la posicion x del final del laser a pixeles
            pixel_y = self.alto - int(y / self.res) - 1 #Pasamos la posicion y del final del laser a pixeles

            if pixel_x < 0:
                pixel_x = 0
            elif pixel_x > self.ancho - 1:
                pixel_x = self.ancho - 1
            if pixel_y < 0:
                pixel_y = 0
            elif pixel_y > self.alto - 1:
                pixel_y = self.alto - 1

            prob = mapa[pixel_y, pixel_x]
            if prob > 0:
                q = q*(self.zhit * prob)
            else:
                q *= 1e-6
        
        angulo += self.increment #Sumamos el incremento para el siguiente ángulo
    # print(angulo)
    if rayos_validos == 0 or np.isnan(q):
        q = 1e-6
        
    return q


def main():
  rclpy.init()

  map_width_pix = 270 # [pix]
  map_height_pix = 270 # [pix]
  map_resolution = 0.01 # [m/pix]

  map_width_m = map_width_pix * map_resolution
  map_height_m = map_height_pix * map_resolution

  particle_manager = ParticlesManager( num_particles = 100 )
  particle_manager.create_particles( [0, map_width_m], [0, map_height_m] )

  rclpy.spin(particle_manager)
  particle_manager.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()



