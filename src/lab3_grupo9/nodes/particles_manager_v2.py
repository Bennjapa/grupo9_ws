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
from std_msgs.msg import String
from funciones_sensor import recibir_mapa, recibir_laser, likelihood_field_range_finder_model
import numpy as np
from random import uniform, choices
from rclpy.qos import QoSProfile, DurabilityPolicy
from decimal import Decimal, getcontext


"""
Buscar forma de guardar mapa
"""

class ParticlesManager(Node):
  
  def __init__( self, num_particles , porcentaje_nuevos , criterio_cercania):
    super().__init__('particles_manager')

    # Código proveniente de la ayudantía y que crea las partículas
    self.num_particles = num_particles
    self.sigma_particulas = 0.01
    self.particles = []

    self.pub_particles = self.create_publisher(PoseArray,
                                               'particles',
                                               10
                                               )
    # ------------- hasta aquí el init de la ayudantía -----------
    self.porcentaje_nuevos = porcentaje_nuevos      # Porcentaje de nuevas partículas por iteración
    self.cercania = criterio_cercania # Metros a los que se consideran dos particulas como de la misma nube
    # Aquí pondremos las variables que necesitaremos para el sensor
    # y que robamos del código sensor_model.py
    self.sigma_sensor = 0.02          # Desviación del sensor
    self.zmax = 4.0                   # Alcance máximo del sensor, en metros
    self.zhit = 1                     # Constante arbitraria
    self.field_ready = False          # Variable para avisar que el likelihood field está ok
    self.sens_x = 0.0                 #[m] 
    self.sens_y = 0.0                 #[m]
    self.res = 0                      # Resolución del mapa
    self.alto = 0                     # Alto del mapa
    self.ancho = 0                    # Ancho del mapa
    self.origin = None                # Origen
    self.field = None                 # Likelihood field
    self.angle_min = 0                # Ángulo mínimo del sensor
    self.increment = 0                # Incremento angular de las mediciones del sensor
    self.mediciones = []              # Mediciones del Lidar
    self.espacios_disponibles = []    # Lista de tuplas que son coordenadas x,y
    self.posiciones_ocupadas = []     # Lista de tuplas con coordenadas con muralla
    self.espacios_no_disponibles = [] # Lista de tuplas de coordenadas con todos los espacios
                                      # que el robot no puede tener
    
    #self.correr_filtro = True       # Esta variable coordina la ejecución del filtro
                                    # y para esto se coordina con el nodo de movimiento
                                    # de modo que solo se ejcuta si es que está quieto
                                    # Si es True se ejecuta el filtro y False se mueve
    self.termino_programa = False   # Una vez terminado el programa se hace True y no se
                                    # se le avisa al robot que se puede mover.
    latching_qos = QoSProfile(depth = 1, durability = DurabilityPolicy.TRANSIENT_LOCAL)
    # Suscripciones para el mapa y sensor, respectivamente
    self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/world_map",
            self.recibir_mapa_cb,
            1
        )

    self.laser = self.create_subscription(
            LaserScan,
            "/scan",
            self.recibir_laser_cb,
            1
        )

    # Suscripción a la odometría, para calcular la variación de movimiento y sus variables
    self.ultima_odom = (0, 0, 0) # Para almacenar la odometría al final de cada cálculo
    self.x_odom = 0
    self.y_odom = 0
    self.theta = 0
    self.odom_sub = self.create_subscription(Odometry,
                                             "/odom",
                                             self.odom_callback,
                                             10
                                             )

    # Finalmente, creamos un tópico en el que estaremos publicando para coordinarnos
    # con el nodo encargado de la rutina de movimiento
    self.pub_estado_carlitos = self.create_publisher(String,
                                               'carlitos',
                                               1
                                               )

    # El anterior dice si está calculando, en caso de que así sea no se puede mover
    # Pero el de ahora leerá si ya terminó de moverse
    # self.sub_movimiento = self.create_subscription(String,
    #                                            'moviendose',
    #                                            self.recibir_estado_movimiento,
    #                                            10
    #                                            )
    self.timer_filtro = self.create_timer(5, self.filtro_de_particulas)
    getcontext().prec = 30

    # For testing only:
    # self.create_timer( 1.0, self.rotate_particles )

  def recibir_estado_movimiento(self, mensaje : String):
    """
    Método principal que está atento a si el robot dejó de moverse para
    aplicar el filtro de particulas.
    Al terminar de ejecutarse el filtro, este avisa publicando en el tópico
    "carlitos" que terminó y ahí inicia el movimiento del robot.
    """
    if mensaje.data == "moviendose":
      self.correr_filtro = False
    else:
      self.get_logger().info("@@@@@@@@@@@@@@@@@@@@@@@Corriendo filtro@@@@@@@@@@@@@@@@@@")
      self.correr_filtro = True
      self.filtro_de_particulas()

  def carlos_termino(self):
    if not self.termino_programa: # Si es que el programa no ha terminado
      msg = String()
      msg.data = "fin_algoritmo"
      self.pub_estado_carlitos.publish(msg)
    else: # Si es que el programa terminó se quedará leyendo en el mismo espacio
      print("Se convergió")
      # self.filtro_de_particulas()

  def recibir_mapa_cb(self, data):
    """
    Método que recibe el mapa y calcula el likelihood field para el modelo
    del sensor.
    """
    print("Recibiendo Mapa?")
    if not self.field_ready:
      recibir_mapa(instancia=self, data=data)
      print("Mapa recibido")
    else:
      return

  def criterio_termino_programa(self, lista_poses_y_bel):
    """
    Método que evalúa si ya se terminó el programa mediante la agrupación de poses
    considerando una cercania inferior self.cercania como suficientemente
    cercana para considerarse de la misma nube.

    Ordena las poses por orden de probabilidad y luego las recorre en busca de
    poses cercanas.

    En detalle, toma una partícula y recorre la lista calculando las diferencias
    en X e Y, si estas son menores suma 1 a sus cercanas. Si la particula no es
    cercana, se recorre un diccionario de particulas populares en busca de la más
    cercana y si no hay ninguna que sea próxima a la particula que estamos viendo,
    entonces añadimos esta al diccionario de populares y repetimos el proceso.

    **Convergencia:** el método converge si es que las partículas se empiezan a
    aproximar, porque cada vez estarán más cercanas entre sí, lo suficiente como
    para que eventualmente una sola sea la popular dentro del diccionario.
    """
    print("Viendo si se logró")
    lista_ordenada = sorted(lista_poses_y_bel, key=lambda x: x[1], reverse=True)
    # Ordenamos según probabilidades para que el proceso sea más eficiente
    # Pero cortaremos la lista para solo considerar las que están dentro del procentaje
    # que siempre se mantiene, es decir, 1 - self.porcenta_nuevas
    lista_ordenada = lista_ordenada[:self.num_particles * (1 - self.porcentaje_nuevos)]

    popular = None # Variable que almacena la particula popular en cuestión
    dict_populares = dict()
    
    def buscar_en_diccionario(instancia, particula, dict_populares, lista_ordenada):
      """
      Función que recorre el diccionario de populares en busca de particulas
      populares que sean cercanas, y en caso de que no reasignan la popular
      y la agregan al diccionario.

      **Retorna:** (popular, dict_populares)

      Se recomienda ver el contenido de esta después de encontrar la
      llamada que se le hace más abajo. Ahí volver a leer los comentarios :)
      """
      alguna_cercana = False # indicador de si se encontró alguna cercana 
      for key in dict_populares.keys(): # Recorremos en busca de una particula cercana
        part_pop = lista_ordenada[int(key)][0] # Extraemos la particula popular
        dif_x = abs(part_pop.x - particula.x)
        dif_y = abs(part_pop.y - particula.y)
        # Y si es que la diferencia es menor al criterio, entonces la particula
        # popular vuelve al trono y le sumamos uno a su conteo
        if dif_x <= instancia.cercania and dif_y <= instancia.cercania:
          alguna_cercana = True
          nueva_popular = (key, part_pop) # Reasignamos 
          dict_populares[key] += 1
          break
      # En caso de que no se haya encontrado ninguna cercana...
      if not alguna_cercana:
        nueva_popular = (str(idx), particula) # Asignamos a la actual particula
        dict_populares[nueva_popular[0]] = 0
      
      return (nueva_popular, dict_populares) # type: ignore
    
    # Recorreremos la lista enumerada para tener acceso a los índices y las partículas
    for idx, (particula, proba) in enumerate(lista_ordenada):
      if not popular: # Si es que no hay alguna popular
        if len(dict_populares) == 0: # Diccionario vacío (para evitar index error)
          popular = (str(idx), particula) # Definimos el [0] como el hash
          dict_populares[popular[0]] = 0
        else: # Recorremos el diccionario en busca de alguna particula cercana
          popular, dict_populares = buscar_en_diccionario(self,
                                                          particula,
                                                          dict_populares,
                                                          lista_ordenada
                                                          )

      else: # Si es que hay una popular, la comparamos con la actual
        dif_x = abs(popular[1].x - particula.x)
        dif_y = abs(popular[1].y - particula.y)
        # Si resulta ser cercana, entonces...
        if dif_x <= self.cercania and dif_y <= self.cercania:
          dict_populares[popular[0]] += 1 # Le sumamos uno a su conteo y seguimos
        else: # En caso de que no sea cercana, debemos revisar el diccionario
          # Buscamos alguna cercana a la actual para no colocar particulas de más
          popular, dict_populares = buscar_en_diccionario(self,
                                                          particula,
                                                          dict_populares,
                                                          lista_ordenada
                                                          )
    # Finalmente, revisaremos el largo del diccionario y si este es de largo 1
    # es porque se encontró la posición más probable
    if len(dict_populares.keys()) == 1:
      print("WOW!!!")
      idx_ganador = int(list(dict_populares.keys())[0])
      part_ganadora = lista_ordenada[idx_ganador][0]
      pose_ganadora = (part_ganadora.x, part_ganadora.y, part_ganadora.ang)
      self.get_logger().info(f"Pose encontrada: {pose_ganadora}")
      return True
    else:
      print("No se convergió, largo =", len(dict_populares.keys()))
      return False

  def recibir_laser_cb(self, data):
    recibir_laser(instancia=self, data=data)

  def create_particles( self, range_x, range_y ):
    """
    Método robado, pero modificado para crear partículas dentro del espacio
    disponible hasta llenar con la cantidad de particulas pedidas.
    Las partículas se crean con sus coordenadas en metros.
    """
    print("Creando partículas")
    while len(self.particles) < self.num_particles:
      x = uniform( range_x[0], range_x[1] )
      y = uniform( range_y[0], range_y[1] )
      print("Par coordenado:", (x, y))
      for (y_esp_d, x_esp_d) in self.espacios_disponibles:
        dif_x = abs(x_esp_d - x)
        dif_y = abs(y_esp_d - y)
        if dif_x <= self.cercania and dif_y <= self.cercania:
          print("particula creada")
          ang = uniform( -np.pi, np.pi )
          new_particle = Particle( x, y, ang, sigma = self.sigma_particulas )
          self.particles.append( new_particle )
      else:
        print("particula fallida")
        continue
    self.publish_particles()
    print("Partículas publicadas")
  
  def resamplear(self, particulas_con_bel):
    """
    Método que toma la lista de particulas con sus respectivos belief y normaliza
    sus bel para que la suma sea 1. Luego, según la probabilidad de cada uno 
    resamplea los elementos en una nueva lista del 90% de elementos iniciales, o
    el resampleo genera la misma cantida de elementos que ya había en caso de ser
    menos que el 90% del número inicial. Esto deja espacio para nuevos en la 
    siguiente iteración del algoritmo.
    **Retorna** una lista con 450 poses.
    """
    print("Resampleando")
    probabilidades = []
    particulas = []
    bel_total = 0
    for particula, bel in particulas_con_bel:
      bel_total += bel

    for particula, bel in particulas_con_bel:
      probabilidades.append(np.divide(bel, bel_total))
      particulas.append(particula)
    
    # Calculamos la cantidad ideal de espacio que debemos dejar para particulas nuevas
    cantidad_sampleo = int(self.num_particles * (1 - self.porcentaje_nuevos))
    # Si tenemos menos partículas que la cantidad ideal, lo dejamos así nomás
    if len(particulas_con_bel) < cantidad_sampleo:
      new_samples = choices(particulas, weights=probabilidades, k=len(particulas))
    # En caso contrario, solo resampleamos hasta alcanzar ese valor
    else:
      new_samples = choices(particulas, weights=probabilidades, k=cantidad_sampleo)
    
    return new_samples

  def update_particles( self, delta_x, delta_y, delta_ang ):
    """
    Mueve todas las partículas y elimina aquellas que estén en zonas prohibidas
    del mapa, o sea, zonas ocupadas o desconocidas.
    """
    print("Actualizando partículas")
    particulas_filtradas = [] # Aquí se almacenan solo las partículas que al moverse no terminan fuera del mapa
    for particle in self.particles:
      particle.move( delta_x, delta_y, delta_ang )
      particula_fuera = False
      for y, x in self.espacios_no_disponibles:
        dist_x = abs(particle.x - x)
        dist_y = abs(particle.y - y)
        # Si es que tenemos menos de un pixel de distancia (en metros), entonces
        # es que estamos "dentro" de un espacio prohibido, si lo vemos como grilla
        if dist_x < self.res and dist_y < self.res:
          particula_fuera = True
          break
      if not particula_fuera: # Si sigue dentro del mapa
        particulas_filtradas.append(particle)
    self.particles = particulas_filtradas.copy()
    print("Partículas publicadas")
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

      pose_array_msg.poses.append(part_pose) # type:ignore
    print("Ahhhhhhhhhh")
    self.pub_particles.publish(pose_array_msg)

  # For testing only:
#   def rotate_particles( self ):
#     self.update_particles( 0, 0, (30*np.pi/180) )
  def odom_callback(self, msg: Odometry):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, self.theta  = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))

    self.x = pos.x
    self.y = pos.y

  def filtro_de_particulas(self):
    """
    Método que es el filtro de particulas que actúa directamente sobre la 
    existencia de partículas que hay en la lista self.particles.
    En la primera iteración crea todas las particulas y en las siguientes
    crea el 10% o más, en caso de que se hayan perdido más al filtrar.
    """
    if self.field_ready:
      print("Corriendo filtro")
    
      # Primero: tiramos a nuestros pequeños (Eso cuando sea la primera vez que lo hacemos)
      # Según el código debería generarse
      self.create_particles( [0, self.ancho * self.res], [0, self.alto * self.res] )

      # Segundo y tercero: calculamos el bel uniforme para cada uno y calculamos su P(z)
      bel_uniforme = Decimal("1") / Decimal(self.num_particles) # Calculamos el bel uniforme
      particulas_y_bel = [] # Lista de listas: [particula, bel]
      for particula in self.particles:
        # Calculamos el P(z|x_i)
        p_z = Decimal(likelihood_field_range_finder_model(self,
                                                  self.mediciones,
                                                  particula,
                                                  self.field
                                                ))
        # Tercero: Ponderamos la partícula y asignamos su nuevo bel
        bel = p_z * bel_uniforme
        particulas_y_bel.append([particula, bel]) # Almacenamos cada par

      # Cuarto: Resampleamos según las probabilidades de cada partícula
      self.particles = self.resamplear(particulas_y_bel)
      self.create_particles( [0, self.ancho * self.res], [0, self.alto * self.res] )

      # Quinto: Movemos las partículas
      delta_x = self.ultima_odom[0] - self.x
      delta_y = self.ultima_odom[1] - self.y
      delta_ang = self.ultima_odom[2] - self.theta

      self.update_particles(delta_x, delta_y, delta_ang)

      # Aprovechando que el robot está quieto actualizamos la Odometria para la siguiente
      self.ultima_odom = (self.x, self.y, self.theta)
      self.termino_programa = self.criterio_termino_programa(particulas_y_bel)
      print("Filtro terminado")
      self.carlos_termino()


if __name__ == '__main__':
  rclpy.init()
  particle_manager = ParticlesManager( num_particles = 100,
                                      porcentaje_nuevos = 0.1,
                                      criterio_cercania = 0.02
  )
  rclpy.spin(particle_manager)
  particle_manager.destroy_node()
  rclpy.shutdown()

