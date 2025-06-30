from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from scipy.stats import norm
from math import sin, cos
import numpy as np
import cv2

"""
En este archivo se colocaron todos los métodos del modelo del sensor para
no levantar varios tópicos con toda la info y poder procesar más fácilmente
todo en particles_manager.py
"""

def likelihood_field_range_finder_model(instancia, mediciones, particula, campo):
    """
    Función que retorna la probabilidad del modelo del sensor o P(z|x)
    """
    theta = particula.ang
    q = 1 #Inicio de la productoria
    angulo = instancia.angle_min #Angulo mínimo que mide el sensor

    rayos_validos = 0
    FOV = 57 * np.pi / 180
    for i in mediciones:
        if i != instancia.zmax and not np.isnan(i) and angulo < FOV/2 and angulo > -FOV/2:
            rayos_validos += 1

            x_real = particula.x * instancia.res #Transformamos x a [m]
            y_real = (instancia.alto - particula.y - 1) * instancia.res #Transformamos y a [m] y lo ponemos en coordenadas reales
            #Aplicamos formulas de clases slide 38
            x = x_real + instancia.sens_x*cos(theta) - instancia.sens_y*sin(theta) + i*cos(theta + angulo)
            y = y_real + instancia.sens_y*cos(theta) + instancia.sens_x*sin(theta) + i*sin(theta + angulo)

            pixel_x = int(x / instancia.res) #Pasamos la posicion x del final del laser a pixeles
            pixel_y = instancia.alto - int(y / instancia.res) - 1 #Pasamos la posicion y del final del laser a pixeles

            if pixel_x < 0:
                pixel_x = 0
            elif pixel_x > instancia.ancho - 1:
                pixel_x = instancia.ancho - 1
            if pixel_y < 0:
                pixel_y = 0
            elif pixel_y > instancia.alto - 1:
                pixel_y = instancia.alto - 1

            prob = campo[pixel_y, pixel_x]
            if prob > 0:
                q = q*(instancia.zhit * prob)
            else:
                q *= 1e-6
        
        angulo += instancia.increment #Sumamos el incremento para el siguiente ángulo
    # print(angulo)
    if rayos_validos == 0 or np.isnan(q):
        q = 1e-6

    return q

def recibir_mapa(instancia, data : OccupancyGrid):
    instancia.ancho = data.info.width
    instancia.alto = data.info.height
    instancia.res = data.info.resolution
    instancia.origin = [data.info.origin.position.x, data.info.origin.position.y, 0.0]
    likelihood_field(instancia=instancia, mapa="mapa.pgm")

def likelihood_field(instancia, mapa : str):
    img = cv2.imread(mapa, cv2.IMREAD_GRAYSCALE) #Leemos la imagen
    alto, ancho = img.shape #Obtenemos las dimensiones de la imagen
    posiciones_ocupadas = [] #Creamos una lista vacia para ingresar posiciones ocupadas
    for y in range(alto): #Recorremos la imagen, para ver las posiciones con obstaculos
        for x in range(ancho):
            if img[y,x] == 0: #Ingresamos posiciones con obstaculos
                posiciones_ocupadas.append((y,x))
            elif img[y,x] == 255: #Ingresamos posiciones desocupadas
                instancia.espacios_disponibles.append((y,x)) # Agregamos las posiciones libres
    #Convertimos en arrays de numpy ambas listas
    posiciones_ocupadas = np.array(posiciones_ocupadas)
    instancia.get_logger().info("Terminamos el mapa :D") 

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
                    dist = np.sqrt(((y-i[0])*instancia.res)**2 + ((x-i[1])*instancia.res)**2) #Obtenemos la distancia al obstaculo desde cada pixel
                    distancias[y,x] = dist

        normal_distribution = norm.pdf(distancias, loc = 0, scale=instancia.sigma_sensor) #Calculamos la probabilidad de todas las distancias cercanas al obstaculo

        for y in range(i[0] - n, i[0]+ n): #Recorremos de nuevo para sumar al campo las probabilidades
            for x in range(i[1] - n, i[1]+ n):
                if y < alto and y >= 0 and x < ancho and x >= 0:
                    field[y,x] += normal_distribution[y,x]

        instancia.get_logger().info(f"Completadas {contador} de {largo}")
        contador += 1
        
    instancia.field = field/field.max() #Normalizamos el campo para que el máximo de probabilidad sea 1
    instancia.field_ready = True #Indicamos que el campo esta listo
    instancia.get_logger().info("Estamos listos con el mapa :D")

def recibir_laser(instancia, data : LaserScan):

    # Cambiar posiciones desocupadas por poses de los posibles robots

    instancia.mediciones = data.ranges #Listado de mediciones
    instancia.angle_min = data.angle_min #ángulo minimio en el que mide el sensor
    instancia.increment = data.angle_increment #cada cuanto incrementa el angulo por medición

    return













    if instancia.field_ready: #Si esta listo el campo de distancias
        valores_verosimilitud = np.zeros((self.alto, self.ancho)) #array vacio, con donde puede estar el robot
        
        # Esto está calculando el valor más probable para cada partícula y eligiendo la más probable
        for particula in instancia.particles: #Probamos con todas las posiciones x,y desocupadas
            p_z = likelihood_field_range_finder_model(instancia, mediciones, particula, instancia.field) #obtenemos la probabilidad de la posición
            valores_verosimilitud[particula.x, particula.y] = p_z #Aqui guardamos en el mapa de verosimilitud la probabilidad de la posición (y,x)
        valores_verosimilitud_normalizado = valores_verosimilitud / np.max(valores_verosimilitud) #Normalizamos el mapa de verosimilitud
        mapa_de_posicion = (valores_verosimilitud_normalizado * 255).astype(np.uint8) #Aqui transformamos a escala de grises

