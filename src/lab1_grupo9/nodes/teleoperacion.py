#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from kobuki_ros_interface.msg import BumperEvent


class TeleOperacion( Node ):

    def __init__(self):
        super().__init__("detecta_colisiones")
        self.timer = self.create_timer( 0.1, self.callback )

        self.tecla = None # representa la tecla presionada
        self.chocamos = False # booleano que indica si se colisionó o no

        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular

        #Creamos nuestro publisher de velocidad
        self.enviar_velocidad = self.create_publisher(
            Twist,
            "/commands/velocity",
            1
        )
        # Creamos el mensaje para este tópico
        self.speed = Twist()

        # Nos suscribimos para leer las señales del bumper
        self.recibir_bumper = self.create_subscription(
            BumperEvent,
            "/events/bumper",
            self.detectar_choque,
            1
        )
        
        #Creamos un timer que estará administrando las velocidades cada 0.1 segundos segun la tecla recibida
        self.timer_mover = self.create_timer(0.1, self.admin_movement)


        # Cosas para administrar la entrada de teclas
        self.orden = "" # Variable que se procesa para luego pasarse a self.tecla
        self.presionado = threading.Event() # Flag para reconocer cuando se presiona una tecla
        self.hilo_teclas = threading.Thread(target= self.recibir_teclas) # Hilo que maneja teclado
        self.hilo_teclas.start() 

    def admin_movement( self ):
        """
        Función principal encargada del movimiento y ejecutada por el timer 
        cada 0.1 s. Trabaja en conjunto con el hilo que detecta las teclas.
        """
        # Si es que se presionó una tecla y no chocamos
        if self.presionado.is_set() and not self.chocamos :
            self.tecla = self.orden
            # print("Tecla:", orden, "\n-------------")
            self.presionado.clear() # Con esto indicamos que ya no se está presionando tecla alguna
            self.detectar_tecla()
        # En caso de que no haya tecla o hayamos chocado
        else:
            # detener todo movimiento al no recibir inputs
            self.frenar()
            pass
    
    def detectar_choque( self, bumper: BumperEvent):
        """
        Método que estará constantemente leyendo para saber si debemos leer
        el teclado o no.
        """
        if bumper.state == 0:
            # Si no se ha detectado un choque se continua con el procesamiento de la tecla
            self.chocamos = False
        else:
            # Si se detecta un choque, se envía un freno instantáneamente
            self.chocamos = True
            self.frenar()

    def frenar( self ):
        """
        Función que frena inmediatamente al robotito
        """
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.move()

    def detectar_tecla( self ):
        """
        Método que según la tecla recibida ejecutará alguna acción si corresponde
        """
        # Vel. lin. 0.2 m/s
        if self.tecla.lower() == "i":
            self.speed.linear.x = self.max_v
            
        # Vel. lin. - 0.2 m/s
        elif self.tecla.lower() == "j":
            self.speed.linear.x = -1 * self.max_v

        # Rota a 1.0 rad/s
        elif self.tecla.lower() == "a":
            self.speed.angular.z = self.max_w

        # Rota a - 1.0 rad/s
        elif self.tecla.lower() == "s":
            self.speed.angular.z = -1 * self.max_w

        # Vel. lin. 0.2 m/s y rota a 1.0 rad/s
        elif self.tecla.lower() == "q":
            self.speed.linear.x = self.max_v
            self.speed.angular.z = self.max_w

        # Vel. lin. 0.2 m/s y rota a - 1.0 rad/s
        elif self.tecla.lower() == "w":
            self.speed.linear.x = self.max_v
            self.speed.angular.z = -1 * self.max_w
        else:
            pass
        self.move()

    def move(self):
        """
        Envía la velocidad en estructura Twist, publicándola en el tópico
        """
        self.enviar_velocidad.publish(self.speed)

    def recibir_teclas(self):
        """
        Método encargado de recibir las teclas desde el teclado y entregarlas al
        resto del código, filtrando y admitiendo la funcionalidad de poder mantener
        presionado "Enter" para repetir comandos.
        """

        # Se creó un "historial" de la última tecla presionada, para poder repetirla
        # a la hora de presionar "Enter en el teclado" y así repetimos la orden previa
        historial = [""]

        while True:
            entrada = input("Accion:")
            # Procesamos la entrada para robustecer el sistema
            if len(entrada) > 1:
                letras = [caracter for caracter in entrada]
                letra_init = letras[0]
                # Ahora corroboramos que se haya escrito una entrada al menos coherente
                # es decir, al menos muchas letras iguales. Si una es distinta no sirve.
                for letra in letras:
                    if letra != letra_init:
                        letra_init = False
            else:
                letra_init = entrada

            if letra_init != False:
                self.presionado.set() # Indicamos que llegó una tecla
                if letra_init == "":
                    self.orden = historial[0] # Repetimos la última tecla
                else:
                    self.orden = letra_init # Entregamos la tecla ingresada
                    historial[0] = self.orden # Actualizamos el historial
            else:
                historial[0] = "" # Reiniciamos el historial para evitar errores
                print("Comando inválido.")


if __name__ == '__main__':
  rclpy.init()
  teleoperador_node = TeleOperacion()
  rclpy.spin( teleoperador_node )
  teleoperador_node.destroy_node()
  rclpy.shutdown()
