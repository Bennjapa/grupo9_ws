#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import BumperEvent

import sys
import select
import tty
import termios
import os

class TeleOperacion( Node ):

    def __init__(self):
        super().__init__("detecta_colisiones")

        self.tecla = None # representa la tecla presionada
        self.chocamos = False # booleano que indica si se colisionó o no

        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular

        #Creamos nuestro publisher de velocidad
        self.enviar_velocidad = self.create_publisher(
            Twist,
            "/cmd_vel_mux/input/navigation",
            10
        )
        # Creamos el mensaje para este tópico
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        # Nos suscribimos para leer las señales del bumper
        self.recibir_bumper = self.create_subscription(
            BumperEvent,
            "/events/bumper",
            self.detectar_choque,
            10
        )

        # Cofigurar el terminal
        self.fd = sys.stdin.fileno()
        self.config_vieja = self.terminal_modo_rafaga()
        
        #Creamos un timer que estará administrando las velocidades cada 0.1 segundos segun la tecla recibida
        self.timer_mover = self.create_timer(0.1, self.admin_movement)

    def admin_movement( self ):
        """
        Función principal encargada del movimiento y ejecutada por el timer 
        cada 0.1 s. Trabaja en conjunto con el hilo que detecta las teclas.
        """
        if select.select([sys.stdin], [], [], 0.0)[0]:
            self.tecla = sys.stdin.read(1)
            self.vaciar_entradas()
        self.detectar_tecla()
    
    def detectar_choque(self, bumper: BumperEvent):
        """
        Método que estará constantemente leyendo para saber si debemos leer
        el teclado o no.
        """
        self.get_logger().info("Detecte un choque")
        if bumper.state == 0:
            # Si no se ha detectado un choque se continua con el procesamiento de la tecla
            self.chocamos = False
        else:
            # Si se detecta un choque, se envía un freno instantáneamente
            self.chocamos = True
            self.frenar()

    def frenar(self):
        """
        Función que frena inmediatamente al robotito
        """
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.move()

    def detectar_tecla(self):
        """
        Método que según la tecla recibida ejecutará alguna acción si corresponde
        """
        # Vel. lin. 0.2 m/s
        if not self.chocamos:
            if self.tecla == "i":
                self.speed.linear.x = self.max_v

            # Vel. lin. - 0.2 m/s
            elif self.tecla == "j":
                self.speed.linear.x = -1 * self.max_v

            # Rota a 1.0 rad/s
            elif self.tecla == "a":
                self.speed.angular.z = self.max_w

            # Rota a - 1.0 rad/s
            elif self.tecla == "s":
                self.speed.angular.z = -1 * self.max_w

            # Vel. lin. 0.2 m/s y rota a 1.0 rad/s
            elif self.tecla == "q":
                self.speed.linear.x = self.max_v
                self.speed.angular.z = self.max_w

            # Vel. lin. 0.2 m/s y rota a - 1.0 rad/s
            elif self.tecla == "w":
                self.speed.linear.x = self.max_v
                self.speed.angular.z = -1 * self.max_w
            
            else:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0

        self.move()

    def move(self):
        """
        Envía la velocidad en estructura Twist, publicándola en el tópico
        """
        self.enviar_velocidad.publish(self.speed)
    
    def terminal_modo_rafaga(self):
        config_vieja = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return config_vieja
    
    def restaurar_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.config_vieja)
    
    def vaciar_entradas(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            os.read(sys.stdin.fileno(), 1)


if __name__ == '__main__':
  
    rclpy.init()
    teleoperador_node = TeleOperacion()
    rclpy.spin( teleoperador_node )
    teleoperador_node.restaurar_terminal()
    teleoperador_node.destroy_node()
    rclpy.shutdown()
