#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry

import matplotlib.pyplot as p
from matplotlib.animation import FuncAnimation

import threading

class NodoVentana(Node):
    def __init__(self):
        super().__init__("nodo_demostracion")

        # Variables para la lectura del path
        self.path = []
        self.path_leido = False

        # Variables para comparar posicion real con la odometría
        self.historial_posiciones = []
        self.historial_odometria = []

        # Nos suscribimos a la pose real
        self.leer_pose = self.create_subscription(
            Pose,
            "/real_pose",
            self.pose_callback,
            10
        )
        # Nos suscribimos al nav_plan
        self.leer_path = self.create_subscription(
            Path,
            "nav_plan",
            self.path_callback,
            10
        )
        # Nos suscribimos a la odometría
        self.leer_odo = self.create_subscription(
            Odometry,
            "/odom",
            self.leer_odometria,
            10
        )

    def pose_callback(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        self.historial_posiciones.append((x, y))

    def path_callback(self, path: Path):
        if not self.path_leido:
            self.path = [(coordenadas.pose.position.x, coordenadas.pose.position.y) for coordenadas in path.poses]
            if len(self.path) > 20:
                self.path_leido = True
                self.get_logger().info(f"Path recibido con {len(self.path)} puntos.")

    def leer_odometria(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.historial_odometria.append((x, y))

# ------------------------------- Hasta aquí el nodo --------------------------------


def iniciar_vista(nodo):
    """
    Método para graficar la info mediante matplotlib
    """
    figura, ejes = p.subplots()

    ejes.set_title("Nav.Plan, odometría y real pose")
    ejes.set_xlim(-1, 7)
    ejes.set_ylim(-1, 7)
    ejes.set_aspect('equal')

    path_plot, = ejes.plot([], [],
                           'bo', # b de blue 
                           markersize = 2,
                           label = "Nav.Plan"
                        )
    recorrido_plot, = ejes.plot([], [],
                                'r-', # r de red y punteada
                                linewidth = 1,
                                label = "Recorrido real"
                        )
    odom_plot, = ejes.plot([], [],
                           'y--', # y de yellow y linea doble punteada
                           linewidth = 1,
                           label = "Odometría"
                        )
    robot_plot, = ejes.plot([], [],
                            'ro', # r de red y una "pelota"
                            label = "Posición actual"
                        )

    ejes.legend() # Activamos la leyenda

    def actualizar_plot(frame): # frame es un argumento que usa la función FuncAnimation
        # y que le entrega al método en cada iteración
        """
        Método encargado de actualizar el gráfico según la info recibida
        """
        # Actualizar recorrido real
        if nodo.historial_posiciones:
            x_hist, y_hist = zip(*nodo.historial_posiciones)
            recorrido_plot.set_data(x_hist, y_hist)
            robot_plot.set_data(x_hist[-1], y_hist[-1])

        # Dibujar odometría
        if nodo.historial_odometria:
            x_odom, y_odom = zip(*nodo.historial_odometria)
            odom_plot.set_data(x_odom, y_odom)

        # Dibujar path una vez
        if nodo.path:
            x_path, y_path = zip(*nodo.path)
            path_plot.set_data(x_path, y_path)

        return path_plot, recorrido_plot, odom_plot, robot_plot

    animacion = FuncAnimation(figura, actualizar_plot, interval=100) # FuncAnimation se encarga de "animar" algo con un intervalo de tiempo
    # Y se guarda en una variable porque si no, python lo borra y no se actualiza el gráfico.

    p.show()


def spin_ros(nodo):
    """
    Método encargado de ejecutar el nodo una vez y con un timeout máximo de 0.01
    """
    while rclpy.ok(): # Mientras el nodo esté activo hacer spin_once
        rclpy.spin_once(nodo, timeout_sec=0.01)


if __name__ == "__main__":

    rclpy.init()
    nodo = NodoVentana()

    # Iniciamos el thread de ros2
    hilo_ros = threading.Thread(target = spin_ros, args = (nodo,), daemon = True)
    hilo_ros.start()

    # Visualización matplotlib
    try:
        iniciar_vista(nodo)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
