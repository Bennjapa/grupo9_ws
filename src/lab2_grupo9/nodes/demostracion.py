#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import threading

class NodoVentana(Node):
    def __init__(self):
        super().__init__("nodo_con_matplotlib")

        # Datos del robot y path
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.path = []
        self.path_leido = False

        self.historial_posiciones = []
        self.historial_odometria = []

        # Subscripciones
        self.create_subscription(Pose, "/real_pose", self.pose_cb, 10)
        self.create_subscription(Path, "nav_plan", self.path_cb, 10)
        self.create_subscription(Odometry, "/odom", self.leer_odometria, 10)

    def pose_cb(self, pose: Pose):
        self.pos_x = pose.position.x
        self.pos_y = pose.position.y
        self.historial_posiciones.append((self.pos_x, self.pos_y))

    def path_cb(self, msg: Path):
        if not self.path_leido:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(self.path) > 20:
                self.path_leido = True
                self.get_logger().info(f"Path recibido con {len(self.path)} puntos.")

    def leer_odometria(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.historial_odometria.append((x, y))


def iniciar_vista(nodo):
    fig, ax = plt.subplots()
    ax.set_title("Nav.Plan, odometría y real pose")
    ax.set_xlim(-1, 7)
    ax.set_ylim(-1, 7)
    ax.set_aspect('equal')

    path_plot, = ax.plot([], [], 'bo', markersize=2, label="Nav.Plan")
    recorrido_plot, = ax.plot([], [], 'r-', linewidth=1, label="Recorrido real")
    odom_plot, = ax.plot([], [], 'y--', linewidth=1, label="Odometría")
    robot_plot, = ax.plot([], [], 'ro', label="Posición actual")

    ax.legend()

    def actualizar(frame):
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

    ani = FuncAnimation(fig, actualizar, interval=100)
    plt.show()


def spin_ros(nodo):
    while rclpy.ok():
        rclpy.spin_once(nodo, timeout_sec=0.01)


def main():
    rclpy.init()
    nodo = NodoVentana()

    # Iniciar el hilo de ROS
    thread_ros = threading.Thread(target=spin_ros, args=(nodo,), daemon=True)
    thread_ros.start()

    # Iniciar visualización matplotlib (bloqueante)
    try:
        iniciar_vista(nodo)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
