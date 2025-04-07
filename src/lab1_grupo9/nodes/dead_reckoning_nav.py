import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray

class dead_reckoning_nav(Node):
    def __init__(self):
        super.__init__("dead_reckoning_nav")
        self.max_v = 0.2 # [m/s] maxima velocidad lineal
        self.max_w = 1.0 # [rad/s] maxima velocidad angular
        
        #Creamos nuestro publisher de velocidad
        self.enviar_velocidad = self.create_publisher(
            Twist,
            "/cmd_vel_mux/input/navigation",
            10)
        self.recibir_poses = self.create_subscription(
            PoseArray,
            "goal_list",
            self.accion_mover_cb,
            10
        )
        #Creamos nuestro mensaje de velocidad
        self.speed = Twist()
        self.speed.linear.x = 0
        self.speed.angular.z = 0

        #Creamos un timer que estara enviando velocidades cada 0.2 segundos
        self.timer_mover = self.create_timer(0.2, self.move)


    def aplicar_velocidad(self, speed_command_list):
        for i in  speed_command_list:
            self.speed.linear.x = i[0]
            self.speed.angular.z = i[1]
            time.sleep(i[2])
            #Esta parte del codigo igual es como prueba desconozco si el sleep
            #tambien dormira el timer, en caso de que si, esto no funcionara.
        
        #Una vez se corre la lista se detiene el robot
        self.speed.linear.x = 0
        self.speed.angular.z = 0
    
    def mover_robot_a_destino(self, goal_pose):
        x = goal_pose[0]
        y = goal_pose[1]
        theta = goal_pose[2]
        t1 = x/self.max_v
        t2 = theta/self.max_w
        t3 = y/self.max_v
        lista_de_comandos = [
            (self.max_v, 0, t1),
            (0, self.max_w, t2)
            (self.max_v, 0, t3)
        ]
        self.aplicar_velocidad(lista_de_comandos)

    def accion_mover_cb(self, goal_list: PoseArray):
        pass

    def move(self):
        self.enviar_velocidad.publish(self.speed)

if __name__ == "__main__":
    rclpy.init()
    nodo = dead_reckoning_nav()
    rclpy.spin(nodo)
