import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3

class obstacle_detector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        self.image_sub = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.image_callback,
            10
        )
        self.obstacle_pub = self.create_publisher(
            Vector3,
            "/occupancy_state",
            10
        )
        self.get_logger().info("Nodo detector de obstáculos iniciado")

        self.vector = Vector3()
        self.vector.x = 0.0
        self.vector.y = 0.0
        self.vector.z = 0.0

    def image_callback(self, data):
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        altura, ancho = self.current_cv_depth_image.shape
        profundidad_max = 0.5 # 50 cm

        tercio = ancho // 3 # dividimos la imagen en tres partes

        zona_izquierda = self.current_cv_depth_image[:, :tercio]
        zona_central = self.current_cv_depth_image[:, tercio:2*tercio]
        zona_derecha = self.current_cv_depth_image[:, 2*tercio:]

        def hay_obstaculo(zona):
            return np.any((zona >= 0.0) & (zona <= profundidad_max))
        izquierda = 1.0 if hay_obstaculo(zona_izquierda) else 0.0
        centro = 1.0 if hay_obstaculo(zona_central) else 0.0
        derecha = 1.0 if hay_obstaculo(zona_derecha) else 0.0

        self.vector.x = izquierda
        self.vector.y = centro
        self.vector.z = derecha

        self.obstacle_pub.publish(self.vector)
        self.get_logger().info(f"Obstáculo detectado: izquierda={izquierda}, centro={centro}, derecha={derecha}")


if __name__ == '__main__':
    rclpy.init()
    kinect_node = obstacle_detector()
    rclpy.spin(kinect_node)
