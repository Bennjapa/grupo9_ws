import rclpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from rclpy.node import Node

class pose_loader(Node):
    
    def __init__(self):
        super().__init__("pose_loader")

        self.enviar_poses = self.create_publisher(
            PoseArray,
            "goal_list",
            10
        )
        self.timer = self.create_timer(0.2, self.leer_poses)
    
    def leer_poses(self):
        pass
    