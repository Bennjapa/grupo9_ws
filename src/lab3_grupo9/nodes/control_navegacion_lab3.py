#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty


class PController( Node ):

  def __init__( self, kp):
    super().__init__( 'wp_controller' )
    self.kp = kp
    self.state = None
    self.proportional_action = 0
    self.get_logger().info( 'PController node started' )

    self.actuation_pub = self.create_publisher( Vector3, '/control_action', 1 )
    self.dist_state_sub = self.create_subscription( Vector3, 'state', self.state_cb, 1 )

  def state_cb( self, msg: Vector3 ):
    izq = msg.x
    centro = msg.y
    der = msg.z
    min_der_dist = 0.7
    error_derecha = min_der_dist - der

    self.get_logger().info( f'State: {error_derecha}' )

    # Actuation
    if centro < 0.3:
      velocidad = 0.0
      actuation_angular = 1.0
    else:     
      actuation_angular = self.kp * error_derecha
      velocidad = 0.2

    self.get_logger().info( f'Angular: {actuation_angular}, Velocidad: {velocidad}' )

    # Message sending
    act_msg = Vector3()
    act_msg.x = actuation_angular
    act_msg.y = velocidad
    act_msg.z = 0.0
    self.actuation_pub.publish( act_msg )
    # self.get_logger().info( f'Actuation: {msg.data}' )

  def reset( self ):
    self.setpoint = None
    self.state = None


def main():
  rclpy.init()
  p_ctrl = PController(0.8)
  rclpy.spin( p_ctrl )

if __name__ == '__main__':
  main()



