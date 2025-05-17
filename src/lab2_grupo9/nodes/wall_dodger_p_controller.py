#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty


class PController( Node ):

  def __init__( self, kp):
    super().__init__( 'wp_controller' )
    self.kp = kp
    self.state = None
    self.proportional_action = 0
    self.get_logger().info( 'PController node started' )

    self.actuation_pub = self.create_publisher( Float64, '/control_action', 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )

  def state_cb( self, msg: Float64 ):
    self.state = msg.data
    error = self.state

    # Proportional
    p_actuation = self.kp*error

    # Actuation
    actuation = p_actuation

    # Message sending
    msg = Float64()
    msg.data = actuation
    self.actuation_pub.publish( msg )
    # self.get_logger().info( f'Actuation: {msg.data}' )

  def reset( self ):
    self.setpoint = None
    self.state = None


def main():
  rclpy.init()
  p_ctrl = PController(1.2)
  rclpy.spin( p_ctrl )

if __name__ == '__main__':
  main()



