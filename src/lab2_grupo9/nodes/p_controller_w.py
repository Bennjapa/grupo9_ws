#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import String
import numpy as np
from math import atan2, sin, cos

class PController(Node):
    
    def __init__(self, kp):
        super().__init__("p_controller")
        self.kp = kp
        self.setpoint = None
        self.state = None

        self.control_effort_pub = self.create_publisher( Float64, "control_effort_w", 1) 
        self.setpoint_sub = self.create_subscription( Float64, "setpoint_w", self.definir_setpoint, 1)
        self.state_sub = self.create_subscription( Float64, "state_w", self.recibir_estado, 1)
        self.controlling_sub = self.create_subscription(String, "controlling_w", self.controlando, 1)


    def definir_setpoint(self, msg:Float64):
        self.get_logger().info(f"[PICTRL] nuevo setpoint recibido: {msg.data}")
        self.get_clock()
        self.reset()
        self.setpoint = msg.data    
    def recibir_estado(self, msg:Float64):

        if self.setpoint == None:
            return
        self.state = msg.data
        
        #Definimos nuestro error como deseado - actual
        #Phase unwrapping
        
        error = self.setpoint - self.state
        error = atan2(sin(error),cos(error))

        #Proporcional
        p_actuation = self.kp*error

        #actuacion
        actuation = p_actuation 

        #Enviar mensaje
        msg = Float64()
        msg.data = actuation
        self.control_effort_pub.publish( msg )
    
    def controlando(self, msg:String):
        estado = msg.data
        if estado == "stop":
            self.reset()
            msg = Float64()
            msg.data = 0.0
            self.control_effort_pub.publish( msg )

    def reset(self):
        self.setpoint = None
        self.state = None

def main():
    rclpy.init()
    p_ctrl = PController(0.6)
    rclpy.spin(p_ctrl)

if __name__ == "__main__":
    main()