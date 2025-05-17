#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import String
import numpy as np

class PIController(Node):
    
    def __init__(self, kp, ki):
        super().__init__("pi_controller")
        self.kp = kp
        self.ki = ki
        self.setpoint = None
        self.state = None
        self.error_integral = 0

        self.control_effort_pub = self.create_publisher( Float64, "control_effort_w", 1) 
        self.setpoint_sub = self.create_subscription( Float64, "setpoint_w", self.definir_setpoint, 1)
        self.state_sub = self.create_subscription( Float64, "state_w", self.recibir_estado, 1)
        self.controlling_sub = self.create_subscription( String, "controlling_w", self.controlando, 1)

    def definir_setpoint(self, msg:Float64):
        self.get_logger().info(f"[PICTRL] nuevo setpoint recibido: {msg.data}")
        self.get_clock()
        self.tiempo_anterior = (self.get_clock().now().nanoseconds) * 1e-9
        self.reset()
        self.setpoint = msg.data
        self.error_integral = 0
    
    def recibir_estado(self, data:Float64):
        tiempo_actual = (self.get_clock().now().nanoseconds) * 1e-9

        if self.setpoint == None:
            return
        self.state = data.data
        
        dt = tiempo_actual - self.tiempo_anterior
        
        #Definimos nuestro error como deseado - actual
        error = self.setpoint - self.state
        if error >= 0:
            error = error % 2*np.pi
        else:
            error = -(error % 2*np.pi)
        self.error_integral += error*dt
        self.tiempo_anterior = tiempo_actual

        #Proporcional
        p_actuation = self.kp*error

        #Integral
        i_actuation = self.ki*self.error_integral

        #actuacion
        actuation = p_actuation + i_actuation 

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
    pi_ctrl = PIController(0.1, 0.05)
    rclpy.spin(pi_ctrl)

if __name__ == "__main__":
    main()