#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import numpy as np
import time

class SimulacionTrayectoria(Node):
    def __init__(self):
        super().__init__('simulacion_node')

        self.sub_traj = self.create_subscription(
            Float32MultiArray,
            '/robot_traj',
            self.callback_trayectoria,
            10
        )

        self.pub_marker = self.create_publisher(Marker, '/visualization_marker', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.l0 = 0.0905 
        self.l1 = 0.20   
        self.l2 = 0.20   

        #  estado inicial
        self.last_q1 = 180.0 - 40.0 
        self.last_q2 = -10.0
        self.mover_robot_virtual(self.last_q1, self.last_q2)

        self.get_logger().info(f"Simulación Lista. Inicio corregido en: ({self.last_q1:.1f}, {self.last_q2:.1f})")

    def forward_kinematics(self, q1_deg, q2_deg):
        t1 = math.radians(q1_deg)
        t2 = math.radians(q2_deg)
        ex1 = -self.l0 + self.l1 * math.cos(t1)
        ey1 =            self.l1 * math.sin(t1)

        ex2 =  self.l0 + self.l1 * math.cos(t2)
        ey2 =            self.l1 * math.sin(t2)
        d_sq = (ex2 - ex1)**2 + (ey2 - ey1)**2
        d = math.sqrt(d_sq)

        if d > (2 * self.l2) or d == 0:
            return None, None 

        a = (self.l2**2 - self.l2**2 + d_sq) / (2 * d) 
        h = math.sqrt(max(0, self.l2**2 - a**2))

        x2 = ex1 + a * (ex2 - ex1) / d
        y2 = ey1 + a * (ey2 - ey1) / d

        x_final = x2 - h * (ey2 - ey1) / d
        y_final = y2 + h * (ex2 - ex1) / d

        return x_final, y_final

    def calcular_angulos_codos(self, x, y):
        d_izq = math.sqrt((-self.l0 - x)**2 + y**2)
        d_der = math.sqrt(( self.l0 - x)**2 + y**2)

        cos_gamma_izq = (self.l1**2 + self.l2**2 - d_izq**2) / (2 * self.l1 * self.l2)
        q2_izq_rad = math.acos(max(-1.0, min(1.0, cos_gamma_izq))) - math.pi

        cos_gamma_der = (self.l1**2 + self.l2**2 - d_der**2) / (2 * self.l1 * self.l2)
        q2_der_rad = math.pi - math.acos(max(-1.0, min(1.0, cos_gamma_der)))

        return q2_izq_rad, q2_der_rad

    def mover_robot_virtual(self, q1_deg, q2_deg):
        x, y = self.forward_kinematics(q1_deg, q2_deg)
        
        if x is not None:
            q2_izq, q2_der = self.calcular_angulos_codos(x, y)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['motor_izq', 'codo_izq', 'motor_der', 'codo_der']
            msg.position = [
                math.radians(q1_deg), 
                float(q2_izq), 
                math.radians(q2_deg), 
                float(q2_der)
            ]
            self.pub_joints.publish(msg)

    def callback_trayectoria(self, msg):
        if len(msg.data) < 2: return

        raw_q1 = msg.data[0]
        raw_q2 = msg.data[1]

        # 180 - angulo1, para evitar colision
        target_q1 = 180.0 - raw_q1
        target_q2 = raw_q2

        self.get_logger().info(f"Trayectoria: ({self.last_q1:.1f}, {self.last_q2:.1f}) -> ({target_q1:.1f}, {target_q2:.1f})")

        points_list = []
        STEPS = 40 

        for i in range(STEPS + 1):
            t = i / float(STEPS)
            
            q1_curr = self.last_q1 + (target_q1 - self.last_q1) * t
            q2_curr = self.last_q2 + (target_q2 - self.last_q2) * t

            x, y = self.forward_kinematics(q1_curr, q2_curr)

            if x is not None:
                p = Point()
                p.x, p.y, p.z = float(x), float(y), 0.0
                points_list.append(p)
                
                self.publicar_marker(points_list)

                self.mover_robot_virtual(q1_curr, q2_curr)
                
                time.sleep(0.02) 
        self.last_q1 = target_q1
        self.last_q2 = target_q2

    def publicar_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "traj_simulada"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.005 
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 
        marker.points = points
        marker.lifetime.sec = 15 
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SimulacionTrayectoria()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()