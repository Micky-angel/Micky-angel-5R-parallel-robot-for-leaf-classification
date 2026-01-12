#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import math
import numpy as np

class CalculoCinematica(Node):
    def __init__(self):
        super().__init__('cinematica_node')
        
        self.subscription = self.create_subscription(
            String, '/cinematica/request', self.calcular_callback, 10
        )
        self.publisher_ = self.create_publisher(JointState, '/degrees', 10)
        # CALIBRACIÓN
        self.CAM_W = 640.0
        self.CAM_H = 360.0 
        
        self.REAL_WIDTH_VIEW  = 0.29
        self.REAL_HEIGHT_VIEW = 0.16
        
        self.Y_OFFSET_BASE = 0.12
        
        # ROI PIXELES
        self.ROI_X_MIN, self.ROI_X_MAX = 0, 640
        self.ROI_Y_MIN, self.ROI_Y_MAX = 0, 360
        
        #  EJE X
        self.CALIB_X_OFFSET = +0.005
        self.CALIB_X_SCALE  = 1.15

        #  EJE Y 
        self.CALIB_Y_OFFSET = -0.001
        self.CALIB_Y_SCALE  = 2 
        
        #  PARÁMETROS DEL ROBOT
        self.l0 = 0.0905 
        self.l1 = 0.20   
        self.l2 = 0.20   
        
        self.L2_MAX_SAFE_ANGLE_AT_EF = math.radians(170.0)
        self.MOTOR1_MIN_DEG, self.MOTOR1_MAX_DEG = -15.0, 180.0
        self.MOTOR2_MIN_DEG, self.MOTOR2_MAX_DEG = 0.0, 180.0
        self.min_elbow_dist = 0.04 
        self.min_y_height = -0.05 
        
        self.REAL_X_MIN = -(self.REAL_WIDTH_VIEW / 2) - 0.05
        self.REAL_X_MAX =  (self.REAL_WIDTH_VIEW / 2) + 0.05
        self.REAL_Y_MIN = self.Y_OFFSET_BASE
        self.REAL_Y_MAX = self.Y_OFFSET_BASE + self.REAL_HEIGHT_VIEW + 0.05

        self.generar_curva_limite()
        self.get_logger().info("Nodo Cinemática LISTO.")

    def check_pixel_roi(self, px, py):
        if (px >= self.ROI_X_MIN and px <= self.ROI_X_MAX and
            py >= self.ROI_Y_MIN and py <= self.ROI_Y_MAX):
            return True
        return False

    def pixel_to_meters(self, px, py):
        scale_x = self.REAL_WIDTH_VIEW / self.CAM_W
        scale_y = self.REAL_HEIGHT_VIEW / self.CAM_H

        # --- EJE X ---
        center_x_px = self.CAM_W / 2.0
        mx_raw = (px - center_x_px) * scale_x
        mx_scaled = mx_raw * self.CALIB_X_SCALE

        offset_x_mag = abs(self.CALIB_X_OFFSET)
        if mx_scaled < 0:
            mx_final = mx_scaled - offset_x_mag
        else:
            mx_final = mx_scaled + offset_x_mag

        # EJE Y
        dist_from_bottom_meters = (self.CAM_H - py) * scale_y
        my_raw = self.Y_OFFSET_BASE + dist_from_bottom_meters

        pixel_ref_y = 85.0
        dist_ref_from_bottom = (self.CAM_H - pixel_ref_y) * scale_y
        ref_y_meters = self.Y_OFFSET_BASE + dist_ref_from_bottom
        delta_y = my_raw - ref_y_meters
        
        my_scaled = ref_y_meters + (delta_y * self.CALIB_Y_SCALE)

        offset_y_mag = abs(self.CALIB_Y_OFFSET)
        
        if my_scaled < ref_y_meters:
            my_final = my_scaled - offset_y_mag
        else:
            my_final = my_scaled + offset_y_mag

        return mx_final, my_final

    def generar_curva_limite(self):
        NX, NY = 181, 161
        self.grid_x = np.linspace(self.REAL_X_MIN, self.REAL_X_MAX, NX)
        grid_y = np.linspace(self.REAL_Y_MIN, self.REAL_Y_MAX, NY)
        self.boundary_y = np.full(NX, np.nan) 
        for i, x in enumerate(self.grid_x):
            ys_danger = []
            for y in grid_y:
                if self.check_l2_alignment_danger(x, y): ys_danger.append(y)
            if ys_danger: self.boundary_y[i] = max(ys_danger)

    def check_l2_alignment_danger(self, x, y):
        d1_sq = (self.l0 + x)**2 + y**2; d2_sq = (self.l0 - x)**2 + y**2
        if d1_sq <= 0 or d2_sq <= 0: return False
        d1 = math.sqrt(d1_sq); d2 = math.sqrt(d2_sq)
        c1 = (self.l1**2 + d1_sq - self.l2**2) / (2 * self.l1 * d1)
        c2 = (self.l1**2 + d2_sq - self.l2**2) / (2 * self.l1 * d2)
        if abs(c1) > 1.0 or abs(c2) > 1.0: return False 
        alpha1, alpha2 = math.acos(c1), math.acos(c2)
        beta1, beta2 = math.atan2(y, (self.l0 + x)), math.atan2(y, (self.l0 - x))
        sh1, sh2 = (beta1 + alpha1), (math.pi - beta2 - alpha2)
        v1x, v1y = (-self.l0 + self.l1 * math.cos(sh1)) - x, (self.l1 * math.sin(sh1)) - y
        v2x, v2y = (self.l0 + self.l1 * math.cos(sh2)) - x, (self.l1 * math.sin(sh2)) - y
        n1, n2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if n1 < 1e-6 or n2 < 1e-6: return False
        cos_phi = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (n1 * n2)))
        return math.acos(cos_phi) > self.L2_MAX_SAFE_ANGLE_AT_EF

    def es_seguro_segun_curva(self, x, y):
        idx = (np.abs(self.grid_x - x)).argmin()
        y_lim = self.boundary_y[idx]
        return True if np.isnan(y_lim) else (y > y_lim)

    def is_motor_within_limits(self, rad, min_d, max_d):
        d = math.degrees(rad)
        return (d >= min_d) and (d <= max_d)

    def check_collisions(self, q1_izq, q1_der):
        e_izq_x = -self.l0 + self.l1 * math.cos(q1_izq); e_izq_y = self.l1 * math.sin(q1_izq)
        e_der_x =  self.l0 + self.l1 * math.cos(q1_der); e_der_y = self.l1 * math.sin(q1_der)
        dist = math.sqrt((e_izq_x - e_der_x)**2 + (e_izq_y - e_der_y)**2)
        if dist < self.min_elbow_dist: return True 
        if e_izq_y < self.min_y_height or e_der_y < self.min_y_height: return True 
        if e_izq_x > e_der_x: return True
        return False

    def calcular_callback(self, msg):
        try:
            data = json.loads(msg.data)
            px = data['x']
            py = data['y']
            
            if not self.check_pixel_roi(px, py):
                self.get_logger().warn(f"Ignorado: Pix({px:.0f},{py:.0f}) fuera de ROI.")
                self.publicar_fallo(); return

            x, y = self.pixel_to_meters(px, py)

            if not self.es_seguro_segun_curva(x, y):
                self.get_logger().error(f"Peligro L2: ({x:.3f}, {y:.3f})"); self.publicar_fallo(); return

            d_izq = math.sqrt((-self.l0 - x)**2 + y**2); d_der = math.sqrt((self.l0 - x)**2 + y**2)
            limit = self.l1 + self.l2
            if d_izq > limit or d_der > limit or d_izq == 0 or d_der == 0:
                self.get_logger().error(f"Fuera alcance: ({x:.3f}, {y:.3f})"); self.publicar_fallo(); return 

            alpha_izq = math.atan2(y, x - (-self.l0))
            cos_beta_izq = (self.l1**2 + d_izq**2 - self.l2**2) / (2 * self.l1 * d_izq)
            beta_izq = math.acos(max(-1.0, min(1.0, cos_beta_izq)))
            q1_izq = alpha_izq + beta_izq 
            cos_gamma_izq = (self.l1**2 + self.l2**2 - d_izq**2) / (2 * self.l1 * self.l2)
            q2_izq = math.acos(max(-1.0, min(1.0, cos_gamma_izq))) - math.pi

            alpha_der = math.atan2(y, x - (self.l0))
            cos_beta_der = (self.l1**2 + d_der**2 - self.l2**2) / (2 * self.l1 * d_der)
            beta_der = math.acos(max(-1.0, min(1, cos_beta_der)))
            q1_der = alpha_der - beta_der 
            cos_gamma_der = (self.l1**2 + self.l2**2 - d_der**2) / (2 * self.l1 * self.l2)
            q2_der = math.pi - math.acos(max(-1.0, min(1.0, cos_gamma_der)))

            if not self.is_motor_within_limits(q1_izq, self.MOTOR1_MIN_DEG, self.MOTOR1_MAX_DEG): self.publicar_fallo(); return
            if not self.is_motor_within_limits(q1_der, self.MOTOR2_MIN_DEG, self.MOTOR2_MAX_DEG): self.publicar_fallo(); return
            if self.check_collisions(q1_izq, q1_der): self.publicar_fallo(); return 

            dist_mt = math.sqrt(x**2 + y**2)
            msg_joint = JointState()
            msg_joint.header.stamp = self.get_clock().now().to_msg()
            msg_joint.name = ['motor_izq', 'codo_izq', 'motor_der', 'codo_der']
            msg_joint.position = [
                math.degrees(q1_izq), math.degrees(q2_izq), 
                math.degrees(q1_der), math.degrees(q2_der)
            ]
            self.publisher_.publish(msg_joint)
            
            self.get_logger().info(f"OK: Pix({px:.0f},{py:.0f}) -> M({x:.3f},{y:.3f}) | Dist: {dist_mt:.3f}m")

        except Exception as e:
            self.get_logger().error(f"Error: {e}"); self.publicar_fallo()

    def publicar_fallo(self):
        msg = JointState(); msg.position = []; self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args); node = CalculoCinematica(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()