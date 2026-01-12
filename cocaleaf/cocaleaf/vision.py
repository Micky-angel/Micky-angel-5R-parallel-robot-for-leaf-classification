#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np 
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- PUBLICADORES ---
        self.publisher_ = self.create_publisher(Image, "/kinect/image_raw", 10)
        self.labels_publisher_ = self.create_publisher(String, "/vision/results", 10)

        # --- SUSCRIPTOR DE COMANDOS ---
        self.sub_cmd = self.create_subscription(
            String, 
            '/vision/cmd', 
            self.cmd_callback, 
            10
        )

        self.bridge = CvBridge()
        
        self.get_logger().info("Cargando modelo YOLO...")
        # MODELO
        self.model = YOLO('/home/fabricio/Desktop/bestcocaY11l_35.pt') 
        self.get_logger().info("Modelo cargado. Esperando comandos...")

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")

        self.captured_frame = None 
        self.current_frame = None 

        self.timer = self.create_timer(0.03, self.timer_video_feed)
        self.CONFIDENCE_THRESHOLD = 0.6 

    def obtener_centroide_preciso(self, frame_completo, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        
        h, w, _ = frame_completo.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)

        crop = frame_completo[y1:y2, x1:x2]
        if crop.size == 0: return None

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Binarización (Threshold)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None

        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)
        if M["m00"] != 0:
            cX_local = int(M["m10"] / M["m00"])
            cY_local = int(M["m01"] / M["m00"])
            
            cX_global = x1 + cX_local
            cY_global = y1 + cY_local
            
            cv2.drawContours(frame_completo[y1:y2, x1:x2], [c], -1, (0, 255, 255), 2)
            cv2.circle(frame_completo, (cX_global, cY_global), 5, (0, 0, 255), -1)
            
            return [cX_global, cY_global]
        
        return None

    def dibujar_cuadricula(self, img):
        h, w, _ = img.shape
        REAL_W, REAL_H, OFFSET_Y = 29.0, 16.0, 11.0
        px_per_cm_x = w / REAL_W
        px_per_cm_y = h / REAL_H
        center_x = w // 2

        cv2.line(img, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        for i in range(1, 4):
            d = int(i * 5 * px_per_cm_x)
            if center_x + d < w: cv2.line(img, (center_x+d, 0), (center_x+d, h), (180,180,180), 1)
            if center_x - d > 0: cv2.line(img, (center_x-d, 0), (center_x-d, h), (180,180,180), 1)

        for dist_cm in range(15, 30, 5): 
            if dist_cm > OFFSET_Y and dist_cm < (OFFSET_Y + REAL_H):
                y_px = int(h - ((dist_cm - OFFSET_Y) * px_per_cm_y))
                cv2.line(img, (0, y_px), (w, y_px), (0, 255, 0), 2)
                cv2.putText(img, f"{dist_cm}cm", (10, y_px - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def timer_video_feed(self):
        ret, frame = self.cap.read()
        if ret:
            self.current_frame = frame
            display_frame = frame.copy()
            self.dibujar_cuadricula(display_frame)
            msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
            self.publisher_.publish(msg)
            cv2.imshow("Camara Robot (Live)", display_frame)
            cv2.waitKey(1)

    def cmd_callback(self, msg):
        comando = msg.data
        if self.current_frame is None: return

        if comando == "CAPTURE":
            self.captured_frame = self.current_frame.copy()
            self.get_logger().info("Foto capturada en memoria.")
        elif comando == "DETECT":
            if self.captured_frame is not None:
                self.process_inference()
            else:
                self.get_logger().error("Faltó enviar CAPTURE antes.")

    def process_inference(self):
        self.get_logger().info(f"Iniciando inferencia YOLO + FORMA...")
        target_frame = self.captured_frame.copy() # Copia para dibujar contornos sobre ella
        
        results = self.model(self.captured_frame, conf = 0.6)
        detections_list = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf < self.CONFIDENCE_THRESHOLD: continue

                coords = box.xyxy[0].cpu().numpy().tolist()
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                
                centro_masa = self.obtener_centroide_preciso(target_frame, coords)
                x1, y1, x2, y2 = map(int, coords)
                # rectángulo Verde
                cv2.rectangle(target_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # etiqueta
                cv2.putText(target_frame, f"{cls_name} {conf:.2f}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # ---------------------------------------------
                
                # Seguridad si no existe centro de masa
                if centro_masa is None:
                    cx_box = (coords[0] + coords[2]) / 2
                    cy_box = (coords[1] + coords[3]) / 2
                    centro_masa = [cx_box, cy_box]
                    tipo_centro = "BBOX (Fallback)"
                else:
                    tipo_centro = "FORMA (Preciso)"

                detection_data = {
                    "label": cls_name,
                    "bbox": coords,
                    "conf": conf,
                    "center": centro_masa 
                }
                detections_list.append(detection_data)
                
                self.get_logger().info(f"Obj: {cls_name} | {tipo_centro} -> {centro_masa}")

        json_output = json.dumps(detections_list)
        msg = String(); msg.data = json_output
        self.labels_publisher_.publish(msg)
        
        # Mostrar resultado
        cv2.imshow("Frame", target_frame)
        cv2.waitKey(500)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()