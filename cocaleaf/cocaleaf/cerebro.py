#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import json
from enum import Enum
import time

#  0 - is BAD leaf
#  1 - is GOOD leaf
#  2 - is GO TO HOME leaf

class Estado(Enum):
    INICIO = 0
    YENDO_HOME = 1             
    CAPTURANDO = 2              
    SOLICITANDO_PROCESAMIENTO = 3 
    ESPERANDO_RESULTADOS_VIS = 4
    PROCESANDO_CINEMATICA = 5
    MOVIENDO_ROBOT = 6         

class CerebroNode(Node):
    def __init__(self):
        super().__init__('cerebro_node')

        # --- SUSCRIPTORES ---
        self.sub_vision = self.create_subscription(String, '/vision/results', self.callback_vision_result, 10)
        self.sub_cinematica = self.create_subscription(JointState, '/degrees', self.callback_cinematica_result, 10)
        self.sub_robot_status = self.create_subscription(Bool, '/robot_status', self.callback_robot_status, 10)

        # --- PUBLICADORES ---
        self.pub_vision_cmd = self.create_publisher(String, '/vision/cmd', 10)
        self.pub_cinematica_req = self.create_publisher(String, '/cinematica/request', 10)
        self.pub_robot_cmd = self.create_publisher(String, '/motor_cmd', 10)

        # --- VARIABLES ---
        self.estado = Estado.INICIO
        
        self.lista_bboxes_pendientes = []
        self.lista_final_acciones = []
        self.label_temporal = ''
        
        self.esperando_respuesta_cinematica = False
        self.esperando_robot_libre = False
        self.orden_enviada = False 
        self.tiempo_foto = 0

        self.timer = self.create_timer(0.1, self.loop_control)
        self.get_logger().info("CEREBRO LISTO.")

    def callback_vision_result(self, msg):
        if self.estado == Estado.ESPERANDO_RESULTADOS_VIS:
            objetos = json.loads(msg.data)
            if objetos:
                self.lista_bboxes_pendientes = objetos
                self.lista_final_acciones = []
                self.get_logger().info(f"Visión detectó {len(objetos)} objetos.")
                self.estado = Estado.PROCESANDO_CINEMATICA
                self.esperando_respuesta_cinematica = False
            else:
                self.get_logger().warn("Nada detectado. Reiniciando.")
                self.reiniciar_ciclo()

    def callback_cinematica_result(self, msg):
        if self.estado == Estado.PROCESANDO_CINEMATICA:
            if len(msg.position) == 0:
                self.esperando_respuesta_cinematica = False
                return 
            
            angulos = list(msg.position)
            
            motores_activos = [angulos[0], angulos[2]]
            self.get_logger().info(f"angulo calculado. {motores_activos}")
            accion = {"label": self.label_temporal, "angulos": motores_activos} 
            self.lista_final_acciones.append(accion)
            
            self.esperando_respuesta_cinematica = False

    def callback_robot_status(self, msg):
        if msg.data is True and self.esperando_robot_libre:
            self.get_logger().info("Robot confirmó tarea finalizada.")
            self.esperando_robot_libre = False

    # Loop principal 
    def loop_control(self):
        
        if self.estado == Estado.INICIO:
            self.estado = Estado.YENDO_HOME
            self.orden_enviada = False

        # yendo a home
        elif self.estado == Estado.YENDO_HOME:
            if not self.orden_enviada:
                self.enviar_batch([{"label": "2", "angulos": []}]) 
                self.get_logger().info("Yendo a Home (Inicio)...")
                self.esperando_robot_libre = True
                self.orden_enviada = True 
            else:
                if not self.esperando_robot_libre:
                    self.estado = Estado.CAPTURANDO
                    self.orden_enviada = False

        elif self.estado == Estado.CAPTURANDO:
            if not self.orden_enviada:
                self.enviar_vision("CAPTURE")
                self.get_logger().info("Tomando Foto...")
                self.orden_enviada = True
                self.tiempo_foto = time.time()
            else:
                if time.time() - self.tiempo_foto > 0.5:
                    self.estado = Estado.SOLICITANDO_PROCESAMIENTO
                    self.orden_enviada = False 

        elif self.estado == Estado.SOLICITANDO_PROCESAMIENTO:
            if not self.orden_enviada:
                self.enviar_vision("DETECT") 
                self.get_logger().info("Solicitando detección...")
                self.orden_enviada = True
                self.estado = Estado.ESPERANDO_RESULTADOS_VIS

        elif self.estado == Estado.ESPERANDO_RESULTADOS_VIS:
            pass

        elif self.estado == Estado.PROCESANDO_CINEMATICA:
            if not self.esperando_respuesta_cinematica:
                if len(self.lista_bboxes_pendientes) > 0:
                    obj = self.lista_bboxes_pendientes.pop(0)
                    if 'center' in obj: 
                        cx = obj['center'][0]
                        cy = obj['center'][1]
                    else:
                        cx = (obj['bbox'][0] + obj['bbox'][2]) / 2
                        cy = (obj['bbox'][1] + obj['bbox'][3]) / 2
                    if obj['label'] == 'bad':
                        self.label_temporal = '0'
                    else:
                        self.label_temporal = '1'
                    
                    req = {"x": cx, "y": cy}
                    msg = String(); msg.data = json.dumps(req)
                    self.esperando_respuesta_cinematica = True
                    self.pub_cinematica_req.publish(msg)
                else:
                    self.estado = Estado.MOVIENDO_ROBOT
                    self.get_logger().info("Cálculos terminados. Iniciando transmisión masiva...")
                    self.esperando_robot_libre = False
                    self.orden_enviada = False

        elif self.estado == Estado.MOVIENDO_ROBOT:
            
            if not self.orden_enviada:
                if len(self.lista_final_acciones) > 0:
                    self.get_logger().info(f"Enviando lote de {len(self.lista_final_acciones)} acciones...")
                    self.enviar_batch(self.lista_final_acciones)
                    
                    self.esperando_robot_libre = True
                    self.orden_enviada = True
                else:
                    self.get_logger().warn("Lista vacía. Reiniciando.")
                    self.reiniciar_ciclo()
            else:
                if not self.esperando_robot_libre:
                    self.get_logger().info("Ciclo Completado.")
                    self.reiniciar_ciclo()

    def enviar_vision(self, cmd):
        msg = String(); msg.data = cmd; self.pub_vision_cmd.publish(msg)

    def enviar_batch(self, lista_acciones):
        msg = String()
        msg.data = json.dumps(lista_acciones)
        self.pub_robot_cmd.publish(msg)

    def reiniciar_ciclo(self):
        self.get_logger().info(" >>> Reiniciando ciclo<<<")
        self.orden_enviada = False 
        self.lista_final_acciones = []
        self.estado = Estado.CAPTURANDO

def main(args=None):
    rclpy.init(args=args)
    node = CerebroNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()