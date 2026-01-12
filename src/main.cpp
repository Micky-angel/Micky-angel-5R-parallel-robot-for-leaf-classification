//final
#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h> 
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>

#define PUL1_PIN 21 
#define DIR1_PIN 20 
#define EN1_PIN  19 
#define SENSOR1_PIN 14 

#define PUL2_PIN 11
#define DIR2_PIN 12
#define EN2_PIN  13
#define SENSOR2_PIN 15 

#define SERVO_PIN 18 
#define LED_PIN 25 

#define LED_PICK_PIN 26
#define LED_LET_PIN 27


const float PASOS_POR_REV = 3200.0; 
const float RELACION_TRANSMISION = 3.12;  
const float PASOS_POR_GRADO = (PASOS_POR_REV * RELACION_TRANSMISION) / 360.0;

const float VELOCIDAD_CRUCERO_US = 400.0; 
const int VELOCIDAD_HOMING_US = 800;       
const float PORCENTAJE_RAMPA = 0.20; 
const float FACTOR_LENTITUD = 4.0;   
const float UMBRAL_MINIMO_GRADOS = 7.0;

float posActual1 = 0.0;
float posActual2 = 0.0;
float targetGlobal1 = 0.0;
float targetGlobal2 = 0.0;

Servo herramienta;

rcl_publisher_t traj_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t cmd_subscriber;

std_msgs__msg__Float32MultiArray traj_msg;
std_msgs__msg__Bool status_msg;
std_msgs__msg__String cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define CMD_BUFFER_SIZE 4096 
char cmd_buffer[CMD_BUFFER_SIZE];


void set_angles_smooth(float theta1, float theta2);
void calibrar_motores();
void pickservo(int angle);
void publish_trajectory();
void notify_completion();
void keep_alive();
void led_pick_start();
void led_pick_end();

//     CALLBACK of commands
void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  StaticJsonDocument<4096> doc; 

  DeserializationError error = deserializeJson(doc, msg->data.data);

  if (error) {
  
    for(int i=0; i<10; i++){ digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(50); }
    return;
  }

  bool homing_realizado = false;

  if (doc.is<JsonArray>()) {
    JsonArray lista = doc.as<JsonArray>();

    for (JsonObject item : lista) {
      keep_alive(); 

      const char* label = item["label"];
      
      if (strcmp(label, "2") == 0) {
        delay(3000);
        calibrar_motores(); 
        homing_realizado = true;
      }
      
      else {
        float t1 = item["angulos"][0];
        float t2 = item["angulos"][1];
               
        set_angles_smooth(180.0 - t1, t2);
        
        
        if (strcmp(label, "1") == 0) {
            delay(250);         
            pickservo(80);       
            delay(500);          
            led_pick_start();    
            delay(50);           
            pickservo(0);        
            delay(300);          
            led_pick_end();
           set_angles_smooth(30.0, 130.0); 
        } 
        else if (strcmp(label, "0") == 0) {
          delay(250);         
            pickservo(80);       
            delay(500);          
            led_pick_start();    
            delay(50);           
            pickservo(0);        
            delay(300);          
            led_pick_end();
           set_angles_smooth(130.0, 30.0); 
        }
        led_pick_end();
        // 4. Soltar
        pickservo(0); 
        delay(200); 
      }
    }
    

    if (!homing_realizado) {
      set_angles_smooth(90,90);
      calibrar_motores();
    }

    notify_completion();  
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(PUL1_PIN, OUTPUT); pinMode(DIR1_PIN, OUTPUT); pinMode(EN1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT); pinMode(EN2_PIN, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT_PULLDOWN); pinMode(SENSOR2_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PICK_PIN, OUTPUT);
  pinMode(LED_LET_PIN, OUTPUT);
  digitalWrite(LED_PICK_PIN, LOW);
  digitalWrite(LED_LET_PIN, LOW);


  herramienta.attach(SERVO_PIN);
  pickservo(0); // Pinza abierta
  digitalWrite(EN1_PIN, LOW); digitalWrite(EN2_PIN, LOW);

  delay(2000);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "parallel_robot_node", "", &support);

  rclc_publisher_init_default(&status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "robot_status");
  rclc_publisher_init_default(&traj_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "robot_traj");

  traj_msg.data.capacity = 2;
  traj_msg.data.size = 2;
  traj_msg.data.data = (float*) malloc(2 * sizeof(float));

  rclc_subscription_init_default(&cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_cmd");
  
  cmd_msg.data.capacity = CMD_BUFFER_SIZE;
  cmd_msg.data.data = cmd_buffer;
  cmd_msg.data.size = 0;

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_msg, &motor_cmd_callback, ON_NEW_DATA);
  
}

void loop() {
  keep_alive();
}


void led_pick_start() {
  digitalWrite(LED_PICK_PIN, HIGH);
  digitalWrite(LED_LET_PIN, LOW);
}

void led_pick_end() {
  digitalWrite(LED_PICK_PIN, LOW);
  digitalWrite(LED_LET_PIN, HIGH);
}

void keep_alive() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void publish_trajectory() {
  traj_msg.data.data[0] = posActual1;
  traj_msg.data.data[1] = posActual2;
  rcl_publish(&traj_publisher, &traj_msg, NULL);
}

void notify_completion() {
  status_msg.data = true;
  rcl_publish(&status_publisher, &status_msg, NULL);
  keep_alive();
  delay(5);

  status_msg.data = false;
  rcl_publish(&status_publisher, &status_msg, NULL);
  keep_alive();
}


void pickservo(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  herramienta.write(angle);
}


void calibrar_motores() {
  digitalWrite(LED_PIN, HIGH);

  auto stepPulseFast = [](uint pin) {
    gpio_put(pin, 1);
    sleep_us(2);
    gpio_put(pin, 0);
  };

  auto readFast = [](uint pin) -> bool {
    return gpio_get(pin);
  };

  gpio_init(PUL1_PIN); gpio_set_dir(PUL1_PIN, GPIO_OUT);
  gpio_init(DIR1_PIN); gpio_set_dir(DIR1_PIN, GPIO_OUT);
  gpio_init(EN1_PIN);  gpio_set_dir(EN1_PIN,  GPIO_OUT);

  gpio_init(PUL2_PIN); gpio_set_dir(PUL2_PIN, GPIO_OUT);
  gpio_init(DIR2_PIN); gpio_set_dir(DIR2_PIN, GPIO_OUT);
  gpio_init(EN2_PIN);  gpio_set_dir(EN2_PIN,  GPIO_OUT);

  gpio_init(SENSOR1_PIN); gpio_set_dir(SENSOR1_PIN, GPIO_IN);
  gpio_init(SENSOR2_PIN); gpio_set_dir(SENSOR2_PIN, GPIO_IN);

  gpio_put(DIR1_PIN, 1);   // HIGH
  gpio_put(DIR2_PIN, 1);   // HIGH

  const uint32_t d1 = (uint32_t)VELOCIDAD_HOMING_US;
  const uint32_t d2 = (uint32_t)VELOCIDAD_HOMING_US * 2u;

  absolute_time_t t1 = get_absolute_time();
  absolute_time_t t2 = t1;

  while (!readFast(SENSOR1_PIN)) {
    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, t1) <= 0) {
      stepPulseFast(PUL1_PIN);
      t1 = delayed_by_us(t1, d1);
    }
    if (absolute_time_diff_us(now, t2) <= 0) {
      stepPulseFast(PUL2_PIN);
      t2 = delayed_by_us(t2, d2);
    }
    tight_loop_contents();
  }

  posActual1 = -20.5;
  publish_trajectory();
  sleep_ms(200);

  gpio_put(DIR1_PIN, 0);   
  gpio_put(DIR2_PIN, 0);   
  const uint32_t d2b = (uint32_t)VELOCIDAD_HOMING_US;
  const uint32_t d1b = (uint32_t)((float)VELOCIDAD_HOMING_US / 1.05f);

  t1 = get_absolute_time();
  t2 = t1;

  long pasosDriftM1 = 0;

  while (!readFast(SENSOR2_PIN)) {
    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, t2) <= 0) {
      stepPulseFast(PUL2_PIN);
      t2 = delayed_by_us(t2, d2b);
    }
    if (absolute_time_diff_us(now, t1) <= 0) {
      stepPulseFast(PUL1_PIN);
      t1 = delayed_by_us(t1, d1b);
      pasosDriftM1++;
    }
    tight_loop_contents();
  }

  posActual2 = -18.5;
  float gradosDrift = (float)pasosDriftM1 / PASOS_POR_GRADO;
  posActual1 = -20.5 + gradosDrift;

  sleep_ms(100);

  set_angles_smooth(140.0, -10.0);
  delay(200);

  digitalWrite(LED_PIN, LOW);

  publish_trajectory();
}

void set_angles_smooth(float targetTheta1, float targetTheta2) {
  targetGlobal1 = targetTheta1;
  targetGlobal2 = targetTheta2;

  float delta1 = targetTheta1 - posActual1;
  float delta2 = targetTheta2 - posActual2;

  if (fabsf(delta1) <= UMBRAL_MINIMO_GRADOS && fabsf(delta2) <= UMBRAL_MINIMO_GRADOS) return;

  gpio_put(DIR1_PIN, (delta1 >= 0) ? 0 : 1);
  gpio_put(DIR2_PIN, (delta2 >= 0) ? 1 : 0);

  long pasosMeta1 = labs((long)lroundf(delta1 * PASOS_POR_GRADO));
  long pasosMeta2 = labs((long)lroundf(delta2 * PASOS_POR_GRADO));
  long maxPasos   = (pasosMeta1 > pasosMeta2) ? pasosMeta1 : pasosMeta2;
  if (maxPasos == 0) return;

  long pasosRampa = (long)lroundf((float)maxPasos * PORCENTAJE_RAMPA);
  if (pasosRampa * 2 > maxPasos) pasosRampa = maxPasos / 2;
  if (pasosRampa < 1) pasosRampa = 1;

  const float tiempoTotalCrucero = (float)maxPasos * VELOCIDAD_CRUCERO_US;

  float dBase1_f = (pasosMeta1 > 0) ? (tiempoTotalCrucero / (float)pasosMeta1) : 0.0f;
  float dBase2_f = (pasosMeta2 > 0) ? (tiempoTotalCrucero / (float)pasosMeta2) : 0.0f;

  if (pasosMeta1 > 0 && dBase1_f < 40.0f) dBase1_f = 40.0f;
  if (pasosMeta2 > 0 && dBase2_f < 40.0f) dBase2_f = 40.0f;

  auto stepPulseFast = [](uint pin) {
    gpio_put(pin, 1);
    sleep_us(2);
    gpio_put(pin, 0);
  };

  long hechos1 = 0, hechos2 = 0;

  absolute_time_t t1 = get_absolute_time();
  absolute_time_t t2 = t1;


  while (hechos1 < pasosMeta1 || hechos2 < pasosMeta2) {
    absolute_time_t now = get_absolute_time();

    long progreso = (hechos1 > hechos2) ? hechos1 : hechos2;

    float factorVelocidad = 1.0f;

    if (progreso < pasosRampa) {
      float p = (float)progreso / (float)pasosRampa;
      factorVelocidad = FACTOR_LENTITUD - (p * (FACTOR_LENTITUD - 1.0f));
    } else if (progreso > (maxPasos - pasosRampa)) {
      long falt = maxPasos - progreso;
      float p = (float)falt / (float)pasosRampa;
      factorVelocidad = FACTOR_LENTITUD - (p * (FACTOR_LENTITUD - 1.0f));
    }

    uint32_t d1 = (pasosMeta1 > 0) ? (uint32_t)lroundf(dBase1_f * factorVelocidad) : 0u;
    uint32_t d2 = (pasosMeta2 > 0) ? (uint32_t)lroundf(dBase2_f * factorVelocidad) : 0u;

    if (hechos1 < pasosMeta1 && absolute_time_diff_us(now, t1) <= 0) {
      stepPulseFast(PUL1_PIN);
      hechos1++;
      t1 = delayed_by_us(t1, d1);
    }

    if (hechos2 < pasosMeta2 && absolute_time_diff_us(now, t2) <= 0) {
      stepPulseFast(PUL2_PIN);
      hechos2++;
      t2 = delayed_by_us(t2, d2);
    }

    // uint32_t ms = to_ms_since_boot(get_absolute_time());
    // if (ms - last_pub_ms >= 50) { publish_trajectory(); last_pub_ms = ms; }

    tight_loop_contents();
  }

  posActual1 = targetTheta1;
  posActual2 = targetTheta2;


  publish_trajectory();
}


extern "C" {
  bool __atomic_test_and_set(volatile void *ptr, int memorder) {
    (void)memorder;
    volatile unsigned char *p = (volatile unsigned char *)ptr;
    noInterrupts();
    unsigned char res = *p;
    *p = 1;
    interrupts();
    return (res != 0);
  }
}