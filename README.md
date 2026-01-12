# Micky-angel-5R-parallel-robot-for-leaf-classification

STEPS FOR CONNECTION AND RUN
on RASPI PICO:
1. Compile and run the project on an PLATFORMIO environment
2. Connect via serial comm the MCU and on terminal run the docker agent with the USB port (typically ACM0 or change if necessary):

'''docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200'''

