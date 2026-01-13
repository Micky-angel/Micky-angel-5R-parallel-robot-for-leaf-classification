## Micky-angel-5R-parallel-robot-for-leaf-classification

STEPS FOR CONNECTION AND RUN
# 1. on RASPI PICO:
1. Compile and run the project on an PLATFORMIO environment
2. Connect via serial comm the MCU and on terminal run the docker agent with the USB port (typically ACM0 or change if necessary):

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
```
# 2. on Laptop or Raspberry pi 5
1. Clone the package, cocaleaf on your local src directory
2. Run the next commands on terminal.

```bash
colcon build --symlink-install
source install/setup.bash 
ros2 launch cocaleaf robot.launch.py
```
#  3. CAD Design.
```bash
https://drive.google.com/drive/folders/1_vDUbXiMIU5-rc2W7qZbikIN8O1vwofr?usp=sharing
```
