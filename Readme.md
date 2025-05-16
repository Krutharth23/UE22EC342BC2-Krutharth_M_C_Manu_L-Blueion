# Smart Indoor Delivery Assistant

A simple indoor mobile robot implementing obstacle avoidance by integrating an ESP32 running Micro-ROS with a ROS 2 host system over Wi-Fi.

## Description

The Smart Indoor Delivery Assistant demonstrates how to link a resource-constrained microcontroller to a full ROS 2 environment for basic autonomous navigation.  
- **Low-level control** on ESP32 (Micro-ROS): sensor polling, `cmd_vel` translation to motor PWM.  
- **High-level logic** on ROS 2 host: subscribes to ultrasonic data, applies threshold-based obstacle avoidance, publishes velocity commands.

## Features

- 🚗 Differential-drive obstacle avoidance  
- 📡 Wireless communication via Wi-Fi using Micro-ROS Agent  
- 🔄 Real-time `cmd_vel` translation to motor driver speeds  
- 🔧 Modular design: easy extension for path planning or multi-robot coordination

## Prerequisites

- Ubuntu 20.04 or later with ROS 2 Humble/Foxy installed :contentReference[oaicite:0]{index=0}  
- [Micro-ROS setup for ESP32](https://micro.ros.org/docs/tutorials/core/first_application_rtos/) :contentReference[oaicite:1]{index=1}  
- ESP32 development board (e.g., DevKitC)  
- Ultrasonic sensors (e.g., HC-SR04) and L298N motor driver  

## Installation

1. **Clone repository**  
   ```bash
   git clone https://github.com/Krutharth23/UE22EC342BC2-Krutharth_M_C_Manu_L-Blueion.git
   cd UE22EC342BC2-Krutharth_M_C_Manu_L-Blueion
   ``` :contentReference[oaicite:2]{index=2}  

2. **Set up Micro-ROS on ESP32**  
   ```bash
       ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
       cd firmware
       source /opt/ros/$ROS_DISTRO/setup.bash
       colcon build
   ```
   # Flash to ESP32
   idf.py flash monitor
3. **Launch Micro-ROS Agent on host**
    ```bash
        source /opt/ros/$ROS_DISTRO/setup.bash
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ```
4. **Build and run ROS 2 nodes***
    ```bash
        cd src/ir_obstacle_avoidance
        colcon build
        source install/local_setup.bash
        ros2 launch ir_obstacle_avoidance obstacle_avoidance.launch.py
    ```

