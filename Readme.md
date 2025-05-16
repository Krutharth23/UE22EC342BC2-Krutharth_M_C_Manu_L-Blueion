# Smart Indoor Delivery Assistant

A simple indoor mobile robot implementing obstacle avoidance by integrating an ESP32 running Micro-ROS with a ROS 2 host system over Wi-Fi.

## Description

The Smart Indoor Delivery Assistant demonstrates how to link a resource-constrained microcontroller to a full ROS 2 environment for basic autonomous navigation.  
- **Low-level control** on ESP32 (Micro-ROS): sensor polling, cmd_vel translation to motor PWM.  
- **High-level logic** on ROS 2 host: subscribes to ultrasonic data, applies threshold-based obstacle avoidance, publishes velocity commands.

## Features

- 🚗 Differential-drive obstacle avoidance.
- 📡 Wireless communication via Wi-Fi using Micro-ROS Agent.
- 🔄 cmd_vel translation to motor driver speeds allowing the bot to be controlled even by teleoptwistkeyboard. 
- 🔧 Modular design: easy to develop on the project.

## Working 
- The ESP32 reads the values from the three IR sensors and publishes it over the IR_val topic. 
- The IR_Processor is responsible for reading these values. by subscribing to IR_val.
- The Obstacle_avoidance then takes this values and then moves the bit accordingly by publishing over the cmd_vel topic.
- The ESP32 then subscribes to cmd_vel and moves the bot accordingly.


