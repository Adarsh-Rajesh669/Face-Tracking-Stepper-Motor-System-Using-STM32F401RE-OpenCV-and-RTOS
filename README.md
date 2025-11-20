#  Face-Tracking Stepper Motor System Using STM32F401RE, OpenCV and RTOS

This project demonstrates a real-time face-tracking system by integrating computer vision and embedded control. A webcam detects the position of a human face using OpenCV and the Haar cascade algorithm. Based on the face’s position, the system sends direction commands to an STM32F401RE microcontroller over UART, which in turn rotates a stepper motor to track the face movement and it also measures humidity and pressure using dht111 sensor.

## 🔧 System Overview

- **OpenCV (PC side)**:
  - Captures video from webcam
  - Detects face using Haar Cascade algorithm
  - Determines face position: LEFT, RIGHT, or CENTER
  - Sends position command to STM32 over UART

- **STM32F401RE (Microcontroller side)**:
  - Receives serial commands (LEFT / RIGHT / CENTER)
  - Controls a stepper motor via GPIO
  - Rotates motor to align with detected face position

---

## 🛠️ Hardware Requirements

- STM32F401RE Nucleo Board  
- Stepper Motor (28BYJ-48)  
- Stepper Motor Driver (ULN2003)
- DHT111 Sensor
- USB cable for UART communication  
---



