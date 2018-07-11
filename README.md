# Self-Balancing-Robot (Arduino UNO - C)
This code is a part of an academic work for the subject "Embedded Systems" (18.1) from UFOP (Federal University of Ouro Preto). In this code there is a LQR control implementation with a simple Kalman Filter to read the MPU-6050 sensor. For this project you need a MPU6050 sensor, an Arduino UNO (atmega-328p), two DC motors and a motor driver. To compile and upload the code to arduino you need to install the AVR tools. The makefile contains "make" and "make upload" to compile and upload the code in an easy way. 
 
## Used Arduino Pins:
* A4,A5 - MPU6050
* Digital Pins 5,6 - PWM
* Digital Pins 8,9,10,11 - Motor Driver

#### Credits:
* Jiang Yifan
* https://github.com/YifanJiangPolyU/MPU6050
  
