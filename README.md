# Driver Attention Monitoring System – STM32F401

## Overview
This is an embedded Driver Attention Monitoring System built on the STM32F401 microcontroller. It is a non-camera, privacy-friendly solution that monitors driver alertness using biometric sensors and provides real-time alerts to improve road safety.

## Demo Video
You can watch the working demo of this project here:  
**YouTube:** https://youtu.be/qQDaqbbR_SY

## Features
- Detects driver drowsiness and inattention using eye-blink and pulse data. 
- Provides audio alerts using a buzzer and visual alerts using LEDs and an LCD/TFT display.   
- Implements threshold-based logic on STM32F401 without any machine learning. 
- Designed to be low-cost, low-power, and suitable for various vehicles.  
- Privacy-friendly since it does not use any camera-based monitoring.
  
## Hardware Components
- STM32F401 MCU (ARM Cortex-M4, 84 MHz) – reads sensors and runs decision logic.   
- IR eye-blink sensor – detects eye open/close state for drowsiness.   
- Pulse sensor (e.g., MAX30102) – measures heart rate and variability related to fatigue.   
- Buzzer – provides audio alerts when drowsiness is detected.   
- LEDs – indicate system and alert status.   
- LCD/TFT display – shows driver status such as “Active”, “Drowsy”, etc.   
- Power supply stage – regulated 3.3 V from vehicle battery (via 5 V regulator if required).  

> Note: Accelerometer/gyroscope (e.g., MPU6050) was part of the design but is not used in the current implementation.

## System Working
1. **Sensor Input**  
   - IR eye-blink sensor checks if the eyes remain closed beyond a set time.  
   - Pulse sensor monitors heart rate irregularities indicating fatigue.  

2. **Processing on STM32F401**  
   - STM32F401 reads sensor values via GPIO/ADC/I2C.  
   - Threshold-based logic decides whether the driver is active or drowsy. 

3. **Alerts and Output**  
   - If drowsiness is detected, buzzer and LEDs are activated.  
   - LCD/TFT displays the current driver status in real time.    



