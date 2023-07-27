# ESP32 Fall Detection Device

<!--![ESP32 Fall Detection Device](fall_detection_device.jpg)-->

The ESP32 Fall Detection Device is a wearable device designed to detect falls using the MPU6050 accelerometer and gyroscope sensor. It utilizes the jerk in accelerations to determine if a fall event has occurred. In case of a fall, the device activates a buzzer to alert people in the vicinity. Additionally, it includes an event cancellation button that generates an interrupt to stop the buzzer. The device also features a built-in WiFi manager for easy connection to WiFi networks. In the event of a fall, it sends SOS SMS messages to the added emergency contacts to request help.


## Features

- **Fall Detection:** The device continuously monitors the user's movements using the MPU6050 sensor. A significant jerk in accelerations is detected to trigger a fall event.

- **Buzzer Alert:** Once a fall is detected, a built-in buzzer is activated to alert nearby individuals about the fall.

- **Event Cancellation Button:** The device is equipped with an event cancellation button. Pressing this button generates an interrupt, stopping the buzzer and preventing false alarms.

- **WiFi Manager:** The ESP32 Fall Detection Device has a built-in WiFi manager, allowing easy connection to WiFi networks.

- **SOS SMS:** In the event of a fall, the device can send SOS SMS messages to pre-added emergency contacts to request help.


## Hardware Components Used:

<center><img src="https://i.ibb.co/1vSSR8p/Whats-App-Image-2023-07-25-at-12-28-09-PM.jpg" height="500" width="500"></center>
<br>

- **ESP32 Development Board:** The ESP32 development board serves as the main microcontroller for the fall detection device. It provides processing power, built-in WiFi and Bluetooth capabilities, and multiple GPIO pins for interfacing with other components.

- **MPU6050 Accelerometer and Gyroscope Sensor:** The MPU6050 sensor is a crucial component for detecting falls. It combines an accelerometer and a gyroscope to measure the device's motion, allowing the detection of sudden changes in acceleration that could indicate a fall event.

- **Piezo Buzzer:** An Piezo buzzer is used to generate an audible alert in case of a fall detection. It produces a continuous sound to attract the attention of people nearby.

- **Event Cancellation Button:** The event cancellation button is a momentary push-button that allows the user to manually stop the buzzer in the event of a false alarm or after a fall detection.

- **ESP32 WiFi Connectivity:** The ESP32 Fall Detection Device is equipped with built-in WiFi capabilities. It uses this WiFi connectivity to communicate with the Twilio cloud services and send SMS alerts. With the Twilio integration, the fall detection device can benefit from a robust cloud communication platform to ensure reliable and efficient SMS delivery to the emergency contacts in case of a fall.

- **Internal Flash Memory (SPIFFS):** The ESP32 has 4MB of onboard memory which is used to store emergency contact information, wifi credentials and other settings required by the device.

The hardware components work together to enable the ESP32 Fall Detection Device to effectively detect falls, generate alerts, and send SOS messages to emergency contacts, ensuring the safety of the device user. Proper assembly, calibration, and testing are essential to achieve accurate fall detection and reliable performance.


## How it Works
<center><img src="https://i.ibb.co/q1C2q6c/Whats-App-Image-2023-07-25-at-1-08-28-PM.jpg" height="700" width="450"></center>
<br>

1. **Fall Detection:** The MPU6050 sensor measures the user's accelerations and jerk in real-time. If the jerk exceeds a certain threshold, it indicates a fall event.

2. **Buzzer Activation:** Upon fall detection, the built-in buzzer is triggered to alert people nearby.

3. **SOS SMS:** When a fall is detected, the device retrieves the emergency contact information stored in the external flash memory and sends SOS SMS messages to the pre-added emergency contacts.

4. **Event Cancellation:** The user can press the event cancellation button to stop the buzzer in case of a false alarm.

5. **WiFi Connection:** The device enters WiFi configuration mode, enabling the user to connect it to a WiFi network using a smartphone or computer.


## Getting Started

To use the ESP32 Fall Detection Device, follow these steps:

1. Assemble the hardware components according to the schematic diagram provided. (to be added)

2. Upload the provided firmware to the ESP32 using the Arduino IDE or your preferred programming environment.

3. Connect to the device's WiFi manager to configure the WiFi settings for the first time.

4. Add emergency contacts and other required settings through the WiFi manager.

5. Wear the device, and it will continuously monitor your movements.

6. In the event of a fall, the buzzer will sound, and SOS SMS messages will be sent to the emergency contacts.