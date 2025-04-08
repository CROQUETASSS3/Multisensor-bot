# Multisensor Bot

An Arduino-based robot equipped with multiple sensors for environmental awareness.

## Features
- Ultrasonic distance sensing
- barometric pressure sensing
- Infrared remote communication
- RFID tag reader
- light colour sensor
- temp and humidity% sensor
- PlatformIO + VS Code development environment

## Getting Started

### Requirements
- Arduino nano BLE3 Sense Rev1/Rev2  (or compatible board)
- Ultrasonic sensor (HC-SR04)
- IR sensor
- RTC module
- LCD with its I2C module for one wire communication
- rotary encoder
- preferably a buzzer (active, code must be modified for passive buzzers)
- PlatformIO extension for VS Code

### Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/CROQUETASSS3/Multisensor-bot.git

2. Open the folder in VS Code

3. Make sure PlatformIO is installed

4. Upload the code to your board using PlatformIO

### WIRING
explained for begginers:

HC-SRO4 (Ultrasonic):
Trig - D2
Echo - D3
Vcc - 5v
Gnd - ground

IR sensor module:
R (sensor pin) - D4
G - ground
Y - 5v

Rotary encoder:
CLK - D5
DT - D6
SW (button pin) - D7
+ - 5v
GND - ground

buzzer (if using):
short side (-) - ground
long side (+) - through a transistor controlled by pin 8 to vcc
(buzzer uses too much voltage and current which the nano can't provide, therefore we use a transistor connected to 5v for a steady power supply)

RFID (RC522):
SDA (SS) - D10
RST - D9
SCK - D13
MOS - D11
MISO - D12
GND - ground
3.3v -3.3v (IMPORTANT, only component thet uses 3.3v, any more will fry it)

I2C pins:
(can change depending on arduino model)
RTC and LCD:
SDA- A5
SCL - A4


### LICENSE
MIT – you are free to use, modify, and distribute this project.


