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

## Wiring

### HC-SR04 (Ultrasonic Sensor)
- **Trig** - D2
- **Echo** - D3
- **Vcc** - 5V
- **GND** - Ground

### IR Sensor Module
- **R (sensor pin)** - D4
- **GND** - Ground
- **Y (5V)** - 5V

### Rotary Encoder
- **CLK** - D5
- **DT** - D6
- **SW (button pin)** - D7
- **+** - 5V
- **GND** - Ground

### Buzzer (if using)
- **Short side (-)** - Ground
- **Long side (+)** - Through a transistor controlled by pin 8 to 5V

*Note: The buzzer uses too much voltage and current for the Arduino Nano, so we use a transistor connected to 5V for a steady power supply.*

### RFID (RC522)
- **SDA (SS)** - D10
- **RST** - D9
- **SCK** - D13
- **MOSI** - D11
- **MISO** - D12
- **GND** - Ground
- **3.3V** - 3.3V (Important: This is the only component that uses 3.3V. Using more will damage the module.)

### I2C Pins (RTC and LCD)
*(These can change depending on your Arduino model.)*
- **SDA** - A5
- **SCL** - A4



## LICENSE
MIT – you are free to use, modify, and distribute this project.


