#include <Arduino_LPS22HB.h>       // LPS22HB sensor for pressure and temperature
#include <Wire.h>
#include <RTClib.h>                // RTC for time
#include <LiquidCrystal_PCF8574.h> // LCD display (I2C)
#include <Arduino_APDS9960.h>      // RGB color sensor
#include <MFRC522.h>               // RFID library
#include <PDM.h>                   // Microphone (sound level)
#include <IRremote.hpp>            // IR communication
#include <math.h>




// LCD setup (I2C)
LiquidCrystal_PCF8574 lcd(0x27);  

// RTC setup (DS3231)
RTC_DS3231 myRTC;

// RFID setup (MFRC522)
#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN);

// Ultrasonic sensor pins (TRIG and ECHO)
const int trigPin = 2;
const int echoPin = 3;

// Rotary Encoder Pins (using pins 5,6,7)
const int encoderPinA = 5;  // CLK pin
const int encoderPinB = 6;  // DT pin
const int encoderButton = 7; // SW (button)

// IR Sensor Pin (pin 4)
const int irSensorPin = 4;  

// Buzzer Pin (pin 8)
const int buzzerPin = 8;

// PDM microphone variables
#define BUFFER_SIZE 512
int16_t sampleBuffer[BUFFER_SIZE];
volatile int samplesRead;

// Rotary Encoder Variables
volatile int encoderPos = 0;
volatile bool buttonPressed = false;

// Timing and sensor selection variables
unsigned long previousMillis = 0;
const long interval = 400;  // Refresh sensor display every 400ms
int currentSensor = 0;      // 0: RTC, 1: Pressure, 2: Temperature, 3: RGB, 4: Ultrasonic, 5: Sound Level
bool rfidDetected = false;
bool irDetected = false;

// Previous values for LCD optimization
float previousPressure = -1;
float previousTemperature = -1;
int previousR = -1, previousG = -1, previousB = -1;
float previousDistance = -1;
float previousdB = -1;
DateTime previousTime;

// ----------------- Rotary Encoder & Button ISR -----------------

// Debounced rotary encoder read (without IRAM_ATTR for Nano 33 BLE Sense Rev2)
void readEncoder() {
  static unsigned long lastEncoderChange = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderChange > 50) { // 50ms debounce
    // Compare CLK and DT states to determine rotation direction
    if (digitalRead(encoderPinA) != digitalRead(encoderPinB)) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    lastEncoderChange = currentTime;
  }
}

void handleButtonPress() {
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceTime = millis();
  if (debounceTime - lastDebounceTime > 200) {  // 200ms debounce for button
    buttonPressed = true;
  }
  lastDebounceTime = debounceTime;
}

// ----------------- Setup -----------------

void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);
  lcd.setBacklight(255);
  lcd.print("Initializing...");

  // Initialize RTC
  if (!myRTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (myRTC.lostPower()) {
    myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize BARO sensor (pressure and temperature)
  if (!BARO.begin()) {
    lcd.clear();
    lcd.print("Pressure Error");
    while (1);
  }

  // Initialize APDS sensor (RGB)
  if (!APDS.begin()) {
    lcd.clear();
    lcd.print("RGB Sensor Err");
    while (1);
  }

  // Initialize RFID
  SPI.begin();
  rfid.PCD_Init();

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up rotary encoder pins with internal pull-ups
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderButton, INPUT_PULLUP);

  // Set up IR sensor pin and buzzer pin
  pinMode(irSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Attach interrupts for rotary encoder and button
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderButton), handleButtonPress, FALLING);

  // Initialize PDM microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    lcd.clear();
    lcd.print("Mic Error");
    while (1);
  }

  delay(1000);
  lcd.clear();
}

// ----------------- Main Loop -----------------

void loop() {
  unsigned long currentMillis = millis();
  
  // Check IR sensor and update irDetected status
  checkIRSensor();
  
  // Continuously check RFID status
  checkRFID();
  
  // Handle rotary encoder changes for sensor selection
  if (encoderPos > 0) {
    encoderPos = 0;
    currentSensor = (currentSensor - 1 + 6) % 6;
    buzzerBeep();
  } else if (encoderPos < 0) {
    encoderPos = 0;
    currentSensor = (currentSensor + 1) % 6;
    buzzerBeep();
    delay(400);
  }
  
  // Show Menu if button pressed
  if (buttonPressed) {
    buttonPressed = false;
    displayMenu();
  }
  
  // If an RFID tag is detected, display it and skip the sensor cycle
  if (rfidDetected) {
    displayRFID();
    return;
  }
  
  // If no IR object is detected and time has elapsed, update the sensor display
  if (!irDetected && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    switch (currentSensor) {
      case 0: displayRTC(); break;
      case 1: displayPressure(); break;
      case 2: displayTemperature(); break;
      case 3: displayRGB(); break;
      case 4: displayUltrasonic(); break;
      case 5: displaySoundLevel(); break;
    }
  }
}


void displayRTC() {
  DateTime now = myRTC.now();
  if (now != previousTime) {
    previousTime = now;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    lcd.print(now.hour());
    lcd.print(":");
    lcd.print(now.minute());
    lcd.print(":");
    lcd.print(now.second());
    
    lcd.setCursor(0, 1);
    lcd.print("Date: ");
    lcd.print(now.day());
    lcd.print("/");
    lcd.print(now.month());
    lcd.print("/");
    lcd.print(now.year());
    lcd.setCursor(19, 3);
    lcd.print("1");
  }
}

void displayPressure() {
  float pressure = BARO.readPressure();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.print(pressure);
  lcd.print(" kPa");
  lcd.setCursor(19, 3);
  lcd.print("2");
}

void displayTemperature() {
  float temperature = BARO.readTemperature();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print((char)223);  // Degree symbol
  lcd.print("C");
  lcd.setCursor(19, 3);
  lcd.print("3");
}

void displayRGB() {
  int r, g, b;
  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b);
    
    // Convert the RGB values to a hex string (formatted as 6 digits, e.g., 000000)
    char hexColor[9];  // 8 characters for the hex code and 1 for null terminator
    sprintf(hexColor, "%02X%02X%02X", r, g, b);  // Format as 6-digit hex value
    
    // Create the full string: "hex: 000000"
    char fullHex[13];  // "hex: " + 6 digits + null terminator
    sprintf(fullHex, "hex: %s", hexColor);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("R:");
    lcd.print(r);
    lcd.print(" G:");
    lcd.print(g);
    lcd.print(" B:");
    lcd.print(b);
    
    // Calculate the starting position to center the hex code on the third line
    int hexLength = strlen(fullHex);
    int centerPos = (16 - hexLength) / 2;  // Assuming a 16-character wide LCD screen
    
    lcd.setCursor(centerPos, 2);  // Set cursor to line 3 (index 2)
    lcd.print(fullHex);
    lcd.setCursor(19, 3);
    lcd.print("4");
  }
}

void displayUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034) / 2;
  if (distance<300){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.print(" cm");
  }else{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("invalid distance!");
    lcd.setCursor(0, 1);
    lcd.print("Distance > 300 cm");
    lcd.setCursor(0, 2);
    lcd.print("(can't measure dist");
    lcd.setCursor(0,3);
    lcd.print("accurately if 300+)");
    lcd.setCursor(19, 3);
    lcd.print("5");

  }
}

void displaySoundLevel() {
  // init peak to peak value from the PDM sample buffer
  int16_t minValue = 32767;
  int16_t maxValue = -32768;
  for (int i = 0; i < samplesRead; i++) {
    if (sampleBuffer[i] < minValue) minValue = sampleBuffer[i];
    if (sampleBuffer[i] > maxValue) maxValue = sampleBuffer[i];
  }
  float peakToPeak = maxValue - minValue;
  float dB = 20 * log10(peakToPeak);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sound Level:");
  lcd.setCursor(0, 1);
  lcd.print(dB);
  lcd.print(" dB");
  lcd.setCursor(19, 3);
  lcd.print("6");
}

void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menu Mode");
  delay(1000); // diiisplay menu for 1 second before returning to sensor cycle
}



void checkRFID() {
  // Update RFID status, if new tag is present and read, mark as detected
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    rfidDetected = true;
  } else {
    rfidDetected = false;
  }
}

void displayRFID() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RFID Tag:");
  lcd.setCursor(0, 1);
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) lcd.print("0");
    lcd.print(rfid.uid.uidByte[i], HEX);
    lcd.print(" ");
  }
  delay(2000);
}

void checkIRSensor() {
  //assume IR sensor returns LOW when an object is detected
  irDetected = (digitalRead(irSensorPin) == LOW);
  if (irDetected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("IR Detected!");
    delay(500);
  }
}

void buzzerBeep() {
  digitalWrite(buzzerPin, HIGH);
  delay(200);
  digitalWrite(buzzerPin, LOW);
}


void onPDMdata() {
  samplesRead = PDM.available();
  PDM.read(sampleBuffer, samplesRead);
}
