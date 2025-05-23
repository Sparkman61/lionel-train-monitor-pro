/*
 * Lionel Train Monitor v5 (Final Version)
 * For Arduino Nano 33 BLE
 * Complete monitoring system with lap tracking
 */

// Libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoBLE.h>

// Pin definitions
#define HALL_SENSOR_PIN     2   // Digital pin for hall effect sensor
#define IR_SENSOR_PIN       A6  // Analog pin for TCRT5000L sensor (moved from D5 to avoid I2C interference)
#define VOLTAGE_SENSOR_PIN  A0  // Analog pin for voltage sensor
#define CURRENT_SENSOR_PIN  A1  // Analog pin for current sensor
#define TEMP_SENSOR_PIN     A2  // Analog pin for temperature sensor
#define SD_CS_PIN           10  // SPI CS pin for SD card

// Constants
#define VOLTAGE_FACTOR      6.61  // Calibrated value
#define WHEEL_CIRCUMFERENCE 1.0   // Wheel circumference in inches
#define TEMP_NOMINAL        25    // Nominal temperature for thermistor
#define TEMP_R_NOMINAL      5000  // Nominal resistance of thermistor at TEMP_NOMINAL
#define TEMP_B_COEFFICIENT  3950  // Beta coefficient of the thermistor
#define TEMP_SERIES_RESISTOR 10000 // Value of series resistor for thermistor

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Bluetooth service and characteristics
BLEService trainService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic voltageCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic currentCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic powerCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic speedCharacteristic("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic distanceCharacteristic("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic tempCharacteristic("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// Global variables
float voltage = 0.0;
float current = 0.0;
float power = 0.0;
float temperature = 0.0;
float temperatureF = 0.0;
unsigned long previousMillis = 0;
unsigned long lastLogMillis = 0;

// Speed and distance variables
volatile unsigned long revolutions = 0;
volatile unsigned long lastRevTime = 0;
volatile boolean startPositionDetected = false;
volatile unsigned int lapCount = 0;
float speedInchPerSec = 0.0;
float speedFtPerSec = 0.0;
float distanceInches = 0.0;
float distanceFeet = 0.0;

// Display mode variable
int displayMode = 0;
unsigned long displayToggleTime = 0;

// SD card variables
bool sdCardAvailable = false;
String logFileName = "TRAIN.CSV";
File logFile;

// Interrupt handler for hall effect sensor
void wheelRevolution() {
  // Debounce
  if ((millis() - lastRevTime) > 10) {
    revolutions++;
    
    // Calculate speed based on time between revolutions
    if (lastRevTime > 0) {
      unsigned long timeDiff = millis() - lastRevTime;
      speedInchPerSec = (WHEEL_CIRCUMFERENCE * 1000.0) / timeDiff;
      speedFtPerSec = speedInchPerSec / 12.0;
    }
    
    lastRevTime = millis();
    
    // Update distance in different units
    distanceInches = revolutions * WHEEL_CIRCUMFERENCE;
    distanceFeet = distanceInches / 12.0;
  }
}

// Check IR sensor for lap detection (polling method)
void checkIRSensor() {
  static unsigned long lastDetectionTime = 0;
  static boolean lastSensorState = false; // false = not detecting, true = detecting
  static int stableCount = 0;
  
  // Read analog value and convert to detection state
  int analogValue = analogRead(IR_SENSOR_PIN);
  // Detection occurs when voltage is very low (covered sensor reads ~7mV = analog value ~2-5)
  // Normal state is ~116mV = analog value ~35-40
  boolean currentState = (analogValue < 20); // Threshold: 20 = ~65mV (between 7mV and 116mV)
  
  // Require stable readings for debouncing
  if (currentState == lastSensorState) {
    stableCount++;
  } else {
    stableCount = 0;
    lastSensorState = currentState;
  }
  
  // Only trigger on stable detection after 5 consistent readings (more stable)
  if (stableCount >= 5 && currentState && (millis() - lastDetectionTime > 2000)) { // 2 second minimum between laps
    startPositionDetected = true;
    lapCount++;
    lastDetectionTime = millis();
    
    Serial.print("Lap ");
    Serial.print(lapCount);
    Serial.print(" completed! Analog value: ");
    Serial.println(analogValue);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Lionel Train Monitor v5 (Final) Starting...");
  
  // Initialize pins
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  // Note: IR sensor on A6 will use analog reading (A6 is analog-only)
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Train Monitor v5");
    display.println("Final Version");
    display.println("Starting up...");
    display.display();
    Serial.println("OLED initialized");
  }
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SD Card Init...");
  display.display();
  
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(100);
  
  if (SD.begin(SD_CS_PIN)) {
    sdCardAvailable = true;
    Serial.println(" SD card initialized successfully.");
    
    // Create log file with headers
    if (SD.exists(logFileName)) {
      SD.remove(logFileName);
    }
    
    logFile = SD.open(logFileName, FILE_WRITE);
    if (logFile) {
      logFile.println("Time(s),Voltage(V),Current(A),Power(W),Speed(in/s),Distance(in),Temp(C),Temp(F),Lap,Revs");
      logFile.close();
      Serial.println("Log file created: " + logFileName);
    }
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD Card: OK");
    display.display();
    delay(1000);
  } else {
    sdCardAvailable = false;
    Serial.println(" SD card initialization failed!");
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD Card: FAILED");
    display.println("Check connections");
    display.display();
    delay(2000);
  }
  
  // Initialize Bluetooth
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
  } else {
    BLE.setLocalName("TrainMonitor");
    BLE.setAdvertisedService(trainService);
    
    trainService.addCharacteristic(voltageCharacteristic);
    trainService.addCharacteristic(currentCharacteristic);
    trainService.addCharacteristic(powerCharacteristic);
    trainService.addCharacteristic(speedCharacteristic);
    trainService.addCharacteristic(distanceCharacteristic);
    trainService.addCharacteristic(tempCharacteristic);
    
    BLE.addService(trainService);
    
    voltageCharacteristic.writeValue(0.0);
    currentCharacteristic.writeValue(0.0);
    powerCharacteristic.writeValue(0.0);
    speedCharacteristic.writeValue(0.0);
    distanceCharacteristic.writeValue(0.0);
    tempCharacteristic.writeValue(0.0);
    
    BLE.advertise();
    Serial.println("BLE initialized. Broadcasting as 'TrainMonitor'");
  }
  
  // Attach interrupts for sensors
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), wheelRevolution, FALLING);
  // Note: IR sensor on A6 uses polling (A6 is analog-only, no interrupts)
  
  // Initialize variables
  revolutions = 0;
  lapCount = 0;
  distanceInches = 0;
  distanceFeet = 0;
  speedInchPerSec = 0;
  speedFtPerSec = 0;
  
  Serial.println("Setup complete - All systems ready!");
}

void loop() {
  // Check IR sensor for lap detection (polling method)
  checkIRSensor();
  
  // Check if start position has been detected (lap completed)
  if (startPositionDetected) {
    Serial.print("Lap ");
    Serial.print(lapCount);
    Serial.println(" - resetting distance counter");
    revolutions = 0;
    distanceInches = 0;
    distanceFeet = 0;
    startPositionDetected = false;
  }
  
  // Read sensors every 100ms
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    
    // Read voltage
    voltage = readVoltage();
    
    // Read current
    current = readCurrent();
    
    // Read temperature
    temperature = readTemperature();
    
    // Calculate power
    power = voltage * current;
    
    // Check for warnings
    if (temperature > 60.0) {
      Serial.println("WARNING: High temperature detected!");
    }
    
    // Update BLE characteristics if connected
    if (BLE.connected()) {
      voltageCharacteristic.writeValue(voltage);
      currentCharacteristic.writeValue(current);
      powerCharacteristic.writeValue(power);
      speedCharacteristic.writeValue(speedInchPerSec);
      distanceCharacteristic.writeValue(distanceInches);
      tempCharacteristic.writeValue(temperature);
    }
    
    // Log data to SD card (every second)
    if (currentMillis - lastLogMillis >= 1000) {
      lastLogMillis = currentMillis;
      logData();
    }
    
    // Update display
    updateDisplay();
  }
  
  // Poll BLE for new clients
  BLE.poll();
}

// Read voltage
float readVoltage() {
  int rawValue = analogRead(VOLTAGE_SENSOR_PIN);
  float voltageRaw = (rawValue / 1023.0) * 3.3;
  float scaledVoltage = voltageRaw * VOLTAGE_FACTOR;
  
  return scaledVoltage;
}

// Read current
float readCurrent() {
  int rawValue = analogRead(CURRENT_SENSOR_PIN);
  float voltageRaw = (rawValue / 1023.0) * 3.3;
  float current = (voltageRaw - 1.65) * 0.1;
  
  return current;
}

// Read temperature from NTC thermistor
float readTemperature() {
  const int numSamples = 5;
  float samplesSum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int rawValue = analogRead(TEMP_SENSOR_PIN);
    samplesSum += rawValue;
    delay(2);
  }
  
  int rawValue = samplesSum / numSamples;
  
  // Convert the analog reading to resistance
  float resistance = TEMP_SERIES_RESISTOR / ((1023.0 / rawValue) - 1.0);
  
  // Sanity check on resistance
  if (resistance < 100 || resistance > 50000) {
    return 25.0;
  }
  
  // Calculate temperature using the Beta parameter equation
  float steinhart;
  steinhart = resistance / TEMP_R_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= TEMP_B_COEFFICIENT;
  steinhart += 1.0 / (TEMP_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  
  // Apply simple filtering
  static float lastTemp = 25.0;
  if (abs(steinhart - lastTemp) > 20) {
    steinhart = lastTemp;
  } else {
    lastTemp = steinhart;
  }
  
  // Calculate Fahrenheit equivalent
  temperatureF = steinhart * 9.0/5.0 + 32.0;
  
  return steinhart;
}

// Log data to SD card
void logData() {
  if (!sdCardAvailable) return;
  
  logFile = SD.open(logFileName, FILE_WRITE);
  if (logFile) {
    // Create a more useful timestamp (hours:minutes:seconds since startup)
    unsigned long totalSeconds = millis() / 1000;
    unsigned long hours = totalSeconds / 3600;
    unsigned long minutes = (totalSeconds % 3600) / 60;
    unsigned long seconds = totalSeconds % 60;
    
    // Format: HH:MM:SS,Voltage,Current,Power,Speed,Distance,TempC,TempF,Lap,Revolutions
    char timeStr[10];
    sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes, seconds);
    
    logFile.print(timeStr);
    logFile.print(",");
    logFile.print(voltage, 2);
    logFile.print(",");
    logFile.print(current, 3);
    logFile.print(",");
    logFile.print(power, 2);
    logFile.print(",");
    logFile.print(speedInchPerSec, 2);
    logFile.print(",");
    logFile.print(distanceInches, 1);
    logFile.print(",");
    logFile.print(temperature, 1);
    logFile.print(",");
    logFile.print(temperatureF, 1);
    logFile.print(",");
    logFile.print(lapCount);
    logFile.print(",");
    logFile.println(revolutions);
    
    logFile.close();
  }
}

// Update OLED display
void updateDisplay() {
  // Toggle display mode every 5 seconds
  if (millis() - displayToggleTime > 5000) {
    displayToggleTime = millis();
    displayMode = (displayMode + 1) % 4;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  if (displayMode == 0) {
    // Electrical readings
    display.println("--- ELECTRICAL ---");
    display.print("Voltage: ");
    display.print(voltage, 1);
    display.println(" V");
    
    display.print("Current: ");
    display.print(current, 2);
    display.println(" A");
    
    display.print("Power: ");
    display.print(power, 1);
    display.println(" W");
    
    display.print("BLE: ");
    display.println(BLE.connected() ? "Connected" : "Waiting...");
  } else if (displayMode == 1) {
    // Motion data with lap count
    display.println("--- MOTION ---");
    
    display.print("Lap: ");
    display.println(lapCount);
    
    int totalInches = (int)distanceInches;
    int feet = totalInches / 12;
    int inches = totalInches % 12;
    
    display.print("Dist: ");
    if (feet > 0) {
      display.print(feet);
      display.print("ft ");
    }
    display.print(inches);
    display.println("in");
    
    display.print("Speed: ");
    display.print(speedInchPerSec, 1);
    display.println(" in/s");
    
    display.print("Revs: ");
    display.println(revolutions);
  } else if (displayMode == 2) {
    // Temperature and thermal info
    display.println("--- THERMAL ---");
    
    display.print("Temp: ");
    display.print(temperature, 1);
    display.print("C / ");
    display.print(temperatureF, 1);
    display.println("F");
    
    if (temperature > 60.0) {
      display.println("Status: HOT!");
    } else if (temperature > 40.0) {
      display.println("Status: Warm");
    } else {
      display.println("Status: Normal");
    }
    
    int tempRaw = analogRead(TEMP_SENSOR_PIN);
    display.print("ADC: ");
    display.println(tempRaw);
  } else {
    // System info
    display.println("--- SYSTEM ---");
    
    display.print("BLE: ");
    display.println(BLE.connected() ? "Connected" : "Waiting...");
    
    display.print("SD: ");
    display.println(sdCardAvailable ? "Logging" : "Failed");
    
    display.print("Uptime: ");
    display.print(millis() / 60000);
    display.println(" min");
    
    display.print("File: ");
    display.println(logFileName);
  }
  
  display.display();
}
