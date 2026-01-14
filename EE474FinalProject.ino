//========================================= Includes =============================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <LiquidCrystal_I2C.h>
#include <stdio.h>

#include "DHT.h" // Adafruit DHT library (as well as Adafruit Unified Sensor library) are required
#include <ESP32Servo.h> // ESP32Servo by Kevin Harrington is required

//========================================= Macros =============================================
// Input pins - Sensors
#define TEMP 7
#define HUMIDITY 3
#define WATER_LVL 2
#define LDR 5

// Input pins - User input
#define BUTTON 1
#define POT 6

// Output pins
#define LED_BLINKING 46
#define LIGHT_SOURCE 17
#define MOTOR 12

// For LCD helper functions
#define LCD_WRITE 0b00001101

//========================================= Typedefs =============================================
typedef enum {PLANT1, PLANT2, PLANT3} plant;

// From Lab 4
// Use addVal function to add values and set moving average.
// FIFO data structure. Automatically removes oldest values as new values are added, and
// automatically tracks moving average.
typedef struct valueContainer {
  int values[5];
  int movingAvg;
  int prevAvg;
} valueContainer;

typedef struct plantConfig {
  int temperature;
  int moisture;
  int lightLevel;
} plantConfig;


//==================================== Global Variables ========================================
// Queues
QueueHandle_t tempQueue;
QueueHandle_t humidityQueue;
QueueHandle_t waterLevelQueue;
QueueHandle_t lightLevelQueue;
QueueHandle_t frequencyQueue;

// Task handles
TaskHandle_t tempHandle;
TaskHandle_t humidityHandle;
TaskHandle_t waterHandle;
TaskHandle_t LDRHandle;

TaskHandle_t buttonHandle;
TaskHandle_t potentiometerHandle;
TaskHandle_t BLEInHandle;

TaskHandle_t lightSourceHandle;
TaskHandle_t plantWatererHandle;
TaskHandle_t LCDHandle;

// Semaphores and protected variables
SemaphoreHandle_t tempSemaphore;
volatile valueContainer tempVals;

SemaphoreHandle_t lightSemaphore;
volatile valueContainer lightVals;

// Normal global variables
volatile unsigned long runtime;

volatile plant myPlant;

SemaphoreHandle_t configSemaphore;
volatile plantConfig settings;

// Peripherals
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Using two sensors to read temp and humidity in parallel, as each sensor has a delay of 1 or 2s.
DHT tempSensor (TEMP, DHT22); // Change to DHT11 later?
DHT humiditySensor (HUMIDITY, DHT11);
Servo actuator;

//=================================== Sensor Data Collection =====================================

// Records temperature
// IF USING DHT22 - NO MORE THAN 0.5Hz SAMPLING RATE (otherwise, if DHT11, 1Hz is good).
// Also, uses BLE to send a low/high temperature warning.
void tempTask(void* pvParameters) { // Core 0
  // Sends to temperature queue.
  // Also takes a semaphore and sets temperature global variable.
  // tempSensor.readTemperature(true); // Reads temp (ºF)

  int currTemp;
  while(1) {
    currTemp = tempSensor.readTemperature(true);
    xQueueSend(tempQueue, &currTemp, portMAX_DELAY);

    if(xSemaphoreTake(tempSemaphore, pdMS_TO_TICKS(2)) == pdTRUE) {
      addVal(currTemp, &tempVals);
      xSemaphoreGive(tempSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}


// Records humidity
// 1Hz sampling rate (DHT11)
void humidityTask(void* pvParameters) { // Core 0
  // Sends to humidity queue.
  // humiditySensor.readHumidity(); // Gives relative humidity as a percentage.
  int currHumidity;
  while(1) {
    currHumidity = humiditySensor.readHumidity();
    xQueueSend(humidityQueue, &currHumidity, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


// Records water level
// Also, uses BLE to send a low water level warning.
// Low frequency (0.25Hz, probably?)
void waterTask(void* pvParameters) { // Core 0
  // Sends to water level queue
  int currWater;
  while(1) {
    currWater = analogRead(WATER_LVL);
    xQueueSend(waterLevelQueue, &currWater, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


// Records light level
// This one can be extremely fast, if need be. Maybe 32Hz.
void LDRTask(void* pvParameters) { // Core 0
  // Sends to light level queue
  // Also takes a semaphore and sets light level global variable.
  // Reads from the frequency queue
  int currLight
  while(1) {
    
  }
}


//=================================== User Input Handling =====================================

// Takes input from the button. Based on input, suspends/resumes tasks.
// This one can be decently fast. About 20Hz, possibly.
void buttonTask(void* pvParameters) { // Core 1
  // I realized from ICTE7 that this MUST be a task, and not an interrupt.
  while(1) {
    
  }
}


// Takes input from the potentiometer to change the frequency of LDR task.
// Should be very fast, about 50Hz.
void potentiometerTask(void* pvParameters) { // Core 0
  // Sends to frequency queue
  while(1) {
    
  }
}


// Takes in user input using BLE.
// User input includes presets for temperature and moisture thresholds.
// Moderate speed, about 4Hz.
void BLEInTask(void* pvParameters) { // Core 0
 // Takes binary semaphore protecting presets
 while(1) {
    
  }
}


//=================================== Using Sensor Data =====================================

// Based on the ambient light level, uses PWM to change the brightness of the light source.
// Should have the same frequency as the LDR task (which is determined by potentiometer task).
void lightSourceTask(void* pvParameters) { // Core 1
  // Receives from light level queue
  while(1) {
    
  }
}


// Uses humidity/moisture data to open/close the valve with the servo motor while also blinking
// an LED (whenever the valve is opened).
// Motor used is the SG90 Servo Motor.
// Same frequency as humidity task, 1Hz.
void plantWatererTask(void* pvParameters) { // Core 1
  // Receives from humidity queue
  while(1) {
    
  }
}


// Prints the temperature, light level and total runtime to the LCD via I2C (printStr() and
// setupLCD() functions, which I originally made for Lab 3).
// Frequency of about 5Hz.
void LCDTask(void* pvParameters) { // Core 1
  // Waits for binary semaphores protecting temperature, light level and runtime.
  // Prints only when at least one of the values change.
  while(1) {
    
  }
}

//=================================== Setup and Loop =====================================
void setup() {
  
  pinMode(TEMP, INPUT_PULLUP);
  pinMode(HUMIDITY, INPUT_PULLUP);
  pinMode(WATER_LVL, INPUT_PULLUP);
  pinMode(LDR, INPUT_PULLUP);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(POT, INPUT_PULLUP); // Needs ADC

  pinMode(LED_BLINKING, OUTPUT);
  pinMode(LIGHT_SOURCE, OUTPUT); // Uses PWM
  pinMode(MOTOR, OUTPUT);

  tempSensor.begin(); // Needs ADC-capable pin
  humiditySensor.begin(); // Needs ADC-capable pin
  lcd.init();
  setupLCD();
  actuator.attach(MOTOR, 500, 2500);
  actuator.setPeriodHertz(50);

  for(int i = 0; i < 5; i++) {
    tempVals.values[i] = 0;
    lightVals.values[i] = 0;
  }

  tempVals.movingAvg = 0;
  lightVals.movingAvg = 0;

  tempVals.prevAvg = -1;
  lightVals.prevAvg = -1;

  myPlant = PLANT1; // Default
  plantConfigurator(&settings);

  tempQueue = xQueueCreate(4, sizeof(int));
  humidityQueue = xQueueCreate(4, sizeof(int));
  waterLevelQueue = xQueueCreate(4, sizeof(int));
  lightLevelQueue = xQueueCreate(4, sizeof(int));
  frequencyQueue = xQueueCreate(4, sizeof(int));

  if(tempQueue == NULL || humidityQueue == NULL || waterLevelQueue == NULL
      || lightLevelQueue == NULL || frequencyQueue == NULL) {
    Serial.print("Queue didn't create!");
    while(1);
  }


  // Data collection tasks
  xTaskCreatePinnedToCore(
    tempTask,
    "Temperature Data Collection",
    2048,
    NULL,
    2,
    &humidityHandle,
    0
  );

  xTaskCreatePinnedToCore(
    humidityTask,
    "Humidity Data Collection",
    2048,
    NULL,
    3,
    &humidityHandle,
    0
  );

  xTaskCreatePinnedToCore(
    waterTask,
    "Water Level Data Collection",
    2048,
    NULL,
    1,
    &waterHandle,
    0
  );

  xTaskCreatePinnedToCore(
    LDRTask,
    "Light Level Data Collection",
    2048,
    NULL,
    2,
    &LDRHandle,
    0
  );


  // User input tasks
  xTaskCreatePinnedToCore(
    buttonTask,
    "Button Input",
    2048,
    NULL,
    3,
    &buttonHandle,
    1
  );

  xTaskCreatePinnedToCore(
    potentiometerTask,
    "Potentiometer Input",
    2048,
    NULL,
    3,
    &potentiometerHandle,
    0
  );

  xTaskCreatePinnedToCore(
    BLEInTask,
    "BLE Input",
    2048,
    NULL,
    1,
    &BLEInHandle,
    0
  );

  // Output/environmental parameter modification tasks
  xTaskCreatePinnedToCore(
    lightSourceTask,
    "Light Source Modification",
    2048,
    NULL,
    1,
    &lightSourceHandle,
    1
  );

  xTaskCreatePinnedToCore(
    plantWatererTask,
    "Plant Waterer",
    2048,
    NULL,
    2,
    &plantWatererHandle,
    1
  );

  xTaskCreatePinnedToCore(
    LCDTask,
    "LCD Output",
    2048,
    NULL,
    0,
    &LCDHandle,
    1
  );


  tempSemaphore = xSemaphoreCreateBinary();
  lightSemaphore = xSemaphoreCreateBinary();
  configSemaphore = xSemaphoreCreateBinary();

  xSemaphoreGive(tempSemaphore);
  xSemaphoreGive(lightSemaphore);
  xSemaphoreGive(configSemaphore);
}

void loop() {/*Unused*/}


// ======================================= All Helpers =========================================
void addVal(int num, valueContainer* container) {
  // Shifts all values right by 1, and removes the rightmost (oldest/first) value.
  for(int i = 4; i > 0; i--) {
    container->values[i] = container->values[i - 1];
  }

  // Adds the new value to the front of the array.
  container->values[0] = num;

  container->prevAvg = container->movingAvg;

  int sum = 0;
  for(int i = 0; i < 5; i++) {
    sum += container->values[i];
  }

  container->movingAvg = sum / 5;
}


void plantConfigurator(plantConfig* configs) {
  switch(myPlant) {
    case PLANT1: {
      configs->temperature = 65;
      configs-> humidity = 65;
      configs-> lightLevel = 2000;
    }

    case PLANT2: {
      configs->temperature = 85;
      configs-> humidity = 40;
      configs-> lightLevel = 3400;
    }

    case PLANT3: {
      configs->temperature = 55;
      configs-> humidity = 70;
      configs-> lightLevel = 2700;
    }
  }
}


void printStr(char str[5]) {
  setupLCD();

  int i = 0;
  while(str[i] != '\0') {
    char curr = str[i];
    char msn = curr & 0b11110000;
    char lsn = curr & 0b00001111;

    lsn = (lsn << 4);

    msn |= LCD_WRITE;
    lsn |= LCD_WRITE;

    Wire.beginTransmission(0x27);
    Wire.write(msn);
    Wire.write(msn ^ 0b00000100);
    delay(1);

    Wire.write(lsn);
    Wire.write(lsn ^ 0b00000100);

    Wire.endTransmission();
    delay(5);
    i++;
  }
}

void setupLCD() {
  Wire.beginTransmission(0x27);

  // Clear display
  Wire.write(0b00001100);
  Wire.write(0b00001000);
  Wire.write((0x01 << 4) | 0b00001100);
  Wire.write((0x01 << 4) | 0b00001000);
  delay(1);

  // Reset cursor
  Wire.write(0x80 | 0b00001100);
  Wire.write(0x80 | 0b00001000);
  Wire.write(0b00001100);
  Wire.write(0b00001000);
  delay(1);

  Wire.endTransmission();
}