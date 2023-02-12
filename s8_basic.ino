/*****************
   Get CO2 value 
 *****************/

#include <Arduino.h>
#include "s8_uart.h"
#include <WiFiManager.h>

/* BEGIN CONFIGURATION */
#define DEBUG_BAUDRATE 115200

HardwareSerial S8_serial(0);
S8_UART *sensor_S8;
S8_sensor sensor;


void setup() {

  // Configure serial port, we need it for debug
  Serial1.begin(DEBUG_BAUDRATE);
  // set up wifi manager
  WiFiManager wifiManager;
  wifiManager.autoConnect();

  // Wait port is open or timeout
  int i = 0;
  while (!S8_serial.available() && i < 50) {
    delay(10);
    i++;
  }
  
  // First message, we are alive
  Serial1.println("");
  Serial1.println("Init");

  // Initialize S8 sensor
  S8_serial.begin(S8_BAUDRATE);
  Serial1.println("baudrate set");

  sensor_S8 = new S8_UART(S8_serial);
  Serial1.println("sensor initialized");

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  if (len == 0) {
      Serial1.println("SenseAir S8 CO2 sensor not found!");
      while (1) { delay(1); };
  }

  // Show basic S8 sensor info
  Serial1.println(">>> SenseAir S8 NDIR CO2 sensor <<<");
  Serial1.printf("Firmware version: %s\n", sensor.firm_version);
  sensor.sensor_id = sensor_S8->get_sensor_ID();
  Serial1.print("Sensor ID: 0x"); printIntToHex(sensor.sensor_id, 4); Serial1.println("");

  Serial1.println("Setup done!");
  Serial1.flush();
}


void loop() {
  
  //printf("Millis: %lu\n", millis());

  // Get CO2 measure
  sensor.co2 = sensor_S8->get_co2();
  Serial1.printf("CO2 value = %d ppm\n", sensor.co2);

  //Serial.printf("/*%u*/\n", sensor.co2);   // Format to use with Serial Studio program

  // Compare with PWM output
  //sensor.pwm_output = sensor_S8->get_PWM_output();
  //printf("PWM output = %0.0f ppm\n", (sensor.pwm_output / 16383.0) * 2000.0);

  // Wait 5 second for next measure
  delay(5000);
}