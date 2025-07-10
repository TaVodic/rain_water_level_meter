#include <EEPROM.h>

#include "Arduino.h"  // avr core

#define TRIG_PIN 9
#define ECHO_PIN 10

#define AVG_COUNT 10

// float duration, distance;

uint16_t measure() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (uint16_t)pulseIn(ECHO_PIN, HIGH);
}

void data_colection() {
  static uint16_t addr;
  static uint8_t firstRun = true;

  if (firstRun) {
    EEPROM.get(0, addr);  // Read 2 bytes into addr
    firstRun = false;
  }

  if (addr >= EEPROM.length()) {
    Serial.println("EEPROM full!");
    return;
  }

  uint16_t duration_us;
  uint32_t dur_sum = 0;
  Serial.print("Dur frag: ");
  for (uint8_t i = 0; i < AVG_COUNT; i++) {
    duration_us = measure();
    Serial.print(duration_us);
    Serial.print(" ");
    dur_sum = dur_sum + duration_us;
    delay(500);
    upload_data();
  }

  duration_us = dur_sum / 10;

  Serial.print("\nDuration avg: ");
  Serial.print(duration_us);

  float distance = (duration_us * .0343) / 2;
  Serial.print(" Distance avg: ");
  Serial.println(distance, 1);

  Serial.print("Writing on address: ");
  Serial.println(addr);
  EEPROM.put(addr, duration_us);
  addr = addr + sizeof(duration_us);
  EEPROM.put(0, addr);
}

void upload_data() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      for (uint16_t i = 0; i < 1024; i = i + 2) {
        uint16_t data = 0;
        EEPROM.get(i, data);
        Serial.println(data);
      }
      Serial.println("Upload completed!\n");
      while (1){};
    }
    if (inChar == '.') {
      for (uint16_t i = 2; i < EEPROM.length(); i++) {
        uint8_t data = 0;
        EEPROM.put(i, data);        
      }
      EEPROM.put(0, (uint16_t)2);
      Serial.println("Erase completed!");
      while (1){};
    }
  }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200);

  uint32_t c_time = millis();
  while(c_time + 2000 > millis()){
    upload_data();
  }  

  //EEPROM.put(0, 2);  // store address 2 at adress 0
}

void loop() {
  uint32_t c_time = 0;
  if (c_time + 1000 < millis()){
    data_colection();
    c_time = millis();
  }
  
  upload_data();
}