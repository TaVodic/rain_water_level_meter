#include <EEPROM.h>
#include <RH_ASK.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <DS3231.h>
#include <Wire.h>

#include "Arduino.h"  // avr core

#define INT_pin PD2  // change need to be updated in interrupt settings
#define RF_pin  7

#define RF_VCC_pin    4
#define SONIC_VCC_pin 5

#define TRIG_PIN 6
#define ECHO_PIN 8

#define AVG_COUNT         10   // 10
#define TIME_BETWEEN_MEAS 500  // 500
// #define EEPROM

// uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false
RH_ASK rf_driver(500, RF_pin, RF_pin, RF_pin, false);
DS3231 rtc;

volatile uint16_t start_time = 0;
volatile uint16_t echo_duration = 0;
volatile uint8_t capture_done = 0;

uint16_t measure() {
  TCNT1 = 0;  // Reset timer (optional, but safe)
  capture_done = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  while (!capture_done);  // Wait for echo
  return echo_duration*2; // prescaler 8 means 0.5us resolution
}

void init_ultrasonic_icp1() {

  // Configure Timer1
  TCCR1A = 0;                           // set to normal mode
  TCCR1B = (1 << ICES1) | (1 << CS11);  // Rising edge, prescaler = 8 (1 µs resolution)
  TIMSK1 |= (1 << ICIE1);               // Enable input capture interrupt

  sei();  // Enable global interrupts
}

uint16_t data_colection() {
#ifdef EEPROM
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
#endif

  uint16_t duration_us;
  uint32_t dur_sum = 0;

  Serial.print("Duration ms sample: ");
  for (uint8_t i = 0; i < AVG_COUNT; i++) {
    duration_us = measure();
    Serial.print(duration_us);
    Serial.print(" ");
    dur_sum = dur_sum + duration_us;
    delay(TIME_BETWEEN_MEAS);  // TODO: go to sleep between measurements
    // upload_data();
  }

  duration_us = dur_sum / AVG_COUNT;

  Serial.print("\nDuration avg: ");
  Serial.println(duration_us);


  float tempC = rtc.getTemperature();

  Serial.print("T=");
	Serial.print(tempC, 2);

  float c = 331.3 + 0.606 * tempC; 

  Serial.print("c=");
	Serial.print(c, 2);

  float distance = (duration_us * c / 10000) / 2;
  Serial.print(" Distance avg: ");
  Serial.println(distance, 1);

#ifdef EEPROM
  Serial.print("Writing on address: ");
  Serial.println(addr);
  EEPROM.put(addr, duration_us);
  addr = addr + sizeof(duration_us);
  EEPROM.put(0, addr);
#endif

  return duration_us;
}

void send_packet(uint16_t duration_us) {
  uint8_t msg[2];
  msg[0] = duration_us;
  msg[1] = duration_us >> 8;
  rf_driver.send(msg, 2);
  rf_driver.waitPacketSent();
  // rf_driver.mode() == rf_driver.RHModeTx;
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
      while (1) {
      };
    }
    if (inChar == '.') {
      for (uint16_t i = 2; i < EEPROM.length(); i++) {
        uint8_t data = 0;
        EEPROM.put(i, data);
      }
      EEPROM.put(0, (uint16_t)2);
      Serial.println("Erase completed!");
      while (1) {
      };
    }
  }
}

void setup_pinchange_interrupt() {
  // Set PD2 (INT0) as input
  DDRD &= ~(1 << PD2);
  PORTD |= (1 << PD2);  // Enable pull-up resistor if needed

  // Configure INT0 to trigger on falling edge
  EICRA |= (1 << ISC01);  // ISC01 = 1, ISC00 = 0 => falling edge
  EICRA &= ~(1 << ISC00);

  // Enable External Interrupt Request 0 (INT0)
  EIMSK |= (1 << INT0);

  // Clear any pending external interrupt flags
  EIFR |= (1 << INTF0);
}

void go_to_sleep() {
  // disable peripherals
  digitalWrite(RF_VCC_pin, LOW);
  digitalWrite(SONIC_VCC_pin, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set Power-down mode
  sleep_enable();                       // Enable sleep mode

  // Ensure interrupts are enabled
  sei();

  sleep_cpu();  // Go to sleep

  // Execution resumes here after wakeup
  sleep_disable();  // Disable sleep to avoid accidental re-sleep

  // enbale peripherals
  digitalWrite(RF_VCC_pin, HIGH);
  digitalWrite(SONIC_VCC_pin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup_clock_prescaler() {
  cli();  // Disable interrupts during change

  CLKPR = (1 << CLKPCE);  // Enable change of CLKPR
  CLKPR = (1 << CLKPS1);  // Set prescaler to divide by

  sei();  // Re-enable interrupts
}

void setup() {
  setup_clock_prescaler();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RF_VCC_pin, OUTPUT);
  pinMode(SONIC_VCC_pin, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);

  rf_driver.init();
  Wire.begin();

  setup_pinchange_interrupt();
  init_ultrasonic_icp1();

  digitalWrite(RF_VCC_pin, HIGH);
  digitalWrite(SONIC_VCC_pin, HIGH);

  // EEPROM.put(0, 2);  // store address 2 at adress 0
}

void loop() {
  static uint32_t cTime;
  if (cTime + 2000 < millis()) {
    static uint8_t meas_count;
    uint16_t duration_us = data_colection();

    /*EEPROM.put(2 * meas_count, duration_us);

    meas_count++;
    if (meas_count >= 3) {
      for (uint8_t i = 0; i < meas_count; i++) {
        EEPROM.get(2 * i, duration_us);
        send_packet(duration_us);
      }
      meas_count = 0;
    }*/
    cTime = millis();
  }

  // go_to_sleep();
}

ISR(TIMER1_CAPT_vect) {
  static uint8_t state = 0;

  if (state == 0) {
    // Rising edge detected
    start_time = ICR1;
    TCCR1B &= ~(1 << ICES1);  // Switch to falling edge
    state = 1;
  } else {
    // Falling edge detected
    echo_duration = ICR1 - start_time;
    TCCR1B |= (1 << ICES1);  // Switch back to rising edge
    capture_done = 1;
    state = 0;
  }
}
