#include <DS3231.h>
#include <EEPROM.h>
#include <RH_ASK.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include "Arduino.h"  // avr core

#define INT_pin       PD2  // change need to be updated in interrupt settings
#define RF_VCC_pin    4    // unused
#define SONIC_VCC_pin 5
#define TRIG_PIN      6
#define ECHO_PIN      8
#define RF_pin        7
#define SDA_PIN       18
#define SCL_PIN       19

#define AVG_COUNT         10   // 10
#define TIME_BETWEEN_MEAS 500  // 500
// #define EEPROM

// uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false
RH_ASK rf_driver(500, RF_pin, RF_pin, RF_pin, false);

DS3231 rtc;  // Using I2C
bool alarmTriggered = false;
volatile uint16_t start_time = 0;
volatile uint16_t echo_duration = 0;
volatile uint8_t capture_done = 0;
volatile uint8_t state = 0;

uint16_t measure() {
  TCNT1 = 0;  // Reset timer (optional, but safe)
  capture_done = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  while (capture_done == 0);     // Wait for echo
  return echo_duration * 2;  // prescaler 8 means 0.5us resolution
}

uint16_t data_colection() {
  uint16_t duration_us;
  uint32_t dur_sum = 0;

  Serial.print("Duration ms sample: ");
  for (uint8_t i = 0; i < AVG_COUNT; i++) {    
    duration_us = measure();
    Serial.print(duration_us);
    Serial.print(" ");
    dur_sum = dur_sum + duration_us;
    delay(TIME_BETWEEN_MEAS);  // TODO: go to sleep between measurements
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

void init_ultrasonic_icp1() {
  // Configure Timer1
  TCCR1A = 0;                           // set to normal mode
  TCCR1B = (1 << ICES1) | (1 << CS11);  // Rising edge, prescaler = 8 (1 µs resolution)
  TIMSK1 |= (1 << ICIE1);               // Enable input capture interrupt

  sei();  // Enable global interrupts
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
  //digitalWrite(SONIC_VCC_pin, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.flush();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set Power-down mode
  sleep_enable();                       // Enable sleep mode

  // Ensure interrupts are enabled
  sei();

  sleep_cpu();  // Go to sleep

  // Execution resumes here after wakeup
  sleep_disable();  // Disable sleep to avoid accidental re-sleep

  // enbale peripherals
  digitalWrite(RF_VCC_pin, HIGH);
  //digitalWrite(SONIC_VCC_pin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup_clock_prescaler() {
  cli();  // Disable interrupts during change

  CLKPR = (1 << CLKPCE);  // Enable change of CLKPR
  CLKPR = (1 << CLKPS0);  // Set prescaler to divide by 2

  sei();  // Re-enable interrupts
}

void setNextAlarm() {
  bool h12;
  bool hPM;
  //uint8_t cMin = rtc.getMinute();
  // uint8_t next_min = (cMin + 4) / 5 * 5;
  //uint8_t next_min = cMin + 1;

  uint8_t cSec = rtc.getSecond();
  uint8_t next_sec = (cSec + 9) / 10 * 10;

  uint8_t cHour = rtc.getHour(h12, hPM);
  uint8_t cMin = rtc.getMinute();

  // Set Alarm 1 to match hour, minute, second = 0:00
  rtc.setA1Time(
      rtc.getDate(),  // day
      cHour,
      /*next_min > 60 ? 60 : next_min,*/
      cMin,
      next_sec > 60 ? 60 : next_sec,
      0b00001110,  // Match HH:MM:SS (when sec, min, and hour match)
      false, false, false);

  // enable Alarm 1 interrupts
  rtc.turnOnAlarm(1);
  // clear Alarm 1 flag
  rtc.checkIfAlarm(1);

  getCurrentTime();
  Serial.print("Next alarm set at: ");
  Serial.print(cHour);
  Serial.print(":");
  Serial.print(cMin);
  Serial.print(":");
  Serial.print(next_sec);
  Serial.println("");
}

void setCurrentTime() {
  rtc.setYear(25);
  rtc.setMonth(07);
  rtc.setDate(16);
  rtc.setDoW(3);
  rtc.setHour(11);
  rtc.setMinute(35);
  rtc.setSecond(30);
}

void getCurrentTime() {
  bool h12Flag;
  bool pmFlag;
  bool century;

  Serial.print("\nCurrent time: 20");
  Serial.print(rtc.getYear());
  Serial.print(" ");

  // then the month
  Serial.print(rtc.getMonth(century));
  Serial.print(" ");

  // then the date
  Serial.print(rtc.getDate());
  Serial.print(" ");

  // Finally the hour, minute, and second
  Serial.print(rtc.getHour(h12Flag, pmFlag));
  Serial.print(":");
  Serial.print(rtc.getMinute());
  Serial.print(":");
  Serial.print(rtc.getSecond());
  Serial.print("\n");
}

void setup() {
  setup_clock_prescaler();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RF_VCC_pin, OUTPUT);
  pinMode(SONIC_VCC_pin, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("\n---Rain water level meter---");
  Wire.begin();
  rf_driver.init();

  rtc.setClockMode(false);  // 24h mode

  // setCurrentTime();

  rtc.setA2Time(0, 0, 0xFF, 0b01100000, false, false, false);  // Upload the parameters to prevent Alarm 2 entirely
  rtc.turnOffAlarm(2);                                         // disable Alarm 2 interrupt
  rtc.checkIfAlarm(2);                                         // clear Alarm 2 flag

  setup_pinchange_interrupt();
  init_ultrasonic_icp1();

  digitalWrite(RF_VCC_pin, HIGH);
  digitalWrite(SONIC_VCC_pin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  static uint8_t meas_count;
  uint16_t duration_us = data_colection();
  //send_packet(duration_us);

  setNextAlarm();  // Schedule next 4-hour alarm

  /*EEPROM.put(2 * meas_count, duration_us);

  meas_count++;
  if (meas_count >= 1) {
    for (uint8_t i = 0; i < meas_count; i++) {
      EEPROM.get(2 * i, duration_us);
      send_packet(duration_us);
    }
    meas_count = 0;
  }*/

  go_to_sleep();
}

ISR(TIMER1_CAPT_vect) {
  

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
