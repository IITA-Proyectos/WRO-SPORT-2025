
// Blink onboard LED every 50 ms on Arduino Nano ESP32
#include <Arduino.h>
#include <SPI.h>

// Onboard LED pin for Nano ESP32 (usually 2)
#define LED_PIN_RED 14
#define LED_PIN_GREEN 15
#define LED_PIN_BLUE 16

void setup() {
  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);
}

void loop() {

  digitalWrite(LED_PIN_RED, HIGH); // LED ON
  digitalWrite(LED_PIN_GREEN, LOW);  // LED OFF
  digitalWrite(LED_PIN_BLUE, LOW);  // LED OFF
  delay(500);
  digitalWrite(LED_PIN_RED, LOW);  // LED OFF
  digitalWrite(LED_PIN_GREEN, HIGH); // LED ON
  digitalWrite(LED_PIN_BLUE, LOW);  // LED OFF
  delay(500);
  digitalWrite(LED_PIN_RED, LOW);  // LED OFF
  digitalWrite(LED_PIN_GREEN, LOW);  // LED OFF
  digitalWrite(LED_PIN_BLUE, HIGH); // LED ON
  delay(500);
}