
// Blink onboard LED every 50 ms on Arduino Nano ESP32
#include <Arduino.h>

// Pines de Definicion del Motor 1
#define motor1dir1 12
#define motor1dir2 11
#define motor1pwm 6
// Function to control Motor 1
void motor1(int power, int direction) {
  digitalWrite(motor1dir1, direction == 0 ? HIGH : LOW);
  digitalWrite(motor1dir2, direction == 0 ? LOW : HIGH);
  analogWrite(motor1pwm, power);
}

// Pines de Definicion del Motor 2
#define motor2dir1 10
#define motor2dir2 13  
#define motor2pwm 5

// Function to control Motor 2
void motor2(int power, int direction) {
  digitalWrite(motor2dir1, direction == 0 ? HIGH : LOW);
  digitalWrite(motor2dir2, direction == 0 ? LOW : HIGH);
  analogWrite(motor2pwm, power);
}

// Pines de Definicion del Motor 3
#define motor3dir1 17
#define motor3dir2 7
#define motor3pwm 4

// Function to control Motor 3
void motor3(int power, int direction) {
  digitalWrite(motor3dir1, direction == 0 ? HIGH : LOW);
  digitalWrite(motor3dir2, direction == 0 ? LOW : HIGH);
  analogWrite(motor3pwm, power);
}

# define direction 1
# define power 50

void setup(void)
{

  //InitializeZircon();
  pinMode(motor1dir1, OUTPUT);
  pinMode(motor1dir2, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2dir1, OUTPUT);
  pinMode(motor2dir2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
  pinMode(motor3dir1, OUTPUT);
  pinMode(motor3dir2, OUTPUT);
  pinMode(motor3pwm, OUTPUT);
  Serial.begin(115200);

}


void loop(void)
{
  Serial.println("Motor 1 Andando!");
  motor1(100, 0); // Power 100, Direction 0
  delay(1000);
  Serial.println("Motor 1 Parado!");
  motor1(0, 0); // Power 0, Direction 0
  delay(1000);

  Serial.println("Motor 2 Andando!");
  motor2(100, 0); // Power 100, Direction 0
  delay(1000);
  Serial.println("Motor 2 Parado!");
  motor2(0, 0); // Power 0, Direction 0
  delay(1000);

  Serial.println("Motor 3 Andando!");
  motor3(100, 0); // Power 100, Direction 0
  delay(1000);
  Serial.println("Motor 3 Parado!");
  motor3(0, 0); // Power 0, Direction 0
  delay(1000);
}