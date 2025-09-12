#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define INA1 7
#define INB1 8
#define PWM1 6

#define INA2 5
#define INB2 2
#define PWM2 3

#define INA3 12
#define INB3 11
#define PWM3 4

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float initialYaw = 0;

void setup() {
  Serial.begin(115200);

  // Configurar pines
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  // Inicializar BNO055
  if (!bno.begin()) {
    Serial.println("¡No se pudo encontrar el BNO055!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Obtener orientación inicial
  sensors_event_t event;
  bno.getEvent(&event);
  initialYaw = event.orientation.x;

  Serial.print("Yaw inicial: ");
  Serial.println(initialYaw);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  float currentYaw = event.orientation.x;

  float error = currentYaw - initialYaw;

  // Corregir para overflow de 360°
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  Serial.print("Yaw actual: ");
  Serial.print(currentYaw);
  Serial.print(" | Error: ");
  Serial.println(error);

  float tolerance = 3.0; // grados aceptables de desviación

  if (abs(error) <= tolerance) {
    // Ir recto
    avanzar();
  } else if (error > tolerance) {
    // Desviado hacia la izquierda -> girar a la derecha
    corregirDerecha();
  } else if (error < -tolerance) {
    // Desviado hacia la derecha -> girar a la izquierda
    corregirIzquierda();
  }

  delay(100); // Pequeño retardo
}

void avanzar() {
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW); analogWrite(PWM2, 60);
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW); analogWrite(PWM3, 60);
}

void corregirDerecha() {
  // Reduce velocidad del lado izquierdo para girar un poco a la derecha
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW); analogWrite(PWM2, 40);
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW); analogWrite(PWM3, 40);
}

void corregirIzquierda() {
  // Reduce velocidad del lado derecho para girar un poco a la izquierda
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
  digitalWrite(INA2, 0); digitalWrite(INB2, 1); analogWrite(PWM2, 40);
  digitalWrite(INA3, 0); digitalWrite(INB3, 1); analogWrite(PWM3, 40);
}
