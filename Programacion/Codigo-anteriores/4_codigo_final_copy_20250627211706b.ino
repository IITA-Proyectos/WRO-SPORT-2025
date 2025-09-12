#define START_BYTE 0xAA
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PIDController.hpp>

// Pines del motor A (activo)
#define INA1 7
#define INB1 8
#define PWM1 6

// Pines del motor B (activo)
#define INA2 5
#define INB2 2
#define PWM2 3

// Pines del motor C (inactivo)
#define INA3 12
#define INB3 11
#define PWM3 4

// Sensor BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// PID para controlar el rumbo (Yaw)
PID::PIDParameters<double> pidParamsYaw(1.8, 0.0, 0.8); // Ajusta según pruebas 1.8, 0.0, 0.8 = todo bien
PID::PIDController<double> pidYaw(pidParamsYaw);

// Velocidad base
const int basePWM = 60;
float initialYaw = 0;

void setup() {
  Serial.begin(115200);    // Debug por USB
  Serial1.begin(19200);    // UART desde OpenMV
   // Configurar pines de todos los motores
  pinMode(INA1, OUTPUT); pinMode(INB1, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT); pinMode(INB2, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(INA3, OUTPUT); pinMode(INB3, OUTPUT); pinMode(PWM3, OUTPUT);

  if (!bno.begin()) {
    Serial.println("¡No se pudo encontrar el BNO055!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(500);

  // Leer yaw inicial
  sensors_event_t event;
  bno.getEvent(&event);
  initialYaw = event.orientation.x;

  // Configurar PID
  pidYaw.Setpoint = initialYaw;
  pidYaw.SetOutputLimits(-100, 100);  // Ajusta corrección
  pidYaw.TurnOn();

  Serial.println("Yaw deseado:");
  Serial.println(initialYaw);
}
void controlMotor(int INA,int INB,int ina1, int inb1, int PWM, int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  digitalWrite(INA, ina1);
  digitalWrite(INB, inb1);
  analogWrite(PWM, pwmValue);
}

void apagarMotor(int INA, int INB, int PWM) {
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  analogWrite(PWM, 0);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  float currentYaw = event.orientation.x;

  pidYaw.Input = currentYaw;
  pidYaw.Update();

  int correction = (int)pidYaw.Output;

  // PWM ajustado para ambos motores activos
  int pwmA = basePWM + correction;
  int pwmB = basePWM - correction;

  pwmA = constrain(pwmA, 0, 60);
  pwmB = constrain(pwmB, 0, 60);

  // Motor C apagado
  apagarMotor(INA1, INB1, PWM1);

  // Aplicar control a los motores activos
  int ina1=1;
  int inb1=0;

  // Depuración serial
  Serial.print("Yaw: "); Serial.print(currentYaw);
  Serial.print(" | PWM A: "); Serial.print(pwmA);
  Serial.print(" | PWM B: "); Serial.print(pwmB);
  Serial.print(" | Corrección: "); Serial.println(correction);

  delay(50);

  static enum { WAIT_START, WAIT_LENGTH, WAIT_DATA } estado = WAIT_START;
  static byte buffer[10];
  static byte length = 0;
  static byte index = 0;

  while (Serial1.available()) {
    byte b = Serial1.read();

    switch (estado) {
      case WAIT_START:
        if (b == START_BYTE) {
          estado = WAIT_LENGTH;
        }
        break;

      case WAIT_LENGTH:
        length = b;
        index = 0;
        estado = WAIT_DATA;
        break;

      case WAIT_DATA:
        buffer[index++] = b;
        if (index >= length + 1) { // +1 para el checksum
          // Verificar checksum
          byte checksum = 0;
          for (int i = 0; i < length; i++) checksum += buffer[i];
          checksum %= 256;

          if (checksum == buffer[length]) {
            // Datos válidos
            byte codigo = buffer[0];
            byte x = buffer[1];
            byte y = buffer[2];
           int correction = (int)pidYaw.Output;
           sensors_event_t event;
           bno.getEvent(&event);
           float currentYaw = event.orientation.x;
           pidYaw.Input = currentYaw;
           pidYaw.Update();

           pwmA = basePWM + correction;
           pwmB = basePWM - correction;

           pwmA = constrain(pwmA, 0, 60);
           pwmB = constrain(pwmB, 0, 60);

            switch (codigo)
            {
              case 1:
              controlMotor(INA1, INB1,inb1,ina1, PWM1, pwmA);
              controlMotor(INA2, INB2,ina1,inb1, PWM2, pwmB);
              delay(5); 
              break;
              
              case 0:
            
              ina1 = 1;
              inb1 = 0;
               controlMotor(INA1, INB1,ina1,inb1, PWM1, pwmA);
               controlMotor(INA2, INB2,inb1,ina1, PWM2, pwmB);
               delay(5);
              if(correction > 90 && correction < 91){
               controlMotor(INA1, INB1,inb1,ina1, PWM1, pwmA);
               controlMotor(INA2, INB2,inb1,ina1, PWM2, pwmB);
               delay(5);
              }
            } 
            
          } else {
            Serial.println("⚠️ Checksum incorrecto");
          }
          estado = WAIT_START; // Reiniciar estado
        }
        break;
    }
  }
}
