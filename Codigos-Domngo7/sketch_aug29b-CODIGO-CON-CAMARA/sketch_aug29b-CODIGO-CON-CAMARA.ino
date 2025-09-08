#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SWITCH_IZQUIERDO_PIN 30
#define SWITCH_DERECHO_PIN 31
#define SWITCH_TRASERO_PIN 28
//...................................motores..........................................
#define INA1 7
#define INB1 8 
#define PWM1 6

#define INA2 5
#define INB2 2
#define PWM2 3

#define INA3 12
#define INB3 11
#define PWM3 4

#define START_BYTE 0xAA
//....................................................................................

// Crear objeto del sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float error = 0;
float initialYaw = 0;
float initialPitch = 0;
float initialRoll = 0;
int cont = 0;
float kp = 0.5;

unsigned long tiempoAnterior = 0;
int estado = 0;
int estado2 = 0;
int estado1 = 0;
bool enMovimiento = false;
unsigned long previousMillis = 0;
bool estado_anterior = estado;

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200); 
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  
  pinMode(SWITCH_IZQUIERDO_PIN, INPUT_PULLUP);
  pinMode(SWITCH_DERECHO_PIN, INPUT_PULLUP);
  pinMode(SWITCH_TRASERO_PIN, INPUT_PULLUP);
  
  if (!bno.begin()) {
    Serial.println("¡No se pudo encontrar el BNO055!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);

  initialYaw = event.orientation.x;
  initialPitch = event.orientation.z;
  initialRoll = event.orientation.y;

  Serial.println("Orientación inicial:");
  Serial.print("Yaw: "); Serial.println(initialYaw);
  Serial.print("Pitch: "); Serial.println(initialPitch);
  Serial.print("Roll: "); Serial.println(initialRoll);
  Serial.println("--------------------------");

}
void a() { 

  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 220);

  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 220);

  digitalWrite(INA1, 0);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}
void ai() {
  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 60);

  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 60);

  digitalWrite(INA1, 0);
  digitalWrite(INB1, 1);
  analogWrite(PWM1, 105);
}

void ad() {
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 60);

  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 60);

  digitalWrite(INA1, 1);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 105);
}

void pare() {
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 0);

  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 0);

  digitalWrite(INA1, 1);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}

void r() {
  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 180);

  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 180);

  digitalWrite(INA1, 1);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}
void gd (){
 digitalWrite(INA1, HIGH);
 digitalWrite(INB1, LOW);
 analogWrite(PWM1, 0);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, HIGH);
 digitalWrite(INB2, LOW);
 analogWrite(PWM2, 45);
 digitalWrite(INA3, HIGH);
 digitalWrite(INB3, LOW);
 analogWrite(PWM3, 45);  
 }
 void gi (){
 digitalWrite(INA1, LOW);
 digitalWrite(INB1, HIGH);
 analogWrite(PWM1, 0);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, LOW);
 digitalWrite(INB2, HIGH);
 analogWrite(PWM2, 45);
 digitalWrite(INA3, LOW);
 digitalWrite(INB3, HIGH);
 analogWrite(PWM3, 45);  
 }
 void rutina(){

        sensors_event_t event;
        bno.getEvent(&event);

        float currentYaw = event.orientation.x;
        float currentPitch = event.orientation.z;
        float currentRoll = event.orientation.y;

        error =  initialYaw - currentYaw;
        float correcion=error*kp;
        if(error < -180)
        {
         error = error + 360;
        }
      
  unsigned long currentMillis = millis();
  
  bool switchDerechoActivado = digitalRead(SWITCH_DERECHO_PIN) == LOW;
  bool switchTraseroActivado = digitalRead(SWITCH_TRASERO_PIN) == LOW;
  bool switchIzquierdoActivado = digitalRead(SWITCH_IZQUIERDO_PIN) == LOW; 

switch (estado1) {

      case 0: //primer movimiento a la derecha
    
       if (!enMovimiento) {
        ad();
        previousMillis = currentMillis;
        enMovimiento = true;
       }
       if (currentMillis - previousMillis >= 500) {
        pare();
        estado1 = 1;
        enMovimiento = false;
        previousMillis = currentMillis;
       }
       Serial.println (estado);
      break;

    case 1: //avance para tirar pelotas de el lado derecho
    
        
      //if (error < -180) {
       //error = error + 360;
      //0}
      if (error > 180) error -= 360;
      if (error < -180) error += 360;
      if (error > 7) {
       gi();
      }
      else if (error < -7) {
       gd();
      }
      else {  
        a();
        if ( currentMillis - previousMillis >= 3000){
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 2; 
        }
       }
Serial.println (estado);
    break;

    
    case 2: //retroceso
     
      //if (error < -180) {
       //error = error + 360;
      //}
    if (error > 180) error -= 360;
    if (error < -180) error += 360;  
     if (error > 3) {
      gi();
    }
    else if (error < -3) {
      gd();
    }
    else {
       r();                
       
   if (switchTraseroActivado == 1) {
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 3; 
       }
 }
      break;


    case 3: //moviendose a la izquierda hasta chocar con la pared
      
      if (!enMovimiento) {
        ai();
        enMovimiento = true;
        previousMillis = currentMillis;
      }
      if (switchIzquierdoActivado== 1) {
        pare();
        enMovimiento = false;
        estado1 = 4;
        previousMillis = currentMillis;
      }
      Serial.println (estado);
      break;

    case 4: //avance para tirar pelotitas de la izquierda
              
      //if (error < -180) {
       //error = error + 360;
      //}
      if (error > 180) error -= 360;
      if (error < -180) error += 360;
      if (error > 7) {
       gi();
      }
      else if (error < -7) {
       gd();
      }
     else {  
        a();
        if ( currentMillis - previousMillis >= 3000){
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 5; 
        }
       }
Serial.println (estado);
    break;  // <-- ¡Importante!

    case 5: //novimiento a la derecha
    
      if (!enMovimiento) {
        ad();
        previousMillis = currentMillis;
        enMovimiento = true;
      }
      if (currentMillis - previousMillis >= 1500) {
        pare();
        estado1 = 6;
        enMovimiento = false;
        previousMillis = currentMillis;
      }
      Serial.println (estado);
    break;

    case 6: //retroceso
    
     //if (error < -180) {
       //error = error + 360;
      //}
    if (error > 180) error -= 360;
     if (error < -180) error += 360;
     if (error > 5) {
      gi();
    }
    else if (error < -5) {
      gd();
    }
    else {
       r();                
      
      if(switchTraseroActivado == 1) {
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 7; 
       }
 }
Serial.println (estado);
      break;

    case 7: //pausita que no se nota pero por si acaso no la quito 
      if (!enMovimiento) {
        previousMillis = currentMillis;
        enMovimiento = true;
      }
      if (currentMillis - previousMillis >= 200) {
        estado1 = 8;
        enMovimiento = false;
        previousMillis = currentMillis;
      }
      Serial.println (estado);
    break;

    case 8: //mueve hacia la izquierda hasta hubicarse enla posicion inicial
     
      if (!enMovimiento) {
        ai();
        enMovimiento = true;
        previousMillis = currentMillis;
      }
      if (currentMillis - previousMillis >= 500) {
        pare();
        enMovimiento = false;
        estado1 = 9;
        previousMillis = currentMillis;
      }
      Serial.println (estado);
    break;

    case 9: //avance para tirar las pelotitas del medio
     
        
      //if (error < -180) {
       //error = error + 360;
     // }
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
      if (error > 7) {
       gi();
      }
      else if (error < -7) {
       gd();
      }
     else {  
        a();
        if ( currentMillis - previousMillis >= 3000){
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 10; 
        }
       }
  
Serial.println (estado);
      break;

      case 10: //retroceso para hubicarse en la posicion inicial y enpezar de nuevo
      
       //if (error < -180) {
       //error = error + 360;
       if (error > 180) error -= 360;
       if (error < -180) error += 360;
      //}
     if (error > 5) {
      gi();
      }
     else if (error < -5) {
      gd();
      }
      else {
       r();                
      
      if(switchTraseroActivado == 1) {
        enMovimiento = false;
        previousMillis = currentMillis;
         estado1 = 0; 
       }
      }
     Serial.println (estado);
     break;
     } 
  }

void loop() {
  static enum { WAIT_START, WAIT_LENGTH, WAIT_DATA } estado = WAIT_START;
  static byte buffer[10];
  static byte length = 0;
  static byte index = 0;
   
   
  while (Serial1.available()) {
    byte b = Serial1.read();
    //Serial.println("Iniv");
    //Serial.print("Iniv");
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
            byte codigo = buffer[0];//le llega el color
            byte x = buffer[1];
            byte y = buffer[2];

            Serial.print("OK -> Código: ");
            Serial.print(codigo);
            Serial.print(", X: ");
            Serial.print(x);
            Serial.print(", Y: ");
            Serial.println(y);
///------------------------------------------------------------------

  switch(codigo){
    case 0:
     rutina();
    break;

    case 1:
        a();
    break;

    case 2:
     pare();
    break;
  }
          }
    
//--------------------------------------------------------------------            
           else {
            Serial.println(" Checksum incorrecto");

          }
          estado = WAIT_START; // Reiniciar estado
        }
        break;
    }
  }
  
}