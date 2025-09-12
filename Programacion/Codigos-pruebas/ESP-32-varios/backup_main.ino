#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SWITCH_IZQUIERDO_PIN 19
# define SWITCH_DERECHO_PIN 24
#define SWITCH_TRASERO_PIN 23

//...................................motores..........................................
#define INA1 12
#define INB1 11
#define PWM1 6

#define INA2 10
#define INB2 13
#define PWM2 5

#define INA3 17
#define INB3 7
#define PWM3 4
//....................................................................................

// Crear objeto del sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float correccion;
float error = 0;
float initialYaw = 0;
float initialPitch = 0;
float initialRoll = 0;
int cont = 0;
float kp = 1;
float vel0 = 100;

unsigned long tiempoAnterior = 0;
int estado = 0;
bool enMovimiento = false;
unsigned long previousMillis = 0;
int estado1 = 0;

void setup() {
  Serial.begin(115200);

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

  delay(3000);

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
void a() { 
 //motor izquierdo
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 200);
 //motor derecho
  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 200);
 //motor atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}
void aproporcional() { //se corrige mientras avanza
 //motor izquierdo
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 100 - correccion);
 //motor derecho
  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 100 + correccion);

  if (error = 0){
  //motor de atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
  }
  else if (error > 0){
 //motor atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 1);
  analogWrite(PWM1, correccion);
 }
  else if (error < 0){
   //motor atras
   digitalWrite(INA1, 1);
   digitalWrite(INB1, 0);
   analogWrite(PWM1, - correccion);
  }
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
  analogWrite(PWM1, 103);
}
void aiproporcional() {
  if (error = 0){
   //motor izquierdo
   digitalWrite(INA3, 1);
   digitalWrite(INB3, 0);
   analogWrite(PWM3, 60);
   //motor derecho
   digitalWrite(INA2, 1);
   digitalWrite(INB2, 0);
   analogWrite(PWM2, 60);
   //motor atras
   digitalWrite(INA1, 0);
   digitalWrite(INB1, 1);
   analogWrite(PWM1, 103);
  }
  else if (error > 0){
   //motor izquierdo
   digitalWrite(INA3, 1);
   digitalWrite(INB3, 0);
   analogWrite(PWM3, 60 + correccion);
   //motor derecho
   digitalWrite(INA2, 1);
   digitalWrite(INB2, 0);
   analogWrite(PWM2, 60 + correccion);
   //motor atras
   digitalWrite(INA1, 0);
   digitalWrite(INB1, 1);
   analogWrite(PWM1, 103 - correccion);
  }
  else if (error < 0){
   //motor izquierdo
   digitalWrite(INA3, 1);
   digitalWrite(INB3, 0);
   analogWrite(PWM3, 60 - correccion);
   //motor derecho
   digitalWrite(INA2, 1);
   digitalWrite(INB2, 0);
   analogWrite(PWM2, 60 - correccion);
   //motor atras
   digitalWrite(INA1, 0);
   digitalWrite(INB1, 1);
   analogWrite(PWM1, 103 + correccion);
  }
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
  analogWrite(PWM1, 103);
}
void adproporcional() {
  if (error = 0){
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 60);

  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 60);

  digitalWrite(INA1, 1);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 103);
  }
  else if (error > 0){
   //motor izquierdo
   digitalWrite(INA3, 0);
   digitalWrite(INB3, 1);
   analogWrite(PWM3, 60 + correccion);
   //motor derecho
   digitalWrite(INA2, 0);
   digitalWrite(INB2, 1);
   analogWrite(PWM2, 60 + correccion);
   //motor atras
   digitalWrite(INA1, 1);
   digitalWrite(INB1, 0);
   analogWrite(PWM1, 103 - correccion);
  }
  else if (error < 0){
   //motor izquierdo
   digitalWrite(INA3, 0);
   digitalWrite(INB3, 1);
   analogWrite(PWM3, 60 - correccion);
   //motor derecho
   digitalWrite(INA2, 0);
   digitalWrite(INB2, 1);
   analogWrite(PWM2, 60 - correccion);
   //motor atras
   digitalWrite(INA1, 1);
   digitalWrite(INB1, 0);
   analogWrite(PWM1, 103 + correccion);
  }
}

void rproporcional() {//para que se corriga mientras retrocede
  //motor izquierdo
  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 100 + correccion);
  //motor derecho
  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 100 - correccion);

  if (error = 0){
   //motor atras
   digitalWrite(INA1, 0);
   digitalWrite(INB1, 1);
   analogWrite(PWM1, 0);
  }

  else if (error > 0){ //si se desvia a la derecha para que se corriga con la rueda de atras tambien 
   //motor atras
   digitalWrite(INA1, 0);
   digitalWrite(INB1, 1);
   analogWrite(PWM1, correccion);
  }

  else if (error < 0){//si se desvia a la izquierda para que se corriga con la rueda de atras tambien
   //motor atras
   digitalWrite(INA1, 1);
   digitalWrite(INB1, 0);
   analogWrite(PWM1, - correccion); //correcion sise va para la izquierda seria un numero negativo, asi que lo tenemos que restar par que nos de un numero positivo
  }
}
void r() {
  //motor izquierdo
  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 150);
  //motor derecho
  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 150);
  //motor atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}

void gd (){
 digitalWrite(INA1, HIGH);
 digitalWrite(INB1, LOW);
 analogWrite(PWM1, 45);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, HIGH);
 digitalWrite(INB2, LOW);
 analogWrite(PWM2, 0);
 digitalWrite(INA3, HIGH);
 digitalWrite(INB3, LOW);
 analogWrite(PWM3, 0);  
}
void gi (){
 digitalWrite(INA1, LOW);
 digitalWrite(INB1, HIGH);
 analogWrite(PWM1, 45);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, LOW);
 digitalWrite(INB2, HIGH);
 analogWrite(PWM2, 0);
 digitalWrite(INA3, LOW);
 digitalWrite(INB3, HIGH);
 analogWrite(PWM3, 0);  
}
void ri(){
  //motor izquierdo
  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 70);
 //motor derecho
  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 0);
 //motor atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 1);
  analogWrite(PWM1, 70);
}
void gad(){
 //motor izquierdo
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 200);
 //motor derecho
  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 150);
 //motor atras
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 1);
  analogWrite(PWM1, 80);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  float currentYaw = event.orientation.x;
  float currentPitch = event.orientation.z;
  float currentRoll = event.orientation.y;
  Serial.print("Orientation X: ");
  Serial.println(currentYaw);

  if(currentYaw > 180){
   currentYaw = currentYaw - 360;
  }

  error =  currentYaw - initialYaw;
  correccion= error * kp;

  unsigned long currentMillis = millis();

  bool switchDerechoActivado = digitalRead(SWITCH_DERECHO_PIN) == LOW;
  bool switchTraseroActivado = digitalRead(SWITCH_TRASERO_PIN) == LOW;
  bool switchIzquierdoActivado = digitalRead(SWITCH_IZQUIERDO_PIN) == LOW; 

  switch (estado) {
    case 0: //se mueve hacia la izquierda hasta que toque el swich
      ai();

      if (switchIzquierdoActivado== 1) {
        pare();
        estado = 1;
        previousMillis = currentMillis;
      }

    break;


    case 1://avanza por tiempo
      if (error > 50) {
       gi();
      }
      else if (error < -50) {
       gd();
      }
      else {  
        aproporcional();
        if ( currentMillis - previousMillis >= 1500){
        previousMillis = currentMillis;
         estado = 2; 
        }
      }
    break;
    
    case 2://se mueve hacia la derecha por tiempo
      ad();

      if (currentMillis - previousMillis >= 1100) {
        pare();
        estado = 3;
        previousMillis = currentMillis;
      }
    break;

    case 3:// retrocede hasta que toque el swich

     if (error > 50) {
       gi();
      }
     else if (error < -50) {
       gd();
      }
     else {
       rproporcional();                      
       if (switchTraseroActivado == 0) {
        previousMillis = currentMillis;
         estado = 8; 
       }
      }      
    break;

    case 8://se mueve a la izquierda por tiempo
     ai();
     if (currentMillis - previousMillis >= 500){//puede que el tiempo no este bien y se deba cambiar
       pare();
       estado = 1;
       previousMillis = currentMillis;
      }
    break;

    case 9: //por el mismo tiempo que el case 1. Avanza por el medio

      if (error > 50) {
       gi();
      }
      else if (error < -50) {
       gd();
      }
      else {  
        aproporcional();
        if ( currentMillis - previousMillis >= 15000){
        previousMillis = currentMillis;
         estado = 11; 
        }
      }
    break;

    case 11://se mueve hacia la derecha por tiempo
     ad();
     if (currentMillis - previousMillis >= 600){//tiwmpo debe ser mayor al del caso 8
      pare();
      estado = 12;
      previousMillis = currentMillis;
     }
    break;

    case 12://retrocede hasta que toque el swich y comienza de nuevo

     if (error > 50) {
       gi();
      }
     else if (error < -50) {
       gd();
      }
     else {
       rproporcional();                      
       if (switchTraseroActivado == 0) {
        previousMillis = currentMillis;
         estado = 0; 
       }
      }      
    break;

    case 13:

    break;
  }
}