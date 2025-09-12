#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
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
//....................................................................................
// Crear objeto del sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float error = 0;//error para PID
//float kd = 
// Variables para ángulos iniciales
float initialYaw = 0;
float initialPitch = 0;
float initialRoll = 0;
float kp=10.0;
float vel0=100;
void setup() {
  Serial.begin(115200);
 // delay(1000);
//...............inicializamos pines de motores...................................
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  if (!bno.begin()) {
    Serial.println("¡No se pudo encontrar el BNO055!");
    while (1);
  }

  // Esperar a que el sensor se estabilice
 // delay(1000);
  bno.setExtCrystalUse(true);// Activa el uso del cristal externo de 32.768 kHz en el sensor BNO055.

  // Leer orientación inicial
  sensors_event_t event;
  bno.getEvent(&event);//Llama a la función getEvent() del objeto bno (que representa el sensor BNO055)

  initialYaw = event.orientation.x;   // Heading (Yaw)
  initialPitch = event.orientation.z; // Pitch
  initialRoll = event.orientation.y;  // Roll

  Serial.println("Orientación inicial:");
  Serial.print("Yaw: "); Serial.println(initialYaw);
  Serial.print("Pitch: "); Serial.println(initialPitch);
  Serial.print("Roll: "); Serial.println(initialRoll);
  Serial.println("--------------------------");
}
void a() { //crear funcion donde las velocidades varien y no sean fijas 

  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 45);

  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 45);

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
 analogWrite(PWM2, 45);
 digitalWrite(INA3, HIGH);
 digitalWrite(INB3, LOW);
 analogWrite(PWM3, 45);  
 }
 void gi (){
 digitalWrite(INA1, LOW);
 digitalWrite(INB1, HIGH);
 analogWrite(PWM1, 45);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, LOW);
 digitalWrite(INB2, HIGH);
 analogWrite(PWM2, 45);
 digitalWrite(INA3, LOW);
 digitalWrite(INB3, HIGH);
 analogWrite(PWM3, 45);  
 }
 void ai() {

  digitalWrite(INA3, 1);
  digitalWrite(INB3, 0);
  analogWrite(PWM3, 40);

  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 40);

  digitalWrite(INA1, 0);
  digitalWrite(INB1, 1);
  analogWrite(PWM1, 65);
}
 void ad() {

  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 40);

  digitalWrite(INA2, 0);
  digitalWrite(INB2, 1);
  analogWrite(PWM2, 40);

  digitalWrite(INA1, 1);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 60);
}
void pare(){

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
void loop() {
  // Leer orientación actual
  
  sensors_event_t event;//sensors_event_t Es una estructura (struct) definida por
  // la biblioteca Adafruit_Sensor.h. Sirve como un contenedor estandarizado de datos del sensor
  //Sirve para recibir y almacenar los datos que devuelve un sensor cuando se llama a funciones como:
  bno.getEvent(&event);//Llama a la función getEvent() del objeto bno (que representa el sensor BNO055)
//........almacenamos angulos en posicion actual:...........
  float currentYaw = event.orientation.x;
  float currentPitch = event.orientation.z;
  float currentRoll = event.orientation.y;

  error =  initialYaw - currentYaw;
  float correcion=error*kp;
  if(error < -180){
  error = error + 360;}
  
  Serial.println(error);
 /* 
  Serial.println("Orientación actual:");
  Serial.print("Yaw: "); Serial.print(currentYaw);
  Serial.print(" (Inicial: "); Serial.print(initialYaw); Serial.println(")");

  Serial.print("Pitch: "); Serial.print(currentPitch);
  Serial.print(" (Inicial: "); Serial.print(initialPitch); Serial.println(")");

  Serial.print("Roll: "); Serial.print(currentRoll);
  Serial.print(" (Inicial: "); Serial.print(initialRoll); Serial.println(")");
*/
  Serial.println(error); 

  Serial.println("--------------------------");
  

  if(error > 3){
    gi();
  }
  else if(error < -3){
    gd();
  }
  else if(error > -3 && error <  3){
    
   a();
  }
}