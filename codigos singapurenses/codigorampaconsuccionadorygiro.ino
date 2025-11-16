#include <arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
Servo myservo;

//#define SWITCH_IZQUIERDO_PIN 29
#define SWITCH_DERECHO_PIN 24
#define SWITCH_TRASERO_PIN 23
// -------------- Motores --------------
#define INA1 12
#define INB1 11
#define PWM1 6

#define INA2 10
#define INB2 13
#define PWM2 5

#define INA3 17
#define INB3 7
#define PWM3 4
#define START_BYTE 0xAA
// -------------------------------------

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
const int servoPin = 18;
const int rele = 2;
const int sensorPin = 19;
int morado = 0;
int valorSensor = 0;
int captura = 0;
// Rangosmedidos (ajustar según pruebas)
const int naranjaMin = 210;
const int naranjaMax = 1000;
const int violetaMax = 210;

// Tiempo mínimo que el valor debe mantenerse estable (ms)
const int tiempoConfirmacion = 300;


float correccion = 0;
float error = 0;
float initialYaw = 0;   // heading inicial tal como lo da el BNO (0..360)
float initialPitch = 0;
float initialRoll = 0;
int velocidad_adelante = 100; //antes era 70
int velocidad_atras = 90; // antes era 80
int velocidad_izquierda = 42;
int velocidad_derecha = 42;
int velocidad_medio = 120;
float kp = 2.0f; //antes era 10.0f--<22-9-2025 pasamos a 10, antes era 2
const float DEADBAND = 1.5f;
const int   MAX_CORR = 80;
float currentYaw   = 0;
float currentPitch = 0;
float currentRoll = 0;
float kpatras=4.0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousMillissuc = 0;
int estado = 0;//camara
int estado1 = 0;//rutina
int estadosuc = 0;//giro de esquina

// Variables para cámara - parser no bloqueante
byte colorDetectado = 0;  // 0=nada, 1=naranja, 2=violeta
byte posicionX = 0;
byte posicionY = 0;
unsigned long ultimaDeteccionCamara = 0;
const unsigned long TIMEOUT_CAMARA = 1000; // ms

// Velocidad base lateral para ad
int velocidad_lateralad = 40;
int velocidad_lateral_traseraad = 78.70;
// Velocidad base lateral para ai
int velocidad_lateralai = 40;
int velocidad_lateral_traseraai = 78.70;
// Velocidad base lateral para airam
int velocidad_lateralairam = 39;
int velocidad_lateral_traseraairam = 90;
// Estructura para parser UART no bloqueante
struct UARTParser {
  enum { WAIT_START, WAIT_LENGTH, WAIT_DATA } estado;
  byte buffer[10];
  byte length;
  byte index;
  unsigned long ultimoByteRecibido;
  const unsigned long TIMEOUT_BYTE = 100; // ms
} uartParser;

// ---------- helpers de ángulo ----------
static inline float wrap180(float a){
  while (a <= -180.0f) a += 360.0f;
  while (a >   180.0f) a -= 360.0f;
  return a;
}

static inline int clampPWM(int v){
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

//

// ---------- movimientos ----------
void pare() {
  digitalWrite(INA3, LOW); digitalWrite(INB3, LOW); analogWrite(PWM3, 0);
  digitalWrite(INA2, LOW); digitalWrite(INB2, LOW); analogWrite(PWM2, 0);
  digitalWrite(INA1, LOW); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
}

void a() { // avanzar recto “rápido” sin corrección
  digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH); analogWrite(PWM3, 80);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW);  analogWrite(PWM2, 80);
  digitalWrite(INA1, LOW);  digitalWrite(INB1, LOW);  analogWrite(PWM1, 0);
}

void aproporcional() { // avanza corrigiendo
  int pwmL = clampPWM(velocidad_adelante - (int)correccion);
  int pwmR = clampPWM(velocidad_adelante + (int)correccion);

  // motor izquierdo (M3) en reversa
  digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH); analogWrite(PWM3, pwmL);
  // motor derecho (M2) adelante
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW);  analogWrite(PWM2, pwmR);

  // rueda trasera ayuda según signo
  //if (fabs(error) <= DEADBAND){
    digitalWrite(INA1, LOW); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
  /*} else if (error > 0){
    // inclinación positiva → empuja “atrás adelante”
    digitalWrite(INA1, LOW); 
    digitalWrite(INB1, HIGH); 
    analogWrite(PWM1, clampPWM((int)fabs(correccion)));
  } else { // error < 0
    digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, clampPWM((int)fabs(correccion)));
  }*/
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
  analogWrite(PWM1, 85.5);
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
  analogWrite(PWM1, 70);
}

void airam() { // izquierda proporcional con corrección de rumbo
  int pwmM3 = clampPWM(velocidad_lateralairam - (int)correccion); // M3 adelante
  int pwmM2 = clampPWM(velocidad_lateralairam + (int)correccion); // M2 adelante

  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, pwmM3);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW);  analogWrite(PWM2, pwmM2);

  // Rueda trasera mantiene lateral fijo
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, velocidad_lateral_traseraairam);
}

void aiproporcional() { // izquierda
  int pwmL = clampPWM(velocidad_izquierda + (int)correccion);
  int pwmR = clampPWM(velocidad_izquierda + (int)correccion);
  int pwmM = clampPWM(velocidad_medio + (int)correccion);

  // motor izquierdo (M3) adelante
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, pwmL);
  // motor derecho (M2) atrás
  digitalWrite(INA2, HIGH);  digitalWrite(INB2, LOW); analogWrite(PWM2, pwmR);

  // rueda atrás apoya corrección

    digitalWrite(INA1, LOW); digitalWrite(INB1, HIGH); analogWrite(PWM1, 80);

}

void adproporcional(){
  int pwmL = clampPWM(velocidad_derecha - (int)correccion);
  int pwmR = clampPWM(velocidad_derecha - (int)correccion);
  int pwmM = clampPWM(velocidad_medio + (int)correccion);

  // motor izquierdo (M3) adelante
  digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, pwmL);
  // motor derecho (M2) atrás
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, pwmR);

  // rueda atrás apoya corrección

    digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 80);

}

void rproporcional(float correcion) { // retroceder corrigiendo
  int pwmL = clampPWM(velocidad_atras + (int)correccion);
  int pwmR = clampPWM(velocidad_atras - (int)correccion);

  // motor izquierdo (M3) adelante
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, pwmL);
  // motor derecho (M2) atrás
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, pwmR);

  // rueda atrás apoya corrección
  //if (fabs(error) <= DEADBAND){
    digitalWrite(INA1, LOW); digitalWrite(INB1, HIGH); analogWrite(PWM1, 0);
  /*} else if (error > 0){
    digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, clampPWM((int)fabs(correccion)));
  } else {
    digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW);  analogWrite(PWM1, clampPWM((int)fabs(correccion)));
  }*/
}

void r() { // retro recto sin corrección (se usa poco)
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, 150);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 150);
  digitalWrite(INA1, LOW);  digitalWrite(INB1, LOW);  analogWrite(PWM1, 0);
}
void rmax() { // retro recto sin corrección (se usa poco)
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, 250);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 250);
  digitalWrite(INA1, LOW);  digitalWrite(INB1, LOW);  analogWrite(PWM1, 0);
}


void gd (){ // giro derecha
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 180);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW); analogWrite(PWM2, 180);
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW); analogWrite(PWM3, 180);
}
void gi (){ // giro izquierda
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, 20);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 20);
  digitalWrite(INA3, LOW);  digitalWrite(INB3, HIGH); analogWrite(PWM3, 20);
}
//
void ggd (){
 digitalWrite(INA1, HIGH);
 digitalWrite(INB1, LOW);
 analogWrite(PWM1,40);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, HIGH);
 digitalWrite(INB2, LOW);
 analogWrite(PWM2, 40);
 digitalWrite(INA3, HIGH);
 digitalWrite(INB3, LOW);
 analogWrite(PWM3, 20);  
 }
 void ggi (){
 digitalWrite(INA1, LOW);
 digitalWrite(INB1, HIGH);
 analogWrite(PWM1, 40);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, LOW);
 digitalWrite(INB2, HIGH);
 analogWrite(PWM2, 20);
 digitalWrite(INA3, LOW);
 digitalWrite(INB3, HIGH);
 analogWrite(PWM3, 40);  
 }
void procesarMensajeCompleto() {
  // Verificar checksum
  byte checksum = 0;
  for (int i = 0; i < uartParser.length; i++) {
    checksum += uartParser.buffer[i];
  }
  checksum %= 256;

  if (checksum == uartParser.buffer[uartParser.length]) {
    // Datos válidos
    colorDetectado = uartParser.buffer[0];
    posicionX = uartParser.buffer[1];
    posicionY = uartParser.buffer[2];
    ultimaDeteccionCamara = millis();

    Serial.print("Camara -> Color: ");
    Serial.print(colorDetectado);
    Serial.print(", X: ");
    Serial.print(posicionX);
    Serial.print(", Y: ");
    Serial.println(posicionY);
  } else {
    Serial.println("Checksum incorrecto");
  }
}

// ---------- funciones de cámara no bloqueante ----------
void procesarUARTNoBloqueante() {
  static const int MAX_BYTES_POR_ITERACION = 2; // Solo 2 bytes por iteración
  int bytesProcesados = 0;
  
  while (Serial1.available() && bytesProcesados < MAX_BYTES_POR_ITERACION) {
    byte b = Serial1.read();
    bytesProcesados++;
    uartParser.ultimoByteRecibido = millis();
    
    switch (uartParser.estado) {
      case UARTParser::WAIT_START:
        if (b == START_BYTE) {
          uartParser.estado = UARTParser::WAIT_LENGTH;
        }
        break;

      case UARTParser::WAIT_LENGTH:
        uartParser.length = b;
        uartParser.index = 0;
        uartParser.estado = UARTParser::WAIT_DATA;
        break;

      case UARTParser::WAIT_DATA:
        uartParser.buffer[uartParser.index++] = b;
        if (uartParser.index >= uartParser.length + 1) { // +1 para checksum
          procesarMensajeCompleto();
          uartParser.estado = UARTParser::WAIT_START;
        }
        break;
    }
  }
  
  // Timeout: reiniciar si no hay datos por 100ms
  if (millis() - uartParser.ultimoByteRecibido > uartParser.TIMEOUT_BYTE) {
    uartParser.estado = UARTParser::WAIT_START;
  }
}


bool verificarPelotaNaranja() {
  return (millis() - ultimaDeteccionCamara < TIMEOUT_CAMARA && colorDetectado == 1);
}

bool verificarPelotaVioleta() {
  return (millis() - ultimaDeteccionCamara < TIMEOUT_CAMARA && colorDetectado == 2);
}

// -------------------------------------

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200,SERIAL_8N1, 9, 8); 

  pinMode(INA1, OUTPUT); pinMode(INB1, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT); pinMode(INB2, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(INA3, OUTPUT); pinMode(INB3, OUTPUT); pinMode(PWM3, OUTPUT);

  //pinMode(SWITCH_IZQUIERDO_PIN, INPUT_PULLUP);
  pinMode(SWITCH_DERECHO_PIN,   INPUT_PULLUP);
  pinMode(SWITCH_TRASERO_PIN,   INPUT_PULLUP);
  pinMode(rele, OUTPUT);
  pinMode(sensorPin, INPUT);

  myservo.attach(servoPin, 500, 2400);
  myservo.write(180);
  digitalWrite(rele, 0);



  delay(3000);

  if (!bno.begin()) {
    Serial.println("¡No se pudo encontrar el BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  initialYaw   = event.orientation.x; // 0..360
  initialPitch = event.orientation.z;
  initialRoll  = event.orientation.y;

  previousMillis = millis();

  Serial.println("Orientación inicial:");
  Serial.print("Yaw: ");   Serial.println(initialYaw);
  Serial.print("Pitch: "); Serial.println(initialPitch);
  Serial.print("Roll: ");  Serial.println(initialRoll);
  Serial.println("--------------------------");

  currentMillis = millis();
  previousMillis = currentMillis;
  
  // Inicializar parser UART no bloqueante
  uartParser.estado = UARTParser::WAIT_START;
  uartParser.ultimoByteRecibido = currentMillis;
  
  Serial.println("Sistema inicializado - Parser UART no bloqueante activo");
}

void loop() {
   procesarUARTNoBloqueante();
   if(millis()- ultimaDeteccionCamara > TIMEOUT_CAMARA){
    colorDetectado = 0;
   }
  sensors_event_t event;
  bno.getEvent(&event);

  currentYaw   = event.orientation.x; // 0..360
  currentPitch = event.orientation.z;
  currentRoll  = event.orientation.y;

  // Llevo ambos (target y medición) a -180..+180 y calculo error como arco corto
  float currentYaw180  = wrap180(currentYaw);
  float initialYaw180  = wrap180(initialYaw);
  error = wrap180(currentYaw180 - initialYaw180);

  // zona muerta y saturación de corrección
  float eForCtrl = (fabs(error) < DEADBAND) ? 0.0f : error;
  correccion = eForCtrl * kp;
  if (correccion >  MAX_CORR) correccion =  MAX_CORR;
  if (correccion < -MAX_CORR) correccion = -MAX_CORR;

 currentMillis = millis();

  bool switchDerechoActivado  = (digitalRead(SWITCH_DERECHO_PIN)  == LOW); // LOW = presionado
  bool switchTraseroActivado  = (digitalRead(SWITCH_TRASERO_PIN)  == LOW); //para robot blanco HIGH
  //bool switchIzquierdoActivado = (digitalRead(SWITCH_IZQUIERDO_PIN)== LOW);

  switch (estado1) {
    case 0:
      //if (!enMovimiento) {  
        adproporcional();
       // enMovimiento = true;
       previousMillis = currentMillis;
      //}
      if(switchDerechoActivado || currentMillis - previousMillis >= 2000 ){
        pare();
        //enMovimiento = false;
        estado1 = 1;
        //estado1 = 1;
        previousMillis = currentMillis;
      }
      if(currentMillis - previousMillis >= 1000){
        pare();
        //enMovimiento = false;
        estado1 = 1;
        //estado1 = 1;
        previousMillis = currentMillis;
      }
      break;

    case 1:
     aproporcional();
    if (verificarPelotaVioleta()){
      if(captura == 0){
      estadosuc = 1;
      estado1 = 21;
      }
        else{
          digitalWrite(rele,0);
        }
      }
     if (currentRoll > 5 || currentMillis - previousMillis >= 2000 ){
      pare();
      estado1 = 18;
      previousMillis = currentMillis;
     }
    
       break;  // <-- ¡Importante!portante!
    case 18:
    //aiproporcional();
    rmax();  // ejecutar movimiento constantemente
    if (currentMillis - previousMillis >= 60) {   
      pare();          
        estado1 = 2;
        //estado1 = 3;        
        previousMillis = currentMillis;
    }
    break;

    case 2:
    //aiproporcional();
    aiproporcional();  // ejecutar movimiento constantemente
    if (currentMillis - previousMillis >= 700) { 
        pare();             
        estado1 = 3;
        //estado1 = 3;        
        previousMillis = currentMillis;
    }
      
    break;

    case 3:
    error = wrap180(currentYaw180 - initialYaw180-10);

  // zona muerta y saturación de corrección
   //eForCtrl = (fabs(error) < DEADBAND) ? 0.0f : error;
  correccion = eForCtrl * kpatras;
  if (correccion >  MAX_CORR) correccion =  MAX_CORR;
  if (correccion < -MAX_CORR) correccion = -MAX_CORR;
    if(error > 15){
           digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, 0);
           digitalWrite(INA2, HIGH);  digitalWrite(INB2, LOW); analogWrite(PWM2, 80);
           digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 80);
      }

      else{
        rproporcional(correccion);
      }           
      if (switchTraseroActivado || currentMillis - previousMillis >= 1500) {
        pare();
         estado1 = 14; 
         previousMillis = currentMillis;
       }
      break; 
      case 14:
      aproporcional();
      if(currentMillis - previousMillis >= 60){
        pare();
        estado1 = 16;
        previousMillis = currentMillis;
      }
      break;
      case 16:
      r();
      if(currentMillis - previousMillis >= 200){
        pare();
        estado1 = 8;
        previousMillis = currentMillis;
      }
      break;
    case 8:
      adproporcional();
      if(currentMillis - previousMillis >= 350){
        pare();
      estado1 = 9;
      previousMillis = currentMillis; 
      }
      break;
    case 9:
     aproporcional();
   if (verificarPelotaVioleta()){
       if(captura == 0){
      estadosuc = 1;
      estado1 = 21;
      pare();
      }
     if (currentRoll > 5 || currentMillis - previousMillis >= 2000 ){
      pare();
      estado1 = 19;
      previousMillis = currentMillis;
     }
     break;
       case 19:
    //aiproporcional();
    rmax();  // ejecutar movimiento constantemente
    if (currentMillis - previousMillis >= 60) {   
      pare();          
        estado1 = 11;
        //estado1 = 3;        
        previousMillis = currentMillis;
    }
    break;
    case 11:
    aiproporcional();
    if(currentMillis - previousMillis >= 400){
      pare();
      estado1 = 12;
      previousMillis = currentMillis; 
      }
      break;
      
    case 12:
   error = wrap180(currentYaw180 - initialYaw180-10);

  // zona muerta y saturación de corrección
   //eForCtrl = (fabs(error) < DEADBAND) ? 0.0f : error;
  correccion = eForCtrl * kpatras;
  if (correccion >  MAX_CORR) correccion =  MAX_CORR;
  if (correccion < -MAX_CORR) correccion = -MAX_CORR;
    if(error > 15){
           digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, 0);
           digitalWrite(INA2, HIGH);  digitalWrite(INB2, LOW); analogWrite(PWM2, 80);
           digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 80);
      }

      else{
        rproporcional(correccion);
    if (switchTraseroActivado || currentMillis - previousMillis >= 2000) {
        pare();
        estado1 = 15;
         previousMillis = currentMillis;
        }
        break;
    case 15:
      aproporcional();
      if(currentMillis - previousMillis >= 60){
        pare();
        estado1 = 17;
        previousMillis = currentMillis;
      }
      break;
      case 17:
      r();
      if(currentMillis - previousMillis >= 200){
        pare();
        estado1 = 13;
        previousMillis = currentMillis;
      }
      break;
    case 13:
    adproporcional();
    if(switchDerechoActivado || currentMillis - previousMillis >= 1500) {
    pare();
    estado1 = 1;
    }
    break;

   case 21:
   gi();
   if(error > 85){
    pare();
    previousMillis = currentMillis;
    estado1 = 22;
   }
   break;

   case 22:
   gd();
   if(currentMillis - previousMillis >= 100){
   pare();
   previousMillis = currentMillis;
   estado1 = 23;
   }
   break;

   case 23:
   if(currentMillis - previousMillis >= 20000){
    estado1 = 0;
    previousMillis = currentMillis;}
}}
  switch (estadosuc) {
    case 0:
    digitalWrite(rele, 0);//motor apagado
    myservo.write(180);//servo cerrado
    break;

    case 1:
     digitalWrite(rele, 1);
     if(analogRead(sensorPin) < 500){
      previousMillissuc = currentMillis;
      estadosuc = 2;
     }
     break;

     case 2:
     pare();
     if(currentMillis - previousMillissuc >= 1000){
      estadosuc = 3;
      previousMillissuc = currentMillis;
     }
     break;

     case 3:
     if(analogRead(sensorPin) < 250){
      myservo.write(90);
      estadosuc = 4;
      previousMillissuc = currentMillis;
     }else{
      digitalWrite(rele, 0);
      previousMillissuc = currentMillis;
      estadosuc = 5;
     }
     break;

     case 4:
     if(currentMillis - previousMillissuc >= 3000){
      myservo.write(180);
      digitalWrite(rele,0);
      captura = 1;
      estadosuc = 0;
      previousMillissuc = currentMillis;
     }
     break;

     case 5:
      if(currentMillis - previousMillissuc >= 5000){
        estadosuc = 1;
        previousMillissuc = currentMillis;
        }
     break;
  }
  }}