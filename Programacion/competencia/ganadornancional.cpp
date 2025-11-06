//codigo nacional robot rampa
#include <arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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

float correccion = 0;
float error = 0;
float initialYaw = 0;   // heading inicial tal como lo da el BNO (0..360)
float initialPitch = 0;
float initialRoll = 0;
int velocidad_adelante = 76;//antes era 70
int velocidad_atras = 70; // antes era 80
int velocidad_izquierda = 40;
int velocidad_derecha = 40;
float kp = 2.0f; //antes era 10.0f--<22-9-2025 pasamos a 10, antes era 2
const float DEADBAND = 1.5f;
const int   MAX_CORR = 80;
float currentYaw   = 0;
float currentPitch = 0;
float currentRoll = 0;
float kpatras=4.0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int estado = 0;//camara
int estado1 = 0;//rutina
int esquina = 0;//giro de esquina

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

void gd (){ // giro derecha
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 45);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW); analogWrite(PWM2, 0);
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW); analogWrite(PWM3, 0);
}
void gi (){ // giro izquierda
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, 45);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 0);
  digitalWrite(INA3, LOW);  digitalWrite(INB3, HIGH); analogWrite(PWM3, 0);
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
      if(switchDerechoActivado || (currentMillis - previousMillis) >= 1500) {
        pare();
        //enMovimiento = false;
        estado1 = 1;
        //estado1 = 1;
        previousMillis = currentMillis;
      }
      break;

    case 1:
      /*if (error < -180) {
       error = error + 360;
      }
      if (error > 7) {
       gi();
      }
      else if (error < -7) {
       gd();
      }
      else {*/ 
    /*if (error < -180) {
       error = error + 360;
      }
     if (error > 7) {
      gi();
    }
    else if (error < -7) {
      gd();
    }
    else {*/
     aproporcional();
     if (currentRoll > 4 || currentMillis - previousMillis >= 2000 ){
      pare();
      estado1 = 2;
      previousMillis = currentMillis;
     }
    
       break;  // <-- ¡Importante!
  
    case 2:
    //aiproporcional();
    aiproporcional();  // ejecutar movimiento constantemente
    if (currentMillis - previousMillis >= 750) { 
        pare();             
        estado1 = 3;
        //estado1 = 3;        
        previousMillis = currentMillis;
    }
      
    break;

    case 3:
    error = wrap180(currentYaw180 - initialYaw180-5);

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
      if (switchTraseroActivado || (currentMillis - previousMillis) >= 2000) {
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
    /*if (error < -180) {
       error = error + 360;
      }
     if (error > 7) {
      gi();
    }
    else if (error < -7) {
      gd();
    }
    else {*/
     aproporcional();
     if (currentRoll > 4){
      pare();
      estado1 = 11;
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
    correccion = eForCtrl * kpatras;
  if (correccion >  MAX_CORR) correccion =  MAX_CORR;
  if (correccion < -MAX_CORR) correccion = -MAX_CORR;
     if(error > 14.6){
           digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, 0);
           digitalWrite(INA2, HIGH);  digitalWrite(INB2, LOW); analogWrite(PWM2, 80);
           digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 80);
      }
      else{
    rproporcional(correccion);
  }
    if (switchTraseroActivado || (currentMillis - previousMillis) >= 3500) {
        pare();
        estado1 = 13;
         previousMillis = currentMillis;
        }
        break;
    case 13:
    switch (esquina){
    case 0: 
    adproporcional();
    if(currentMillis - previousMillis >= 1400){
      pare();
        esquina = 1;
        previousMillis = currentMillis;
        }
        break;
    case 1 :
    aiproporcional();
    if(currentMillis - previousMillis >= 170){
    pare();
    esquina = 2;
    previousMillis = currentMillis; }
    break;

    case 2:
    ggi();
    if(currentMillis - previousMillis >= 170){
    pare();
    esquina = 3;
    previousMillis = currentMillis; }
    break;

    case 3:
    ggd();
    if(currentMillis - previousMillis >= 170){
    pare();
    esquina = 4;
    previousMillis = currentMillis; }
    break;

    case 4:
    adproporcional();
    if(currentMillis - previousMillis >= 250){
      esquina = 5;
    }
      break;
    case 5:
     /*if (error < -180) {
       error = error + 360;
      }
     if (error > 50) {
      gi();
    }
    else if (error < -50) {
      gd();
    }
    else {*/
    rproporcional(correccion);
    if (switchTraseroActivado || (currentMillis - previousMillis) >= 3500) {
        pare();
        estado1 = 1;
        esquina = 0;
        previousMillis = currentMillis;
        }
      else if (currentMillis - previousMillis >= 150){
        pare();
        estado1 = 1;
        esquina = 0;
        previousMillis = currentMillis;
      }
      
    break;

  
  // Procesar UART de forma no bloqueante (solo 2 bytes por iteración)
 
  
  // Ejemplo de uso de la cámara (descomenta según necesites):
  /*
  if (verificarPelotaNaranja()) {
    Serial.println("¡Pelota NARANJA detectada!");
    // Tu lógica aquí - por ejemplo:
    // aproporcional(); // Avanzar hacia la pelota
  }
  
  if (verificarPelotaVioleta()) {
    Serial.println("¡Pelota VIOLETA detectada!");
    // Tu lógica aquí - por ejemplo:
    // pare(); // Parar para evitar pasar la pelota
  }
  */
}}}

