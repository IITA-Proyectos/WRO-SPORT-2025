#include <arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define SWITCH_IZQUIERDO_PIN 24
#define SWITCH_DERECHO_PIN   19
#define SWITCH_TRASERO_PIN   23

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
int velocidad_adelante = 120; //antes era 70
int velocidad_atras = 60; // antes era 80
int velocidad_izquierda = 40;
int velocidad_derecha = 40;//--->22-9-2025 estaba en 40, pero no se movia a la derecha casi.
float kp = 2.0f; //antes era 10.0f
const float DEADBAND = 1.5f;
const int   MAX_CORR = 80;
float currentYaw   = 0;
float currentPitch = 0;
float currentRoll = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int estado = 0;
int estado1 = 0;

// Variables para cámara - parser no bloqueante
byte colorDetectado = 0;  // 0=nada, 1=naranja, 2=violeta
byte posicionX = 0;
byte posicionY = 0;
unsigned long ultimaDeteccionCamara = 0;
const unsigned long TIMEOUT_CAMARA = 500; // ms

// Estructura para parser UART no bloqueante
struct UARTParser {
  enum { WAIT_START, WAIT_LENGTH, WAIT_DATA } estado;
  byte buffer[10];
  byte length;
  byte index;
  unsigned long ultimoByteRecibido;
  const unsigned long TIMEOUT_BYTE = 100; // ms
} uartParser;

// ---------- choque por acelerómetro (BNO055) ----------
const float CHOQUE_UMBRAL_MSS = 2.3f;         // m/s^2 (umbral de desaceleración frontal)
const unsigned long CHOQUE_DEBOUNCE_MS = 60;  // ms de persistencia mínima
const unsigned long CHOQUE_COOLDOWN_MS = 400; // ms para evitar repeticiones
const float CHOQUE_LPF_ALPHA = 0.25f;         // filtro paso bajo (0..1)
const unsigned long CHOQUE_IGNORE_START_MS = 250; // ms ignorados al arrancar avance

// Mapa del eje frontal respecto al frame del sensor (ajustar según montaje)
const int ACCEL_FORWARD_AXIS = 0;  // 0=X, 1=Y, 2=Z
const int ACCEL_FORWARD_SIGN = +1; // +1 si el eje del sensor apunta hacia adelante

float choqueAccelLpf = 0.0f;
unsigned long choqueInicioMs = 0;
unsigned long choqueUltimoMs = 0;
unsigned long choqueIgnorarHastaMs = 0;

static inline bool detectarChoque() {
  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float mag = sqrtf(lin.x() * lin.x() + lin.y() * lin.y() + lin.z() * lin.z());
  // Filtro paso bajo
  choqueAccelLpf = choqueAccelLpf + CHOQUE_LPF_ALPHA * (mag - choqueAccelLpf);

  unsigned long now = millis();
  if (now - choqueUltimoMs < CHOQUE_COOLDOWN_MS) {
    return false;
  }

  // Ventana de ignorar tras iniciar avance
  if (now < choqueIgnorarHastaMs) {
    choqueInicioMs = 0;
    return false;
  }

  // Proyección direccional (frontal negativa indica choque)
  float axisComponent = (ACCEL_FORWARD_AXIS == 0) ? lin.x() : (ACCEL_FORWARD_AXIS == 1 ? lin.y() : lin.z());
  float forwardAccel = ACCEL_FORWARD_SIGN * axisComponent;
  bool posibleChoque = (forwardAccel <= -CHOQUE_UMBRAL_MSS);

  if (posibleChoque) {
    if (choqueInicioMs == 0) choqueInicioMs = now;
    if (now - choqueInicioMs >= CHOQUE_DEBOUNCE_MS) {
      choqueUltimoMs = now;
      choqueInicioMs = 0;
      Serial.println("Choque detectado por acelerómetro");
      return true;
    }
  } else {
    choqueInicioMs = 0;
  }
  return false;
}

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


// ---------- movimientos ----------
void pare() {
  digitalWrite(INA3, LOW); digitalWrite(INB3, LOW); analogWrite(PWM3, 0);
  digitalWrite(INA2, LOW); digitalWrite(INB2, LOW); analogWrite(PWM2, 0);
  digitalWrite(INA1, LOW); digitalWrite(INB1, LOW); analogWrite(PWM1, 0);
}

void a() { // avanzar recto “rápido” sin corrección
  digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH); analogWrite(PWM3, 50);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW);  analogWrite(PWM2, 50);
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

void ai() { // izquierda
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, 40);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW);  analogWrite(PWM2, 40);
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, 80);
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

void ad() { // derecha
  digitalWrite(INA3, LOW);  digitalWrite(INB3, HIGH); analogWrite(PWM3, 40);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 40);
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW);  analogWrite(PWM1, 80);
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

void rproporcional() { // retroceder corrigiendo
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
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW);  analogWrite(PWM3, 70);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 70);
  digitalWrite(INA1, LOW);  digitalWrite(INB1, LOW);  analogWrite(PWM1, 0);
}

void gd (){ // giro derecha
  digitalWrite(INA1, HIGH); digitalWrite(INB1, LOW); analogWrite(PWM1, 45);
  digitalWrite(INA2, HIGH); digitalWrite(INB2, LOW); analogWrite(PWM2, 45);
  digitalWrite(INA3, HIGH); digitalWrite(INB3, LOW); analogWrite(PWM3, 45);
}
void gi (){ // giro izquierda
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH); analogWrite(PWM1, 45);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 45);
  digitalWrite(INA3, LOW);  digitalWrite(INB3, HIGH); analogWrite(PWM3, 45);
}
void e(){
  pare();
  //delay(1000);
  a();
  delay(300);
  gi();
  delay(300);
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

  pinMode(SWITCH_IZQUIERDO_PIN, INPUT_PULLUP);
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
  bool switchTraseroActivado  = (digitalRead(SWITCH_TRASERO_PIN)  == HIGH); //para robot blanco HIGH
  bool switchIzquierdoActivado = (digitalRead(SWITCH_IZQUIERDO_PIN)== LOW);

  switch (estado1) {
    case 0: // izquierda hasta tocar switch izquierdo
      aiproporcional();
      if (switchIzquierdoActivado || (currentMillis - previousMillis) >= 1500) {
        pare();
        estado1 = 1;
        choqueIgnorarHastaMs = millis() + CHOQUE_IGNORE_START_MS;
        previousMillis = currentMillis;
      }
      break;

    case 1: // avanza por tiempo con corrección + freno por choque
      aproporcional();

      if (verificarPelotaVioleta()) {
        e();
        pare();
        if(currentMillis - previousMillis >= 1000){
          previousMillis = currentMillis;
          estado1 = 2;
      }
    }
      else{
      if (detectarChoque()) {
        pare();
        previousMillis = currentMillis;
        estado1 = 2;
      }
        else if (currentMillis - previousMillis >= 1500) { //depende la bateria va a cambiar la velocidad y el tiempo
        previousMillis = currentMillis;
        estado1 = 2;
      }
    }
      break;

    case 2: // derecha por tiempo
      adproporcional();
      if (currentMillis - previousMillis >= 800) {
        pare();
        estado1 = 3;
        previousMillis = currentMillis;
      }
      break;

    case 3: // retrocede hasta tocar switch trasero, corrigiendo

      if(error < -15){
           digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, 70);
           digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 0);
           digitalWrite(INA1, LOW); digitalWrite(INB1, HIGH); analogWrite(PWM1, 70);
      }

      else{
        rproporcional();
      }

      if (switchTraseroActivado || (currentMillis - previousMillis) >= 3000) {           // <-- antes comparabas == 0
        pare();                               // paro para evitar “curva” al salir
        previousMillis = currentMillis;
        estado1 = 8;
      }
      break;

    case 8: // izquierda por tiempo (corto)
      aiproporcional();

      if (currentMillis - previousMillis >= 300) {
        estado1 = 9;
        choqueIgnorarHastaMs = millis() + CHOQUE_IGNORE_START_MS;
        previousMillis = currentMillis;
      }
      break;

    case 9: // avanza por el medio + freno por choque
      aproporcional();
      if (verificarPelotaVioleta()) {
        e();
      if(currentMillis - previousMillis >= 2000){
        previousMillis = currentMillis;
        estado1 = 11;
      }    
      }
      else{
      if (detectarChoque()) {
        pare();
        previousMillis = currentMillis;
        estado1 = 11;
      }  else if (currentMillis - previousMillis >= 1500) {
        previousMillis = currentMillis;
        estado1 = 11;
      }
      }
      break;

    case 11: // derecha por tiempo
      adproporcional();
      if (currentMillis - previousMillis >= 600) {
        pare();
        estado1 = 12;
        previousMillis = currentMillis;
      }
      break;

    case 12: // retrocede hasta switch trasero
    
       if(error < -15){
          digitalWrite(INA3, LOW); digitalWrite(INB3, HIGH);  analogWrite(PWM3, 70);
          digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH); analogWrite(PWM2, 0);
          digitalWrite(INA1, LOW); digitalWrite(INB1, HIGH); analogWrite(PWM1, 70);
      }
      
      else{
        rproporcional();
      }

      if (switchTraseroActivado || (currentMillis - previousMillis) >= 3000) {
        pare();
        previousMillis = currentMillis;
        estado1 = 0;
      }
      break;

    case 13:
      // (tu rutina de esquina acá si la necesitás)
      break;
  }
  
  // Procesar UART de forma no bloqueante (solo 2 bytes por iteración)
  procesarUARTNoBloqueante();
  
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
    // pare(); // Parar para evitar pasar la pelota*/
  
  
}