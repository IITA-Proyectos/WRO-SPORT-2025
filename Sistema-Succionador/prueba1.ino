#include <ESP32servo.h>
Servo myservo;

const int servoPin = 18;
const int rele = 2;
const int sensorPin = 19;

int valorSensor = 0;

// Rangosmedidos (ajustar según pruebas)
const int naranjaMin = 210;
const int naranjaMax = 1000;
const int violetaMax = 210;

// Tiempo mínimo que el valor debe mantenerse estable (ms)
const int tiempoConfirmacion = 300;

void setup() {
Serial.begin(115200);
Serial.println("Iniciando sistema...");

pinMode(rele, OUTPUT);
pinMode(sensorPin, INPUT);

myservo.attach(servoPin, 500, 2400);
myservo.write(160);
digitalWrite(rele, LOW);

Serial.println("Sistema listo.");
}

void loop() {
valorSensor = analogRead(sensorPin);
Serial.print("Lectura sensor: ");
Serial.println(valorSensor);

// Detecta color NARANJA
if (valorSensor >= naranjaMin && valorSensor <= naranjaMax) {
if (confirmarColor("naranja")) {
Serial.println("Color confirmado: NARANJA");
myservo.write(160);
digitalWrite(rele, HIGH);
delay(10000);
digitalWrite(rele, LOW);
}
}

// Detecta color VIOLETA
else if (valorSensor < violetaMax) {
if (confirmarColor("violeta")) {
Serial.println("Color confirmado: VIOLETA");
digitalWrite(rele, LOW);
myservo.write(90);
delay(4000);
myservo.write(160);
digitalWrite(rele, HIGH);
}
}

delay(200);
}

// ----------------------
//  Función de confirmación
// ----------------------
bool confirmarColor(String color) {
unsigned long startTime = millis();
while (millis() - startTime < tiempoConfirmacion) {
int lectura = analogRead(sensorPin);

if (color == "naranja") {
if (lectura < naranjaMin || lectura > naranjaMax) return false;
} else if (color == "violeta") {
if (lectura > violetaMax) return false;
}

delay(10); // pequeña pausa entre lecturas
}
return true; // se mantuvo estable todo el tiempo
}

