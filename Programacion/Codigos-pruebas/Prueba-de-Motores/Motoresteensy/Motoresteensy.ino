#include <PWMServo.h>
PWMServo servito;
//CONTROL DE LOS MOTORES
/* 
¿Qué son INA e INB en el controlador VNH7100BASTR?
Son las entradas digitales que le dicen al controlador en qué dirección debe girar el motor.
 El VNH7100BASTR controla un motor DC de escobillas, y la dirección del giro depende de cómo
  estén activadas estas dos entradas.

  ENABLE: Habilita o deshabilita el controlador de motor
Cuando está en HIGH (nivel alto), el controlador habilita las salidas OUTA y OUTB, permitiendo que el motor se mueva según las señales INA, INB y PWM.

Si está en LOW (nivel bajo), el controlador apaga las salidas, como si desconectaras el motor. Aunque INA, INB y PWM estén activos, el motor no se moverá.

DIAG (diagnóstico)
El mismo pin también actúa como salida de diagnóstico (de ahí su nombre ENABLE/DIAG):

Si ocurre un error (como sobrecorriente, sobrecalentamiento, cortocircuito), el controlador pone este pin en LOW para alertarte.

Puedes leer ese pin con digitalRead() para saber si todo está funcionando bien (si está en HIGH) o hay un fallo (si está en LOW).
*/
//Entonces definimos los pines importantes

const int INA3 = 12;
const int INB3= 11;
const int PWM3 = 4;
const int INA1 = 7;
const int INB1 = 8;
const int PWM1 = 6;
const int INA2 = 5;
const int INB2 = 2;
const int PWM2 = 3; 
//const int ENABLE = 4; // opcional, si se conecta

void setup() {
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  servito.attach(29);
 // servito.write(5);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
    // Habilita el controlador
}

void loop() {
  // Mover hacia adelante
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, LOW);
  
  analogWrite(PWM2, 255);  // Velocidad (0-255) señal pwm
  //analogWrite(PWM2, 0);
 
  // 3 segundos adelante
digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  
  analogWrite(PWM1, 255);  // Velocidad (0-255) señal pwm
  //analogWrite(PWM2, 0);
 
  
  digitalWrite(INA3, HIGH);
  digitalWrite(INB3, LOW);
  
  analogWrite(PWM3, 255);  // Velocidad (0-255) señal pwm
  //analogWrite(PM2, 0);
 
  delay(3000);
  /* Mover hacia atrás
  digitalWrite(INA3, LOW);
  digitalWrite(INB3, HIGH);
  analogWrite(PWM3, 100);  // Velocidad (0-255) señal pwm

  delay(3000); // 3 segundos atrás

  // Detener
  analogWrite(PWM3, 0);
  delay(2000);*/
  //servito.write(100);
 //delay (3000);
}

