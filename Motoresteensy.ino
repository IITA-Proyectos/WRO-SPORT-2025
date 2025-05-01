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
const int INA = 5;
const int INB = 2;
const int PWM = 3;
const int ENABLE = 4; // opcional, si se conecta

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  
  digitalWrite(ENABLE, HIGH);  // Habilita el controlador
}

void loop() {
  // Mover hacia adelante
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  analogWrite(PWM, 200);  // Velocidad (0-255) señal pwm

  delay(3000); // 3 segundos adelante

  // Mover hacia atrás
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
  analogWrite(PWM, 200);  // Velocidad (0-255) señal pwm

  delay(3000); // 3 segundos atrás

  // Detener
  analogWrite(PWM, 0);
  delay(2000);
}

