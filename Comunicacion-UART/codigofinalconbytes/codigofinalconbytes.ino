
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

void setup() {
  Serial.begin(115200);    // Debug por USB
  Serial1.begin(19200);    // UART desde OpenMV
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);
}


void a() {
  // Motor 3 inactivo
  digitalWrite(INA3, 0);
  digitalWrite(INB3, 1);
  analogWrite(PWM3, 255);

  // Motor 2 inactivo
  digitalWrite(INA2, 1);
  digitalWrite(INB2, 0);
  analogWrite(PWM2, 255);

  // Motor 1 activo
  digitalWrite(INA1, 0);
  digitalWrite(INB1, 0);
  analogWrite(PWM1, 0);
}

void estp(){ 

  digitalWrite(INA3, HIGH);
  digitalWrite(INB3, LOW);
  analogWrite(PWM3, 0);
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(PWM2, 0); 
    digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(PWM1, 0); 
}
 void g (){
 digitalWrite(INA1, HIGH);
 digitalWrite(INB1, LOW);
 analogWrite(PWM1, 35);  // Velocidad (0-255) señal pwm
 digitalWrite(INA2, HIGH);
 digitalWrite(INB2, LOW);
 analogWrite(PWM2, 35);
 digitalWrite(INA3, HIGH);
 digitalWrite(INB3, LOW);
 analogWrite(PWM3, 35);  
 }
void barrido1(){
  analogWrite(PWM2, 0);
  digitalWrite(INA3, HIGH);
  digitalWrite(INB3, LOW);
  analogWrite(PWM3, 70); 
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, HIGH);
  analogWrite(PWM1, 70);

}
void barrido2(){
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, HIGH);
  analogWrite(PWM2, 70);
  analogWrite(PWM3, 0); 
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 70);  
 }

void loop() {
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

            Serial.print("OK -> Código: ");
            Serial.print(codigo);
            Serial.print(", X: ");
            Serial.print(x);
            Serial.print(", Y: ");
            Serial.println(y);
            switch (codigo)  {
              case 1:
                  a();
                  //delay(5);
                  estp();
                  break;
              case 0:
                  estp();
                  //g();
                  //delay(5);
                  break;  
            } 
          } else {
            Serial.println(" Checksum incorrecto");

          }
          estado = WAIT_START; // Reiniciar estado
        }
        break;
    }
  }
}
