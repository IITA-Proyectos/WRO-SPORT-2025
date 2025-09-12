
#include <Arduino.h>


void setup() {
		Serial.begin(115200); // USB Serial for debug
		Serial1.begin(115200, SERIAL_8N1, 9, 8); // Serial1 for OpenMV (RX=9, TX=8)
}

void loop() {
	Serial.print("Lecture: ");
  // Read data from OpenMV camera
     // Read the most recent byte
    
  byte byteRead = Serial1.read();
  if (byteRead != 255){ // Valor que larga por defecto el UART de la OPENMV
    Serial.println(byteRead);
  }
	
  //Serial1.write('e');  // Si quiero mandar algo a la camara 
	//delay(1000);
}

