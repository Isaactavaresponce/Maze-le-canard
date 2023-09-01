#include "SparkFunISL29125.h"
#include <Wire.h>

SFE_ISL29125 sensor1;

void setup() {
  Serial.begin(9600);

  if (sensor1.init()){
    Serial.println("sensor1 se inicio bien");
  }else{
      Serial.println("No funciona");
  }
}
void loop() {
  if (Serial.available()) {
    char input = Serial.read();

    if (input == 'i') {
      int rojo = sensor1.readRed();
      int verde = sensor1.readGreen();
      int azul = sensor1.readBlue();
      
      int promedioRojo = rojo / 1;
      int promedioVerde = verde / 1;
      int promedioAzul = azul / 1;

      Serial.print("Promedio Rojo: ");
      Serial.println(promedioRojo);
      Serial.print("Promedio Verde: ");
      Serial.println(promedioVerde);
      Serial.print("Promedio Azul: ");
      Serial.println(promedioAzul);
    }
  }
}
