#include "SparkFunISL29125.h"
#include <Wire.h>

SFE_ISL29125 sensor1;

int valorRojo = 4;
int valorVerde = 3;
int valorAzul = 2;

int dt = 1000;

void setup() {
  Serial.begin(9600);

  if (sensor1.init()){
    Serial.println("sensor1 se inicio bien");
  }else{
    Serial.println("No funciona");
  }
}
void loop() {
  int rojo = sensor1.readRed();
  int verde = sensor1.readGreen();
  int azul = sensor1.readBlue();

  if (rojo > valorRojo) {
    Serial.println("Alto Rojo");
    delay(dt);
  }else{
    Serial.println("NADA ROJO");
    delay(dt);
  }

  if (verde > valorVerde) {
    Serial.println("Alto Verde");
    delay(dt);
  }else{
    Serial.println("NADA VERDE");
    delay(dt);
  }

   if (azul > valorAzul) {
      Serial.println("Alto Azul");
      delay(dt);
    }else{
      Serial.println("NADA AZUL");
      delay(dt);
    } 
}
