#include "SparkFunISL29125.h"
#include <Wire.h>

SFE_ISL29125 sensor;

int valorRojo = 19;
int valorVerde = 16;
int valorAzul = 11;

int dt = 1000;

void setup() {
  Serial.begin(9600);

  if (sensor.init()){
    Serial.println("Sensor Initialization Successful");
  }else{
    Serial.println("No funciona");
  }
}

void loop() {
  int rojo = sensor.readRed();
  int verde = sensor.readGreen();
  int azul = sensor.readBlue();

  Serial.print("Rojo: ");
  Serial.println(rojo);
  Serial.print("Verde: ");
  Serial.println(verde);
  Serial.print("Azul: ");
  Serial.println(azul);

  if (rojo == valorRojo || verde == valorVerde || azul == valorAzul) {
    Serial.println("No estan bien calibrados");
    delay(dt);
  }
}
