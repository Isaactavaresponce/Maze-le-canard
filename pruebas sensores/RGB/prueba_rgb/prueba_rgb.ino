#include "SparkFunISL29125.h"
#include <Wire.h>

SFE_ISL29125 sensor;

void setup() {
  Serial.begin(9600);

  if (sensor.init())
  {
    Serial.println("Sensor Initialization Successful\n\r");
  }

  // Calibrate the sensor
  
}

void loop() {
  int red = sensor.readRed();
  int green = sensor.readGreen();
  int blue = sensor.readBlue();

  int averageRed = red / 3;
  int averageGreen = green / 3;
  int averageBlue = blue / 3;

  Serial.print("Promedio Rojo: ");
  Serial.println(averageRed);
  Serial.print("Promedio Verde: ");
  Serial.println(averageGreen);
  Serial.print("Promedio Azul: ");
  Serial.println(averageBlue);

  delay(1000);
}
