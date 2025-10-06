#include <Wire.h>
#include "AS726X.h"

AS726X sensor;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if (sensor.begin() == false) {
    Serial.println("AS726X not detected!");
    while (1); // Halt if not found
  }

  // Turn on the built-in indicator LED
  sensor.enableIndicator();
  sensor.setIndicatorCurrent(3); // 0=1mA, 1=2mA, 2=4mA, 3=8mA (max brightness)

  // Or turn on the built-in illumination LED (for measurement lighting)
  sensor.enableBulb();
  sensor.setBulbCurrent(3); // same current scale
}

void loop() {
  sensor.takeMeasurements();

  if (sensor.getVersion() == SENSORTYPE_AS7263) {
    // Near IR readings (R–W)
    Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedU(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedV(), 2); Serial.print(",");
    Serial.println(sensor.getCalibratedW(), 2);
  }
  else if (sensor.getVersion() == SENSORTYPE_AS7262) {
    // Visible readings (Violet–Red)
    Serial.print(sensor.getCalibratedViolet(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedBlue(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedGreen(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedYellow(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedOrange(), 2); Serial.print(",");
    Serial.println(sensor.getCalibratedRed(), 2);
  }

  delay(1000); // log every 1s
}