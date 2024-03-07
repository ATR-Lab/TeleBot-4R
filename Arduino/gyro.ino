
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done");
}

void loop() {
  mpu.update();

  Serial.print("X: ");
  Serial.print(mpu.getAngleX());
  Serial.print("\tY: ");
  Serial.print(mpu.getAngleY());
  Serial.print("\tZ: ");
  Serial.print(mpu.getAngleZ());
  Serial.println();
  delay(200);
}
