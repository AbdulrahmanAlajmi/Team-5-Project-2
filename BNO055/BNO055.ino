

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100) //tell the sensor to sample every 100 ms
Adafruit_BNO055 IMU = Adafruit_BNO055();    // sensor object to interact with the sesor through the object (IMU)
void setup() { 
   // put your main code here, to run repeatedly:
  Serial.begin(9600);
  IMU.begin();                          // start the sensor
  delay(1000);                          
  IMU.setExtCrystalUse(true);           // use the crystal on board of th chip to get better result
 
  
}

void loop() {
  // put your main code here, to run repeatedly:
imu:: Vector<3> Accelerometer = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
imu:: Vector<3> GyroScope = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
imu:: Vector<3> MagnetoMeter = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

Serial.print("Acceleration: ");
Serial.print("x: ");
Serial.print(Accelerometer.x());
Serial.print(", ");
Serial.print("y: ");
Serial.print(Accelerometer.y());
Serial.print(", ");
Serial.print("z: ");
Serial.println(Accelerometer.z());
delay(500);
Serial.print("Angular velocity: ");
Serial.print("x: ");
Serial.print(GyroScope.x());
Serial.print(", ");
Serial.print("y: ");
Serial.print(GyroScope.y());
Serial.print(", ");
Serial.print("z: ");
Serial.println(GyroScope.z());
delay(500);
Serial.print("Magnetic Fiel: ");
Serial.print("x: ");
Serial.print(MagnetoMeter.x());
Serial.print(", ");
Serial.print("y: ");
Serial.print(MagnetoMeter.y());
Serial.print(", ");
Serial.print("z: ");
Serial.println(MagnetoMeter.z());

delay(BNO055_SAMPLERATE_DELAY_MS);

}                                                                                                                                
