#include <Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define UGAO_PO_IMPULSU 360.0 / 8000.0

Adafruit_MPU6050 imu;

float enkoder_ugao() {
  static Encoder enkoder(2, 3);
  return UGAO_PO_IMPULSU * enkoder.read();
}

float imu_ugao() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.print(a.acceleration.z);
  Serial.println();
  
  return 0;
} 

void setup() {
  Serial.begin(115200);
   if (!imu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  imu.setAccelerometerRange(MPU6050_RANGE_2_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void loop() {
//  Serial.println(enkoder_ugao());
  imu_ugao();
}
