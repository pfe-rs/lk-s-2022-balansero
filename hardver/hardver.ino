#include <Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define UGAO_PO_IMPULSU 360.0 / 8000.0

template <typename T> 
struct vec3d {
  T x, y, z;  
};

Adafruit_MPU6050 imu;
vec3d<float> acc_error;
vec3d<float> gyr_error;

float cmpFilter(float prev_angle, float theta_accel, float theta_gyro){
  const float alpha = 0.5;
  return (1-alpha)*(prev_angle + theta_gyro) + (alpha)*(theta_accel);
}

float enkoder_ugao() {
  static Encoder enkoder(2, 3);
  return UGAO_PO_IMPULSU * enkoder.read();
}

float imu_ugao() {
  sensors_event_t a, g, temp;
  //  noInterrupts();
  long trenutno_merenje_us = micros();
  imu.getEvent(&a, &g, &temp);
  // interrupts();
  static long prethodno_merenje_us;
  float dt = trenutno_merenje_us - prethodno_merenje_us;
  prethodno_merenje_us = trenutno_merenje_us ;

  a.acceleration.x = acc_error.x + ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
  a.acceleration.y = acc_error.y + ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI));

//  Serial.print(a.acceleration.x);
//  Serial.print(" ");
  Serial.print(a.acceleration.y);
//  Serial.print(" ");
//  Serial.print(a.acceleration.z);
  Serial.println();
//
//  Serial.print(g.gyro.x);
//  Serial.print(" ");
//  Serial.print(g.gyro.y);
//  Serial.print(" ");
//  Serial.print(g.gyro.z);
//  Serial.println();


//accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  float theta_gyro = g.gyro.z * dt;
  float theta_accel = (atan(-1 * a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI) - acc_error.y;
  
  static float theta;
  theta = cmpFilter(theta, theta_accel, theta_gyro);
  
  return theta;
} 

void calculate_IMU_error() {
  sensors_event_t a, g, temp;
  int c;
  while (c < 200) {
    imu.getEvent(&a, &g, &temp);
    acc_error.x = acc_error.x + ((atan((a.acceleration.y) / sqrt(pow((a.acceleration.x), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
    acc_error.y = acc_error.y + ((atan(-1 * (a.acceleration.x) / sqrt(pow((a.acceleration.y), 2) + pow((a.acceleration.z), 2))) * 180 / PI));
    gyr_error.x = gyr_error.x + g.gyro.x;
    gyr_error.y = gyr_error.y + g.gyro.y;
    gyr_error.z = gyr_error.z + g.gyro.z;
    ++c;
  }
  acc_error.x /= 200;
  acc_error.y /= 200;
  gyr_error.x /= 200;
  gyr_error.y /= 200;
  gyr_error.z /= 200;  
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
  calculate_IMU_error();
  
}

void loop() {
//  Serial.println(enkoder_ugao());
  imu_ugao();
}
