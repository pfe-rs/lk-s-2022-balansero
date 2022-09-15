//#include <Wire.h>
//
//#define SIGA 2
//#define SIGB 3
//#define SIGZ 4
//
//const int input = 12; // This is where the input is fed.
//int pulse = 0; // Variable for saving pulses count.
//int var = 0;
//
////#define SIGZ = 4;
//const int MPU = 0x68; // MPU6050 I2C address
//float AccX, AccY, AccZ;
//float GyroX, GyroY, GyroZ;
//float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
//float roll, pitch, yaw;
//float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
//float elapsedTime, currentTime, previousTime;
//int c = 0;
//float angle =0;
//int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
//int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
//int giro_deadzone=1;
//
//int16_t ax, ay, az,gx, gy, gz;
//
//int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
//int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
//
//void setup() {
//  
//  Serial.begin(115200);
//  Wire.begin();                      // Initialize comunication
//  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
//  Wire.write(0x6B);                  // Talk to the register 6B
//  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
//  Wire.endTransmission(true);        //end the transmission
//  
//  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
//  Wire.beginTransmission(MPU);
//  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
//  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
//  Wire.endTransmission(true);
//  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
//  Wire.beginTransmission(MPU);
//  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
//  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
//  Wire.endTransmission(true);
//  delay(20);
//  pinMode(SIGA,INPUT);
//  pinMode(SIGB,INPUT);
//  //pinMode(SIGZ,INPUT);
//  pinMode(input, INPUT);
//  Serial.println(F("No pulses yet...")); // Message to send initially (no pulses detected yet).
//
//  
//   
///*
//  // wait for ready
//  while (Serial.available() && Serial.read()); // empty buffer
//  while (!Serial.available()){
//    Serial.println(F("Send any character to start sketch.\n"));
//    delay(1500);
//  
//  // Call this function if you need to get the IMU error values for your module
//  calculate_IMU_error();
//  delay(20);
//  }
//  while (Serial.available() && Serial.read()); // empty buffer again
//
//  // start message
//  Serial.println("\nMPU6050 Calibration Sketch");
//  delay(2000);
//  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
//  delay(3000);
//  // verify connection
//  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//  delay(1000);
//  // reset offsets
//  accelgyro.setXAccelOffset(0);
//  accelgyro.setYAccelOffset(0);
//  accelgyro.setZAccelOffset(0);
//  accelgyro.setXGyroOffset(0);
//  accelgyro.setYGyroOffset(0);
//  accelgyro.setZGyroOffset(0);
//*/
//
//}
//
//
// int counter = 0;
//  int prethodna = 0;
//  int prethodna1 = 0;
//
//  int counter1=0;
//
//void loop() {
//  int trenutna = digitalRead(SIGA);
//  int trenutna1 = digitalRead(SIGB);
//  /*int trenutna2 =*/ digitalWrite(SIGZ, 1);
////  Serial.print(trenutna);
////  Serial.print(" ");
////  Serial.print(trenutna1);
////  Serial.print(" ");
////  Serial.print(trenutna2);
////  Serial.println();
//  if (prethodna == 0 && trenutna == 1) {
//    counter++;
//  }
//  
//
//  if (prethodna1 == 0 && trenutna1 == 1) {
//    counter1++; 
//  }
//
//  if ((prethodna == 0 && trenutna == 1) || (prethodna1 == 0 && trenutna1 == 1)) {
//    Serial.print(counter);
//  Serial.print(" ");
//  Serial.println(counter1);
//  } 
//  prethodna1 = trenutna1;
//  prethodna = trenutna;
//  
//  
//  
//  // === Read acceleromter data === //
//  Wire.beginTransmission(MPU);
//  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
//  
//  // Calculating Roll and Pitch from the accelerometer data
//  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58)//-1.76 See the calculate_IMU_error()custom function for more details
//  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)//3.16
//
//  
//  // === Read gyroscope data === //
//  previousTime = currentTime;        // Previous time is stored before the actual time read
//  currentTime = micros();            // Current time actual time read
//  elapsedTime = (currentTime - previousTime); // Divide by 1000 to get seconds
//  Wire.beginTransmission(MPU);
//  Wire.write(0x43); // Gyro data first register address 0x43
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
//  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
//  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
//  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
//  // Correct the outputs with the calculated error values
//  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)-4.03
//  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)0.63
//  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)2.1
//  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
//  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
//  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//  yaw =  yaw + GyroZ * elapsedTime;
//  
//  // Complementary filter - combine acceleromter and gyro angle values
//  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
//  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
//  
//  float dt=0.005;
//  float  alpha=1.0/2.0;
//  angle = (1-alpha)*(angle + GyroY * dt) + (alpha)*(accAngleY);
//float prvi,drugi,treci;
//  prvi = digitalRead(SIGA);
//  drugi = digitalRead(SIGB);
//  treci = digitalRead(SIGZ);
//  
// // Serial.print(prvi);
//  //Serial.print(" ");
//  //Serial.println(drugi + 2);
//
//  if(digitalRead(input) > var)
//  {
//  var = 1;
//  pulse++;
//  
////  Serial.print(pulse);
////  Serial.print(F(" pulse"));
//  
//  // Put an "s" if the amount of pulses is greater than 1.
//  if(pulse > 1) {Serial.print(F("s"));}
//  
////  Serial.println(F(" detected."));
//  }
//  
//  if(digitalRead(input) == 0) {var = 0;}
//  
//  delay(1); // Delay for stability.
//  
//  
////  Serial.print(" "); 
////  Serial.println(treci);
//
// // Serial.println(elapsedTime);
//  
//  // Print the values on the serial monitor
// /* Serial.print("Acc: ");
//  Serial.print(AccX);
//  Serial.print("/");
//  Serial.print(AccY);
//  Serial.print("/");
//  Serial.print(AccZ);
//  Serial.print(" | Gyro: ");
//  Serial.print(GyroX);
//  Serial.print("/");
//  Serial.print(GyroY);
//  Serial.print("/");
//  Serial.print(GyroZ);
//  Serial.print(" | Angles: ");
//  Serial.print(roll);
//  Serial.print("/");
//  Serial.print(pitch);
//  Serial.print("/");
//  Serial.println(yaw);
//  */
// // Serial.println(angle);
///*
//  if (state==0){
//    Serial.println("\nReading sensors for first time...");
//    meansensors();
//    state++;
//    delay(1000);
//  }
//
//  if (state==1) {
//    Serial.println("\nCalculating offsets...");
//    calibration();
//    state++;
//    delay(1000);
//  }
//
//  if (state==2) {
//    meansensors();
//    Serial.println("\nFINISHED!");
//    Serial.print("\nSensor readings with offsets:\t");
//    Serial.print(mean_ax); 
//    Serial.print("\t");
//    Serial.print(mean_ay); 
//    Serial.print("\t");
//    Serial.print(mean_az); 
//    Serial.print("\t");
//    Serial.print(mean_gx); 
//    Serial.print("\t");
//    Serial.print(mean_gy); 
//    Serial.print("\t");
//    Serial.println(mean_gz);
//    Serial.print("Your offsets:\t");
//    Serial.print(ax_offset); 
//    Serial.print("\t");
//    Serial.print(ay_offset); 
//    Serial.print("\t");
//    Serial.print(az_offset); 
//    Serial.print("\t");
//    Serial.print(gx_offset); 
//    Serial.print("\t");
//    Serial.print(gy_offset); 
//    Serial.print("\t");
//    Serial.println(gz_offset); 
//    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
//    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
//    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
//    while (1);
//  }
//  */
//}
//void calculate_IMU_error() {
//  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
//  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
//  // Read accelerometer values 200 times
//  while (c < 200) {
//    Wire.beginTransmission(MPU);
//    Wire.write(0x3B);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU, 6, true);
//    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//    // Sum all readings
//    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
//    c++;
//  }
//  //Divide the sum by 200 to get the error value
//  AccErrorX = AccErrorX / 200;
//  AccErrorY = AccErrorY / 200;
//  c = 0;
//  // Read gyro values 200 times
//  while (c < 200) {
//    Wire.beginTransmission(MPU);
//    Wire.write(0x43);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU, 6, true);
//    GyroX = Wire.read() << 8 | Wire.read();
//    GyroY = Wire.read() << 8 | Wire.read();
//    GyroZ = Wire.read() << 8 | Wire.read();
//    // Sum all readings
//    GyroErrorX = GyroErrorX + (GyroX / 131.0);
//    GyroErrorY = GyroErrorY + (GyroY / 131.0);
//    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
//    c++;
//  }
//  //Divide the sum by 200 to get the error value
//  GyroErrorX = GyroErrorX / 200;
//  GyroErrorY = GyroErrorY / 200;
//  GyroErrorZ = GyroErrorZ / 200;
//  // Print the error values on the Serial Monitor
//
//  
//  
////  Serial.print("AccErrorX: ");
////  Serial.println(AccErrorX);
////  Serial.print("AccErrorY: ");
////  Serial.println(AccErrorY);
////  Serial.print("GyroErrorX: ");
////  Serial.println(GyroErrorX);
////  Serial.print("GyroErrorY: ");
////  Serial.println(GyroErrorY);
////  Serial.print("GyroErrorZ: ");
////  Serial.println(GyroErrorZ);
//  
//}
