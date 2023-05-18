#include <Wire.h>

const int MPU_ADDR = 0x68; // MPU-6500 I2C address

void setup() {
  Serial.begin(9600);
  Wire.begin(); // initialize I2C bus
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to 0 to wake up MPU-6500
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request 14 bytes from MPU-6500
  int16_t ax = Wire.read() << 8 | Wire.read(); // accelerometer X-axis
  int16_t ay = Wire.read() << 8 | Wire.read(); // accelerometer Y-axis
  int16_t az = Wire.read() << 8 | Wire.read(); // accelerometer Z-axis
  int16_t gx = Wire.read() << 8 | Wire.read(); // gyroscope X-axis
  int16_t gy = Wire.read() << 8 | Wire.read(); // gyroscope Y-axis
  int16_t gz = Wire.read() << 8 | Wire.read(); // gyroscope Z-axis
  Serial.print("Ax: "); Serial.print(ax);
  Serial.print(" Ay: "); Serial.print(ay);
  Serial.print(" Az: "); Serial.print(az);
  Serial.print(" Gx: "); Serial.print(gx);
  Serial.print(" Gy: "); Serial.print(gy);
  Serial.print(" Gz: "); Serial.println(gz);
  delay(1000); // wait 1 second before reading again
}

// #include "mpu6500.h"

// /* Mpu6500 object */
// bfs::Mpu6500 imu;

// void setup() {
//   /* Serial to display data */
//   Serial.begin(9600);
//   while(!Serial) {}
//   /* Start the I2C bus */
//   Wire.begin();
//   Wire.setClock(400000);
//   /* I2C bus,  0x68 address */
//   imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
//   /* Initialize and configure IMU */
//   if (!imu.Begin()) {
//     Serial.println("Error initializing communication with IMU");
//     while(1) {}
//   }
//   /* Set the sample rate divider */
//   if (!imu.ConfigSrd(19)) {
//     Serial.println("Error configured SRD");
//     while(1) {}
//   }
// }

// void loop() {
//   /* Check if data read */
//   if (imu.Read()) {
//     Serial.print(imu.new_imu_data());
//     Serial.print("\t");
//     Serial.print(imu.accel_x_mps2());
//     Serial.print("\t");
//     Serial.print(imu.accel_y_mps2());
//     Serial.print("\t");
//     Serial.print(imu.accel_z_mps2());
//     Serial.print("\t");
//     Serial.print(imu.gyro_x_radps());
//     Serial.print("\t");
//     Serial.print(imu.gyro_y_radps());
//     Serial.print("\t");
//     Serial.print(imu.gyro_z_radps());
//     Serial.print("\t");
//     Serial.print(imu.die_temp_c());
//     Serial.print("\n");
//     // delay(500);
//   }
// }


