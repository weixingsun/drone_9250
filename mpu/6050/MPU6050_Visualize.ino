#include <MadgwickAHRS.h>

#include <IMU_MPU6050.h>

float roll;
float pitch;
float yaw;
long Serialtime;
long last_time;
long microsPerReading;

Madgwick filter;
MPU6050 imu;


void updateIMU(float *roll, float *pitch, float *yaw){

  if (imu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01){    imu.readAccelData(imu.accelCount);
    imu.getAres();

    imu.ax = (float)imu.accelCount[0]*imu.aRes;
    imu.ay = (float)imu.accelCount[1]*imu.aRes;
    imu.az = (float)imu.accelCount[2]*imu.aRes;
    imu.readGyroData(imu.gyroCount);
    imu.getGres();

    imu.gx = (float)imu.gyroCount[0]*imu.gRes;
    imu.gy = (float)imu.gyroCount[1]*imu.gRes;
    imu.gz = (float)imu.gyroCount[2]*imu.gRes;
  }

  imu.updateTime();
  imu.delt_t = millis() - imu.count;
  if (imu.delt_t > microsPerReading){
    filter.updateIMU(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az);
    *roll = filter.getRoll();
    *pitch = filter.getPitch();
    *yaw = filter.getYaw();
    imu.count = millis();
  }
}
void setup() {
  Serial.begin(9600);
  filter.begin(25);
  
  microsPerReading = 1000 / 25;
  Serialtime = 50;
  last_time = millis();
  
  imu.calibrateMPU6050(imu.gyroBias, imu.accelBias);
  imu.initMPU6050();
}

void loop() {
  updateIMU(&roll, &pitch, &yaw);
  if (millis() - last_time > Serialtime) {
    Serial.print("Orientation ");
    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    last_time = millis();
  }
}


