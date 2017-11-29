#include <Servo.h>
#include <SoftwareSerial.h>
#include "Kalman.h"
#define ArraySize(x) (sizeof(x) / sizeof(x[0]))
//String toStr(int i) {return String(i);}
//////////////////////////////////////////////////////////////////////////////////

int ACT_SS = 0;  //no change
int ACT_YC = 1;  //yaw clockwise
int ACT_YA = 2;  //yaw anti-clockwise
int ACT_PC = 3;  //pitch clockwise
int ACT_PA = 4;  //pitch anti-clockwise
int ACT_RC = 5;  //roll clockwise
int ACT_RA = 6;  //roll anti-clockwise
int ACT_UP = 7;  //take-off
int ACT_DN = 8;  //land

int TEN = 10;   //1  //FL
int NIN = 9;    //2  //FR
int FIV = 5;    //3  //RR
int ELE = 11;   //4  //RL
const int minPulseRate = 1000;
const int maxPulseRate = 1800;
Servo escs[4];
const int esc_pins[4] = {TEN,NIN,FIV,ELE};
int esc_ms[4] = {minPulseRate,minPulseRate,minPulseRate,minPulseRate};
// YC -> 1,3 (+) 2,4 (-)
// YA -> 2,4 (+) 1,3 (-)
// PC -> 1,2 (+) 3,4 (-)
// PA -> 1,2 (-) 3,4 (+)
// RC -> 1,4 (+) 2,3 (-)
// RA -> 1,4 (-) 2,3 (+)
void changeMotorPulse(int idx,int delta){
  esc_ms[idx]+=delta;
  escs[idx].writeMicroseconds(esc_ms[idx]);
}
int Yaw(int delta){
  changeMotorPulse(0, delta);
  changeMotorPulse(2, delta);
  changeMotorPulse(1, -delta);
  changeMotorPulse(3, -delta);
}
int Pitch(int delta){
  changeMotorPulse(0, delta);
  changeMotorPulse(1, delta);
  changeMotorPulse(2, -delta);
  changeMotorPulse(3, -delta);
}
int Roll(int delta){
  changeMotorPulse(0, delta);
  changeMotorPulse(3, delta);
  changeMotorPulse(1, -delta);
  changeMotorPulse(2, -delta);
}

const int WHO_AM_I = 0x73;
const int numSensorDataSize = 10;
const long BT_RATE = 115200;
const int  CMD_LEN = 10;

//////////////////////////////////////////////////////////////////////////////////
SoftwareSerial BLE_Serial(2, 3); // BLE's RX, BLE's TXD
//#define MPU_6050
#define MPU_9250
#ifdef MPU_9250
  #include "MPU9250.h"
  MPU9250 myIMU;
#else
  #include "MPU6050.h"
  MPU6050 myIMU;
#endif
int current_speed,i;
//////////////////////////////////////////////////////////////////////////////////
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
double delta;
//////////////////////////////////////////////////////////////////////////////////
bool cmd_end=false;
String cmd="";
//////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(BT_RATE);
  BLE_Serial.begin(BT_RATE);
  initEscs();
  initMPU();
  //if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  //MPU6050_ADDRESS
  calcIMU();
  //}
  //initKalman();
  changeAllSpeed(1000);
}
void loop() {
  readCmd();
  //if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  //MPU6050_ADDRESS
  calcIMU();
  //}
  //computeKalman();
  //printKalman();
  //printMPU();
  changeAllSpeed(current_speed);
  printSpeeds();
  delay(200);
}
void readCmd(){
  byte byte_count=BLE_Serial.available();
  if(byte_count>0){
    for(int i=0;i<byte_count;i++){
      char s=BLE_Serial.read();
      if(s==';') cmd_end=true;
      else cmd+=s;
    }
    if (cmd_end==true){
      current_speed=cmd.toInt();
      //Serial.println("set avg speed to: "+cmd+" avg="+String(avg_speed));
      if(current_speed>1000){
        changeAllSpeed(current_speed);
      }
    }
  }
  if(cmd_end){
    cmd="";
    cmd_end=false;
  }
}
int changeAllSpeed(int throttle){
    if (throttle > maxPulseRate) throttle = maxPulseRate;        //We need some room to keep full control at full throttle.
    //int esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    escs[3].writeMicroseconds(throttle);
    //int esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    escs[2].writeMicroseconds(throttle);
    //int esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    escs[1].writeMicroseconds(throttle);
    //int esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    escs[0].writeMicroseconds(throttle);
}
void initMPU(){
#ifndef MPU_9250
  byte c = myIMU.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
  //Serial.print("WHO_AM_I="+String(WHO_AM_I)+ "  c="+String(c));
  if (c == WHO_AM_I){
    //myIMU.MPU6050SelfTest(myIMU.selfTest);
    myIMU.calibrateMPU6050(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU6050();
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (d != 0x48) Error(d," WHO_AM_I_AK8963 should be 0x48");
    //myIMU.initAK8963(myIMU.factoryMagCalibration);
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    //myIMU.initAK8963(myIMU.factoryMagCalibration);
    delay(2000);
    Serial.print("MPU6050 ready");
  }else{
    Error(c,"Could not connect to MPU6050: 0x");
  }
#else
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  //Serial.print("WHO_AM_I="+String(WHO_AM_I)+ "  c="+String(c));
  if (c == WHO_AM_I){
    //myIMU.MPU9250SelfTest(myIMU.selfTest);
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (d != 0x48) Error(d," WHO_AM_I_AK8963 should be 0x48");
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    delay(2000);
    Serial.print("MPU9250 ready");
  }else{
    Error(c,"Could not connect to MPU9250: 0x");
  }
}
#endif
void calcIMU(){
  myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
  // Now we'll calculate the accleration value into actual g's This depends on scale being set
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
  myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
  myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
  myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  // Get actual magnetometer value, this depends on scale being set
  //myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
  //myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
  //myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  // Must be called before updating quaternions!
  myIMU.updateTime();
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    myIMU.roll   = atan2(myIMU.ay, myIMU.az) * RAD_TO_DEG;
    myIMU.pitch  = atan(-myIMU.ax / sqrt(myIMU.ay * myIMU.ay + myIMU.az * myIMU.az)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    myIMU.roll   = atan(myIMU.ay / sqrt(myIMU.ax * myIMU.ax + myIMU.az * myIMU.az)) * RAD_TO_DEG;
    myIMU.pitch  = atan2(-myIMU.ax, myIMU.az) * RAD_TO_DEG;
  #endif
  /*MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                              * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                      * *(getQ() + 3));
  myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                              * *(getQ() + 2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                              * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                      * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                      * *(getQ() + 3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
  myIMU.yaw  -= 8.36; //Buenos aires 2017
  myIMU.roll *= RAD_TO_DEG;
   */
}
/*void autoBalance(){
    int delta_r = (int)(SPEED_RATIO_R*myIMU.roll);
    int delta_p = (int)(SPEED_RATIO_P*myIMU.pitch);
       changeASpeed(FIV, delta_r);
       changeASpeed(TEN,-delta_r);
       changeASpeed(NIN,-delta);
       changeASpeed(ELE, delta);
}*/

void initEscs() {
  for (i=0;i<ArraySize(escs);i++){
    //Serial.println("init escs:"+String(i));
    escs[i].attach(esc_pins[i], minPulseRate, maxPulseRate);
  }
  delay(20);
  for (i=0;i<ArraySize(escs);i++){
    escs[i].write(0);
  }
  delay(3000);
}
// Ensure the throttle value is between 0 - 180
int normalizeThrottle(int value) {
  if(value < 0) {
    return 0;
  } else if(value > 180) {
    return 180;
  }
  return value;
}
int Error(int code, String msg){
  Serial.print(msg);
  Serial.println(code, HEX);
  //Serial.println(F("Communication failed, abort!"));
  Serial.flush();
  abort();
}
//////////////////////////////////////////////////////////////////////////////////

void initKalman(){
  kalmanX.setAngle(myIMU.roll); // Set starting angle
  kalmanY.setAngle(myIMU.pitch);
  gyroXangle = myIMU.roll;
  gyroYangle = myIMU.pitch;
  compAngleX = myIMU.roll;
  compAngleY = myIMU.pitch;
  timer = micros();
}

void computeKalman(){
  delta = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double gyroXrate = myIMU.gx / 131.0; // Convert to deg/s
  double gyroYrate = myIMU.gy / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((myIMU.roll < -90 && kalAngleX > 90) || (myIMU.roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(myIMU.roll);
    compAngleX = myIMU.roll;
    kalAngleX  = myIMU.roll;
    gyroXangle = myIMU.roll;
  } else {
    kalAngleX = kalmanX.getAngle(myIMU.roll, gyroXrate, delta); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(myIMU.pitch, gyroYrate, delta);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((myIMU.pitch < -90 && kalAngleY > 90) || (myIMU.pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(myIMU.pitch);
    compAngleY = myIMU.pitch;
    kalAngleY = myIMU.pitch;
    gyroYangle = myIMU.pitch;
  } else {
    kalAngleY = kalmanY.getAngle(myIMU.pitch, gyroYrate, delta); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(myIMU.roll, gyroXrate, delta); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * delta; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * delta;
  //gyroXangle += kalmanX.getRate() * delta; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * delta;

  compAngleX = 0.93 * (compAngleX + gyroXrate * delta) + 0.07 * myIMU.roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * delta) + 0.07 * myIMU.pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

}
/*void printKalman(){
  Serial.print("roll="); Serial.print(myIMU.roll); Serial.print("\t");
  Serial.print("  gx=");Serial.print(myIMU.gx); Serial.print("\t");
  Serial.print("gxagl=");Serial.print(gyroXangle); Serial.print("\t");
  Serial.print("caglx=");Serial.print(compAngleX); Serial.print("\t");
  Serial.print("kaglx=");Serial.print(kalAngleX); Serial.print("\t");
  Serial.print("pitch=");Serial.print(myIMU.pitch); Serial.print("\t");
  Serial.print("  gy=");Serial.print(myIMU.gy); Serial.print("\t");
  Serial.print("gyagl=");Serial.print(gyroYangle); Serial.print("\t");
  Serial.print("cagly=");Serial.print(compAngleY); Serial.print("\t");
  Serial.print("kagly=");Serial.print(kalAngleY); Serial.println("\t");
}
void printMPU(){
  myIMU.dspDelt_t = millis() - myIMU.count;
  if (myIMU.dspDelt_t > 1000) {  //slow down printing rate
    Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
    Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
    Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
    Serial.println(" mG-force");
    Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
    Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
    Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
    Serial.println(" deg/s");
    myIMU.count = millis();
  }
}*/

void printSpeeds(){
  myIMU.dspDelt_t = millis() - myIMU.count;
  if (myIMU.dspDelt_t > 500) {  //slow down printing rate
    int tmp_speed=escs[0].readMicroseconds(); //FIV
    Serial.print(" FIV= ");
    Serial.print(tmp_speed);
    Serial.print(" NIN= ");
     tmp_speed=escs[1].readMicroseconds(); //NIN
    Serial.print(tmp_speed);
    Serial.print(" TEN= ");
     tmp_speed=escs[2].readMicroseconds(); //TEN
    Serial.print(tmp_speed);
    Serial.print(" ELE= ");
     tmp_speed=escs[3].readMicroseconds(); //ELE
    Serial.print(tmp_speed);
    Serial.println();
    myIMU.count = millis();
  }
}

