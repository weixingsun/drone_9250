#include <Servo.h>
#include <SoftwareSerial.h>
#include "Kalman.h"
#include "MPU9250.h"
#define ArraySize(x) (sizeof(x) / sizeof(x[0]))
//String toStr(int i) {return String(i);}
//////////////////////////////////////////////////////////////////////////////////
const int WHO_AM_I = 0x73;
const int esc_pins[4] = {5,9,10,11};
const int minPulseRate        = 1000;//700
const int maxPulseRate        = 2000;
const int numSensorDataSize = 10;
const long BT_RATE = 115200;
const int  CMD_LEN = 10;
//////////////////////////////////////////////////////////////////////////////////
SoftwareSerial BLE_Serial(2, 3); // BLE's RX, BLE's TXD
MPU9250 myIMU;
Servo escs[4];
int avg_speed,new_speed,tmp_speed,tmp_speed_total,i;
//////////////////////////////////////////////////////////////////////////////////
// Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RESTRICT_PITCH
Kalman kalmanX;
Kalman kalmanY;
int RIGHT_REAR=0, RIGHT_FRONT=1, LEFT_FRONT=2, LEFT_REAR=3;
uint32_t timer;
double delta;
String cmd="";
bool cmd_end=false;
//////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  Serial.begin(BT_RATE);
  BLE_Serial.begin(BT_RATE);
  initEscs();
  initMPU9250();
  //writeTo4Escs(33);
}
void loop() {
  readCmd();
  getDataMPU9250();
  //printMPU9250();
  //computeKalman();
  //printKalman();
  autoBalance();
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
      new_speed=cmd.toInt();
      //Serial.println("set avg speed to: "+cmd+" avg="+String(avg_speed));
      if(new_speed<50){
        avg_speed=changeAllSpeed(new_speed-avg_speed);
      }
    }
  }
  if(cmd_end){
    cmd="";
    cmd_end=false;
  }
}
int changeAllSpeed(int step){
  tmp_speed_total=0;
  for (i=0;i<ArraySize(escs);i++){
    tmp_speed=escs[i].read();
    tmp_speed+=step;
    escs[i].write(tmp_speed);
    tmp_speed_total+=escs[i].read();
    Serial.print("change speed:"+tmp_speed);
  }
  return tmp_speed_total/ArraySize(escs);
}

int changeASpeed(int id, int step){
  tmp_speed=escs[id].read();
  tmp_speed+=step;
  escs[id].write(tmp_speed);
  tmp_speed_total+=step;
  return tmp_speed_total/ArraySize(escs);
}
void initMPU9250(){
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  //Serial.print("WHO_AM_I="+String(WHO_AM_I)+ "  c="+String(c));
  if (c == WHO_AM_I){
    myIMU.MPU9250SelfTest(myIMU.selfTest);
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
void getDataMPU9250(){
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
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
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
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
        //myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        //myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
  }
}
void autoBalance(){
    Serial.print("myIMU.roll=");Serial.print(myIMU.roll);Serial.print('\t');
    Serial.print("myIMU.pitch=");Serial.println(myIMU.pitch);
    if(myIMU.roll<-5 && myIMU.gx<-1){             //left down
       changeASpeed(LEFT_FRONT,1);
       changeASpeed(LEFT_REAR, 1);
       changeASpeed(RIGHT_FRONT,-1);
       changeASpeed(RIGHT_REAR, -1);
       Serial.println("inc esc left, dec esc right ");
    }else if(myIMU.roll>5 && myIMU.gx>1) {        //left up
       changeASpeed(LEFT_FRONT,-1);
       changeASpeed(LEFT_REAR, -1);
       changeASpeed(RIGHT_FRONT,1);
       changeASpeed(RIGHT_REAR, 1);
       Serial.println("dec esc left, inc esc right ");
    }
    if(myIMU.pitch>5 && myIMU.gy>1) {            //head down
       changeASpeed(LEFT_FRONT, 1);
       changeASpeed(RIGHT_FRONT,1);
       changeASpeed(LEFT_REAR, -1);
       changeASpeed(RIGHT_REAR,-1);
       Serial.println("dec esc rear, inc esc head ");
    }else if(myIMU.roll<-5 && myIMU.gx<-1) {     //head up
       changeASpeed(LEFT_FRONT, -1);
       changeASpeed(RIGHT_FRONT,-1);
       changeASpeed(LEFT_REAR,   1);
       changeASpeed(RIGHT_REAR,  1);
       Serial.println("inc esc rear, dec esc head ");
    }
}
void printMPU9250(){
  myIMU.dspDelt_t = millis() - myIMU.count;
  if (myIMU.dspDelt_t > 500) {  //slow down printing rate
    Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
    Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
    Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
    Serial.println(" mG-force");
    Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
    Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
    Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
    Serial.println(" deg/s");
    Serial.print("mx = ");  Serial.print((int)myIMU.mx);
    Serial.print(" my = "); Serial.print((int)myIMU.my);
    Serial.print(" mz = "); Serial.print((int)myIMU.mz);
    Serial.println(" mGauss");
    /*Serial.print("q0 = ");  Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));
    //Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
    //Serial.println(" degrees C");
    */
    myIMU.count = millis();
  }
}
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
/*
void initKalman(){
  // Set kalman and gyro starting angle
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}
*/
