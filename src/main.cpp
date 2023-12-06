#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9


// #define FAST_MODE

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  Serial.println("Reading events");
  delay(100);
}




struct acc{
  float x;
  float y;
  float z;
}acc;
struct rotVec{
  float R;
  float I;
  float J;
  float K;
}rotVec;
struct grav{
  float x;
  float y;
  float z;
}grav;

void setReport(void){
  if(!bno08x.enableReport(SH2_ACCELEROMETER)){
    Serial.println("Accelerometer not working");
  }
  if(!bno08x.enableReport(SH2_ROTATION_VECTOR)){
    Serial.println("Quaternions not working");
  }
  if(!bno08x.enableReport(SH2_GRAVITY)){
    Serial.println("Gravity not working");
  }
}

void loop() {
  int i = 0;
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    while(bno08x.getSensorEvent(&sensorValue)&&i<3){
        i++;

                if(sensorValue.sensorId==SH2_ACCELEROMETER){
                acc.x=sensorValue.un.accelerometer.x;
                acc.y=sensorValue.un.accelerometer.y;
                acc.z=sensorValue.un.accelerometer.z;
                }
                if(sensorValue.sensorId==SH2_ROTATION_VECTOR){
                rotVec.R = sensorValue.un.rotationVector.real;
                rotVec.I = sensorValue.un.rotationVector.i;
                rotVec.J = sensorValue.un.rotationVector.j;
                rotVec.K = sensorValue.un.rotationVector.k;
                }
                if(sensorValue.sensorId==SH2_GRAVITY){
                grav.x = sensorValue.un.gravity.x;
                grav.y = sensorValue.un.gravity.y;
                grav.z = sensorValue.un.gravity.z;
                }
            
            
    }

  }
  Serial.print("Acceleration"); Serial.print("\t");
  Serial.print("x= "); Serial.print(acc.x); Serial.print("\t");
  Serial.print("y= "); Serial.print(acc.y); Serial.print("\t");
  Serial.print("z= "); Serial.print(acc.z); Serial.print("\n");
  Serial.print("Gravity"); Serial.print("\t");
  Serial.print("x= "); Serial.print(grav.x); Serial.print("\t");
  Serial.print("y= "); Serial.print(grav.y); Serial.print("\t");
  Serial.print("z= "); Serial.print(grav.z); Serial.print("\n");
  Serial.print("Quaternions"); Serial.print("\t");
  Serial.print("r= "); Serial.print(rotVec.R); Serial.print("\t");
  Serial.print("i= "); Serial.print(rotVec.I); Serial.print("\t");
  Serial.print("j= "); Serial.print(rotVec.J); Serial.print("\t");
  Serial.print("k= "); Serial.print(rotVec.K); Serial.print("\n");

}