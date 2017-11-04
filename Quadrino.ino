
//  Include Header Files
//  -----------------------------------------------------------------------------

#include <Wire.h>           // Include I2C communications
#include <MadgwickAHRS.h>
#include <TaskScheduler.h>
#include <complex.h>
#include <Venus838FLP.h>    // GPS Functions
#include <I2Cdev.h>
#include <MPU6050.h>
#include "GPSFunc.h"
#include "IMUFunc.h"

// ================================================================
// ===              	Prototype Def.			                      ===
// ================================================================
void Task10ms(void);
void Task20ms(void);

//  Setup Modules
//  -----------------------------------------------------------------------------
MPU6050 accelGyro;
Madgwick AHRS;
Scheduler runner;

//  Setup Grobal Variables
//  -----------------------------------------------------------------------------
float e,n,u,velmps,heading;
float x0 = 0.0f,y0 = 0.0f,z0 = 0.0f;
float latlonCp[2][2] = {{36.567932, 139.995764},{36.567874, 139.995761}};
float heightCenter = 188.0f;
float latlonCenter[2] = {0.5 * (latlonCp[0][0] + latlonCp[1][0]),0.5 * (latlonCp[0][1] + latlonCp[0][1])};

uint8_t sampleTimems = 10;

//  Setup Tasks
//  -----------------------------------------------------------------------------
Task T10ms(sampleTimems, TASK_FOREVER, &Task10ms, &runner,true);
Task T20ms(sampleTimems * 2, TASK_FOREVER, &Task20ms, &runner,true);


void setup(){
    Serial.begin(115200);            // Start Serial Port at 57600 baud
    Wire.begin();                   // Initialize the 'Wire' class for the I2C-bus.
    Wire.setClock(400000L);    
    GPSModuleInit();                // Venus838FLP: Startup GPS Module
    GPSConfigureDefaults();         // Venus838FLP: Configure Default Values        
    Llh2Ecef(latlonCenter[0] * M_PI / 180.0f,latlonCenter[1] * M_PI / 180.0f,heightCenter,&x0,&y0,&z0); //原点座標設定
    accelGyro.initialize();
    SetParamIMU();
    AHRS.begin(100.0f);
    Serial.print("heightElp,");Serial.print(heightCenter);
    Serial.print("x0,");Serial.print(x0);
    Serial.print("y0,");Serial.print(y0);
    Serial.print("z0,");Serial.println(z0);
    runner.startNow();
}

void SetParamIMU(void)
{
  accelGyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelGyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  accelGyro.setXAccelOffset(1149);
  accelGyro.setYAccelOffset(2154);
  accelGyro.setZAccelOffset(1621);
  accelGyro.setXGyroOffset(-59);
  accelGyro.setYGyroOffset(-16);
  accelGyro.setZGyroOffset(-13);
}


void ReadIMU(void)
{
  int16_t aix,aiy,aiz,gix,giy,giz;  
  float afx,afy,afz,gfx,gfy,gfz;
  accelGyro.getMotion6(&aix,&aiy,&aiz,&gix,&giy,&giz);
  afx = convertRawAcceleration(aix);
  afy = convertRawAcceleration(aiy);
  afz = convertRawAcceleration(aiz);
  gfx = convertRawGyro(gix);
  gfy = convertRawGyro(giy);
  gfz = convertRawGyro(giz);
  AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);                  //コンパスはモータの磁場の影響をモロに受けるため使用せず
}

/************************************************************************
 * FUNCTION : 1周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
 void Task10ms(void)
 {
  ReadIMU();
  Serial.print(",time,");Serial.print(millis());
  Serial.print(",E,");Serial.print(e);Serial.print(",N,");Serial.print(n);Serial.print(",U,");Serial.print(u);
  Serial.print(",Vel,");Serial.print(velmps);Serial.print(",Head,");Serial.print(heading);
  Serial.println(",");
 }
 
 /************************************************************************
  * FUNCTION : 2周期処理
  * INPUT    : なし
  * OUTPUT   : なし
  ***********************************************************************/
 void Task20ms(void)
 {

 }

 /************************************************************************
  * FUNCTION : メインループ
  * INPUT    : なし
  * OUTPUT   : なし
  ***********************************************************************/
 void loop ()
 {
   runner.execute();
 }
 
 /************************************************************************
 * FUNCTION : シリアルポート2(GPS)受信割り込み処理
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
void serialEvent2(){
  if(VenusAsyncRead()){
    GetPosENU(&e,&n,&u,x0,y0,z0);//原点からのENU座標  
    GetVelAndHead(&velmps,&heading);//GPS速度と方位の取得(CCW:正)
  }
}
