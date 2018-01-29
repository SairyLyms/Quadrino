#define WHEELBase 0.266f
#define KStrAngle2PWM 280.0f //TT-02Bの場合(171212)
#define StabilityFactor 0.028f

#define LimitAreaRun 20.0f  //走行可能範囲(m)
#define PWMInitCenter 90.0f //PWM初期中点値
#define AyLim 9.8 * 0.5f          //限界横G
#define MaxVelLimCourse 5.5f

#define PUPWMLimUPR 180       //駆動PWM上限値
#define PUPWMLimLWR  0       //駆動PWM下限値
//  Include Header Files
//  -----------------------------------------------------------------------------

#include <Wire.h>           // Include I2C communications
#include <MadgwickAHRS.h>
#include <TaskScheduler.h>
#include <complex.h>
#include <Venus838FLP.h>    // GPS Functions
#include <I2Cdev.h>
#include <MPU6050.h>
#include <complex.h>
#include <Servo.h>
#include "CourseData.h"
#include "GPSFunc.h"
#include "IMUFunc.h"
#include "Trajectory.h"
#include "Control.h"
#include "TxData.h"

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
Servo FStr,PowUnit;

//  Setup Grobal Variables
//  -----------------------------------------------------------------------------
volatile float x=0,y=0,velmps=0,heading=0;
unsigned long timems = millis(),gpsTimems = millis();
float sampletimes,gpsSampletimes;
float yawRt,yawAngle,yawRtGPS;
float strPWM = 90,puPWM = 90;
float headingOffset = 0;
float strPwmOffset = 90;

float x0 = 0.0f,y0 = 0.0f,z0 = 0.0f;
uint8_t sampleTimems = 10;

int ID = 0;
float head,xNext,yNext,headNext;
float h,phiV,phiU,odo;
float maxVel;

int8_t stateMode = 0;

//  Setup Tasks
//  -----------------------------------------------------------------------------
Task T10ms(sampleTimems, TASK_FOREVER, &Task10ms, &runner,true);
Task T50ms(sampleTimems * 5, TASK_FOREVER, &Task50ms, &runner,true);

void setup(){
    Serial.begin(115200);            // Start Serial Port at 115200 baud
    Wire.begin();                   // Initialize the 'Wire' class for the I2C-bus.
    Wire.setClock(400000L);    
    GPSModuleInit();                // Venus838FLP: Startup GPS Module
    GPSConfigureDefaults();         // Venus838FLP: Configure Default Values        
    Llh2Ecef(latlonCenterRad[0],latlonCenterRad[1],heightCenter,&x0,&y0,&z0); //原点座標設定
    GetDirectionPoint2Point(latlonCp,heightCenter,&directionCp);
    accelGyro.initialize();
    SetParamIMU();
    FStr.attach(3,1000,2000);
    PowUnit.attach(5,1000,2000);
    AHRS.begin(100.0f);
#if 0
    Serial.print("heightElp,");Serial.print(heightCenter);
    Serial.print("x0,");Serial.print(x0);
    Serial.print("y0,");Serial.print(y0);
    Serial.print("z0,");Serial.print(z0);
    Serial.print("directionCp,");Serial.println(directionCp);
#endif
    SetVelocityLimInit(MaxVelLimCourse,AyLim,velLimInitp);//初回コース速度制限
#if 0
    for(int i=0;i<lenCourseData;i++){                       //軌道生成デバッグ
      float len,psi,phi1,h,phiV,phiU;
      SetNextCourseData(&ID,&xNext,&yNext,&headNext);
      GetLenAndDirection(x, y, head,xNext,yNext,headNext,&len,&psi,&phi1);
      SetCalcClothoid(len,psi,0.0f,phi1,&h,&phiV,&phiU,5);
      CheckClothoidwVelLimut(x,y,head,h,phiV,phiU,(float)velLimInitp[ID] * 0.001,20.0f,0.5 * 9.8,10);
      x = xNext;y = yNext;head = headNext;
    }
#endif
    runner.startNow();
}

void SetParamIMU(void)
{
  accelGyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  accelGyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  accelGyro.setDLPFMode(MPU6050_DLPF_BW_20);
  accelGyro.setXAccelOffset(1017);
  accelGyro.setYAccelOffset(2057);
  accelGyro.setZAccelOffset(1608);
  accelGyro.setXGyroOffset(-40);
  accelGyro.setYGyroOffset(-5);
  accelGyro.setZGyroOffset(-9);
}

void ReadIMU(float sampletimes,float* yawRt,float* yawAngle)
{
  int16_t aix,aiy,aiz,gix,giy,giz;  
  float afx,afy,afz,gfx,gfy,gfz;
  float YawAngleNow;
  accelGyro.getMotion6(&aix,&aiy,&aiz,&gix,&giy,&giz);
  afx = convertRawAcceleration(aix);
  afy = convertRawAcceleration(aiy);
  afz = convertRawAcceleration(aiz);
  gfx = convertRawGyro(gix);
  gfy = convertRawGyro(giy);
  gfz = convertRawGyro(giz);
  AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz, sampletimes);            //コンパスはモータの磁場の影響をモロに受けるため使用せず
  YawAngleNow = AHRS.getYawRadians();
  *yawRt = Pi2pi(YawAngleNow - *yawAngle) / sampletimes;
  *yawAngle = YawAngleNow;
}


/************************************************************************
 * FUNCTION : サンプリング時間取得
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
void GetSampleTime(unsigned long* timems,float *sampletimes)
{
  unsigned long time = millis();
  if(!timems){
    *sampletimes = sampleTimems * 0.001f;
  }
  else{
    *sampletimes = 0.001f * (time - *timems);
  }
  *timems = time;
}


/************************************************************************
 * FUNCTION : 1周期処理
 * INPUT    : なし
 * OUTPUT   : なし
 ***********************************************************************/
 void Task10ms(void)
 {
  GetSampleTime(&timems,&sampletimes);    //現在時刻とサンプリングタイム取得
  ReadIMU(sampletimes,&yawRt,&yawAngle);  //IMU読み込み
  stateMode = StateManager(x,y,stateMode);
  VehicleMotionControl(stateMode);
  //Serial.print(",stateMode,");Serial.print(stateMode,HEX);Serial.println(",");
#if 0
  Serial.print(",time,");Serial.print(timems);Serial.print(",SampleTime,");Serial.print(sampletimes);
  Serial.print(",YawRt,");Serial.print(yawRt);Serial.print(",yawAg,");Serial.print(yawAngle);
  Serial.print(",E,");Serial.print(e);Serial.print(",N,");Serial.print(n);Serial.print(",U,");Serial.print(u);
  Serial.print(",Vel,");Serial.print(velmps);Serial.print(",Head,");Serial.print(heading);
  Serial.println(",");
#endif
 }
 
 /************************************************************************
  * FUNCTION : 5周期処理
  * INPUT    : なし
  * OUTPUT   : なし
  ***********************************************************************/
 void Task50ms(void)
 {
  TxEncodeVehicleData(stateMode,x,y,head,yawAngle,yawRt,velmps,odo);
  TxEncodeCourseData(ID,xNext,yNext,headNext,phiV,phiU,h);
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
    GetPosXY(x0,y0,z0,latlonCenterRad,directionCp,&x,&y);
    GetGPSVelHeadYawRtwCourseDirection(latlonCenterRad,directionCp,&velmps,&heading,&yawRtGPS,&gpsTimems,&gpsSampletimes);
    //DebugGPS();
  }
}
