
//  Include Header Files
//  -----------------------------------------------------------------------------

#include <Wire.h>           // Include I2C communications
#include <MadgwickAHRS.h>
#include "Venus838FLP.h"    // GPS Functions
#include "MS5611.h"         // MS5611 Functions
#include "MPU9150.h"        // MPU9150 Functions

//Definition
//-------------------------
#define A				6378137.0		/* Semi-major axis */ 
#define ONE_F			298.257223563   /* 1/F */
#define B				(A*(1.0 - 1.0/ONE_F))
#define E2				((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define NN(lat)			(A/sqrt(1.0 - (E2)*pow(sin(lat),2)))


//  Setup Variables
//  -----------------------------------------------------------------------------
Madgwick AHRS;

String inputString = "";         // Serial Comm: a string to hold incoming data
boolean stringComplete = false;  // Serial Comm: whether the string is complete
int action = 0;                  // Serial Comm: value to hold our currently running action

float x0 = 0.0f,y0 = 0.0f,z0 = 0.0f;
float latlonCp[2][2] = {{36.567932, 139.995764},{36.567874, 139.995761}};
float heightCenter = 188.0f;
float latlonCenter[2] = {0.5 * (latlonCp[0][0] + latlonCp[1][0]),0.5 * (latlonCp[0][1] + latlonCp[0][1])};


void setup(){
    Serial.begin(115200);            // Start Serial Port at 57600 baud
    Wire.begin();                   // Initialize the 'Wire' class for the I2C-bus.
    AHRS.begin(100.0f);
    inputString.reserve(200);       // SerialCommunications: reserve 200 bytes for the inputString:
    GPSModuleInit();                // Venus838FLP: Startup GPS Module
    GPSConfigureDefaults();         // Venus838FLP: Configure Default Values
  
    // Initialize MS5611 sensor
    MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);     // MPU9150: Clear the 'sleep' bit to start the sensor.
    MPU9150_setupCompass();                         // MPU9150: Start the compass
    Llh2Ecef(latlonCenter[0] * M_PI / 180.0f,latlonCenter[1] * M_PI / 180.0f,heightCenter,&x0,&y0,&z0); //原点座標設定
    Serial.print("heightElp,");Serial.print(heightCenter);
    Serial.print("x0,");Serial.print(x0);
    Serial.print("y0,");Serial.print(y0);
    Serial.print("z0,");Serial.println(z0);
}


void Llh2Ecef(float latRad,float lonRad,float height,float* x,float *y,float *z)
{
  *x = (NN(latRad) + height) * cos(latRad) * cos(lonRad);
  *y = (NN(latRad) + height) * cos(latRad) * sin(lonRad);
  *z = (NN(latRad) * (1.0f-E2) + height) * sin(latRad);
}

//原点からのENU座標
void GetPosENU(float* e,float *n,float *u)
{
  float latrad = (float)(venus_ctx.location.latitude/10000000.000000) * M_PI / 180;
  float lonrad = (float)(venus_ctx.location.longitude/10000000.000000) * M_PI / 180;
  
  *e = venus_ctx.location.ecef.x * 0.01 - x0;
  *n = venus_ctx.location.ecef.y * 0.01 - y0;
  *u = venus_ctx.location.ecef.z * 0.01 - z0;

  RotAroudZ(e,n,u,0.5*M_PI);
  RotAroudY(e,n,u,(0.5*M_PI - latrad));
  RotAroudZ(e,n,u,lonrad);
}

//速度と方位(CCW:正)
void GetVelAndHead(float* velmps,float* heading)
{
  float latrad = (float)(venus_ctx.location.latitude/10000000.000000) * M_PI / 180;
  float lonrad = (float)(venus_ctx.location.longitude/10000000.000000) * M_PI / 180;
  
  float ve = venus_ctx.location.vel.x * 0.01;
  float vn = venus_ctx.location.vel.y * 0.01;
  float vu = venus_ctx.location.vel.z * 0.01;

  RotAroudZ(&ve,&vn,&vu,0.5*M_PI);
  RotAroudY(&ve,&vn,&vu,(0.5*M_PI - latrad));
  RotAroudZ(&ve,&vn,&vu,lonrad);

  *velmps = sqrt(pow(ve,2) + pow(vn,2) + pow(vu,2));
  *heading = atan2f(-ve,vn);//CCWを正とする
}


void RotAroudX(float* x,float* y,float* z,float angleRad)
{
  float buf[2] = {};
  buf[0] = *y * cos(angleRad) + *z * sin(angleRad);
  buf[1] = -*y * sin(angleRad) + *z * cos(angleRad);
  *y = buf[0];
  *z = buf[1];
}

void RotAroudY(float* x,float* y,float* z,float angleRad)
{
  float buf[2] = {};
  buf[0] = *x * cos(angleRad) - *z * sin(angleRad);
  buf[1] = *x * sin(angleRad) + *z * cos(angleRad);
  *x = buf[0];
  *z = buf[1];
}

void RotAroudZ(float* x,float* y,float* z,float angleRad)
{
  float buf[2] = {};
  buf[0] = *x * cos(angleRad) + *y * sin(angleRad);
  buf[1] = -*x * sin(angleRad) + *y * cos(angleRad);
  *x = buf[0];
  *y = buf[1];
}

void ReadIMU(void)
{
  float afx,afy,afz;
  float gfx,gfy,gfz;
  float mfx,mfy,mfz;
  afx = convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H));
  afy = convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H));
  afz = convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H));
  gfx = convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H));
  gfy = convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H));
  gfz = convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H));
  mfx = (float)MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H);
  mfy = (float)MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H);
  mfz = (float)MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H);
#if 0
  Serial.print(",Ax,");
  Serial.print(convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H)));
  Serial.print(",Ay,");
  Serial.print(convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H)));
  Serial.print(",Az,");
  Serial.print(convertRawAcceleration(MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H)));
  Serial.print(",Gx,");
  Serial.print(convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H)));
  Serial.print(",Gy,");
  Serial.print(convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H)));
  Serial.print(",Gz,");
  Serial.print(convertRawGyro(MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H)));
  Serial.print("Mx");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H));
  Serial.print("My");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H));
  Serial.print("Mz");
  Serial.println(MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H));
#endif
  AHRS.update(gfx, gfy, gfz, afx, afy, afz, mfy, mfx, mfz);
  //AHRS.updateIMU(gfx, gfy, gfz, afx, afy, afz);
  Serial.print(",yawAngle,");Serial.print(AHRS.getYawRadians()); //ヨー角
}

void DebugGPS(void)
{
  Serial.print(",fix,");Serial.print(venus_ctx.location.fixmode);
  Serial.print(",lon,");Serial.print(venus_ctx.location.longitude);
  Serial.print(",lat,");Serial.print(venus_ctx.location.latitude);
  Serial.print(",height,");Serial.print(venus_ctx.location.ellipsoid_alt * 0.01);
  Serial.print(",ecef.x,");Serial.print((float)venus_ctx.location.ecef.x * 0.01);
  Serial.print(",ecef.y,");Serial.print((float)venus_ctx.location.ecef.y * 0.01);
  Serial.print(",ecef.z,");Serial.print((float)venus_ctx.location.ecef.z * 0.01);
  Serial.print(",spd.x,");Serial.print((float)venus_ctx.location.vel.x * 0.01);
  Serial.print(",spd.y,");Serial.print((float)venus_ctx.location.vel.y * 0.01);
  Serial.print(",spd.z,");Serial.print((float)venus_ctx.location.vel.z * 0.01);

}


void loop ()
{
    float e,n,u,velmps,heading;
    VenusRead(1000); 
    GetPosENU(&e,&n,&u);
    GetVelAndHead(&velmps,&heading);
 #if 0   
    Serial.print(",e,");Serial.print(e);
    Serial.print(",n,");Serial.print(n);
    Serial.print(",u,");Serial.print(u);
    Serial.print(",Vel,");Serial.print(velmps);
    Serial.print(",Head,");Serial.print(heading);
#endif
    ReadIMU();
    //DebugGPS();
    Serial.println(",");
    delay(10);
}


/************************************************************************
 * FUNCTION : IMU加速度生値->物理値変換
 * INPUT    : IMU加速度生値
 * OUTPUT   : 加速度(m/s^2)
 ***********************************************************************/
float convertRawAcceleration(int aRaw)
{
   float a = (aRaw * 2.0 * 9.80665) / 32768.0;
   return a;
}

/************************************************************************
 * FUNCTION : IMU角速度生値->物理値変換
 * INPUT    : IMU角速度生値
 * OUTPUT   : 角速度(deg/s)
 ***********************************************************************/
float convertRawGyro(int gRaw)
{
   float g = (gRaw * 1000.0f) / 32768.0;
   return g;
}