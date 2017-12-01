//-------------------------
//Definition
//-------------------------

#define A				6378137.0		/* Semi-major axis */ 
#define ONE_F			298.257223563   /* 1/F */
#define B				(A*(1.0 - 1.0/ONE_F))
#define E2				((1.0/ONE_F)*(2-(1.0/ONE_F)))
#define NN(lat)			(A/sqrt(1.0 - (E2)*pow(sin(lat),2)))

void Llh2Ecef(float latRad,float lonRad,float height,float *x,float *y,float *z);
void GetDirectionPoint2Point(float latLon[2][2],float height,float *directionP2P);
void GetPosENU(float* e,float *n,float *u ,float x0, float y0, float z0);
void GetVelAndHead(float* velmps,float* heading);
void RotAroudX(float* x,float* y,float* z,float angleRad);
void RotAroudY(float* x,float* y,float* z,float angleRad);
void RotAroudZ(float* x,float* y,float* z,float angleRad);

//緯度経度からECEF座標への変換(コース情報生成用)
void Llh2Ecef(float latRad,float lonRad,float height,float *x,float *y,float *z)
{
  *x = (NN(latRad) + height) * cos(latRad) * cos(lonRad);
  *y = (NN(latRad) + height) * cos(latRad) * sin(lonRad);
  *z = (NN(latRad) * (1.0f-E2) + height) * sin(latRad);
}

//2点間の方位取得(コース情報生成用)
//要デバッグ
void GetDirectionPoint2Point(float latLon[2][2],float height,float *directionP2P)
{
  float x[2] = {},y[2] = {},z[2] = {};
  for(int8_t i = 0;i<2;i++){
    Serial.print(",lat,");Serial.print(latLon[i][0],8);Serial.print(",lon,");Serial.println(latLon[i][1],8);    
    Llh2Ecef(latLon[i][0] * M_PI / 180,latLon[i][1] * M_PI / 180,height,&x[i],&y[i],&z[i]);
  }
  x[1] -= x[0];
  y[1] -= y[0];
  z[1] -= z[0];
  RotAroudZ(&x[1],&y[1],&z[1],latLon[1][1] * M_PI / 180);
  RotAroudY(&x[1],&y[1],&z[1],(0.5*M_PI - latLon[1][0] * M_PI / 180));  
  RotAroudZ(&x[1],&y[1],&z[1],0.5*M_PI);
  Serial.print(",x,");Serial.print(x[1]);Serial.print(",y,");Serial.println(y[1]);      
  *directionP2P = atan2f(x[1],-y[1]);
}

//原点からのENU座標
void GetPosENU(float *e,float *n,float *u ,float x0, float y0, float z0)
{
  float latrad = (float)(venus_ctx.location.latitude/10000000.000000) * M_PI / 180;
  float lonrad = (float)(venus_ctx.location.longitude/10000000.000000) * M_PI / 180;
  
  *e = venus_ctx.location.ecef.x * 0.01 - x0;
  *n = venus_ctx.location.ecef.y * 0.01 - y0;
  *u = venus_ctx.location.ecef.z * 0.01 - z0;

  RotAroudZ(e,n,u,lonrad);
  RotAroudY(e,n,u,(0.5*M_PI - latrad));
  RotAroudZ(e,n,u,0.5*M_PI);
}

//GPS速度と方位の取得(CCW:正)
void GetVelAndHead(float* velmps,float* heading)
{
  float latrad = (float)(venus_ctx.location.latitude/10000000.000000) * M_PI / 180;
  float lonrad = (float)(venus_ctx.location.longitude/10000000.000000) * M_PI / 180;
  
  float ve = venus_ctx.location.vel.x * 0.01;
  float vn = venus_ctx.location.vel.y * 0.01;
  float vu = venus_ctx.location.vel.z * 0.01;

  RotAroudZ(&ve,&vn,&vu,lonrad);
  RotAroudY(&ve,&vn,&vu,(0.5*M_PI - latrad));  
  RotAroudZ(&ve,&vn,&vu,0.5*M_PI);

  *velmps = sqrt(pow(ve,2) + pow(vn,2) + pow(vu,2));
  *heading = atan2f(-ve,vn);//CCWを正とする
}

//GPS情報の表示
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

