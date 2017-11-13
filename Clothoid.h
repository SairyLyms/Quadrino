//-------------------------
//Definition
//-------------------------
void CalcClothoid(float len,float psi,float phi0,float phi1,float *h,float *phiV,float *phiU, int8_t n);
void CalcCurrentCurvature(float h,float phiV,float phiU,float odo);
float Phi(float phi0, float phiV, float phiU, float S);
Complex slope(float phi0,float phiV, float phiU, float S);
Complex CalcParamClothoid(float phiV,float phiU,int8_t n);
float FunctoSolve(float psi,float phi0,float phi1,float phiU,int8_t n);
float Newton(float psi,float phi0,float phi1,float phiU,int8_t n);
float MaxVelocitymps(float curvature,float maxAy);
void CvMaxMin(float valueStart,float valueEnd,float h,float* posMax,float* cvMax,float* cvMin,float* posMin);
float AccelBasedCv(float cv0,float cv1);
float VelocityCv(float velCur,float velLim,float axLim);


//デバッグ用
void CheckClothoid(float h,float phiV,float phiU,int8_t n);
/************************************************************************
 * FUNCTION : クロソイド曲線用関数群(20msほど必要)
 * INPUT    :
 * OUTPUT   :
 ***********************************************************************/
//クロソイド曲線軌道導出
void CalcClothoid(float len,float psi,float phi0,float phi1,float *h,float *phiV,float *phiU, int8_t n)
{ 
  Complex clothid;
  float calclambda,psicalc;
  *phiU = Newton(psi,phi0,phi1,M_PI,n);
  *phiV = phi1 - phi0 - *phiU;
  clothid = CalcParamClothoid(*phiV,*phiU,n);
  *h = len / clothid.modulus();
}

//現地点における曲率を返す
void CalcCurrentCurvature(float h,float phiV,float phiU,float odo,float *cvCul)
{
  float S = odo / h;
  if(S <= 0){
    *cvCul = phiV / h;  //開始曲率を返す
  }
  else if(S > 1){
    *cvCul = (phiV + (2 * phiU)) / h;  //終了曲率を返す
  }
  else
  {
    *cvCul = (phiV + (2 * phiU * S)) / h;  //旋回中曲率を返す
  }
}

//指数部
float Phi(float phi0, float phiV, float phiU, float S)
{
	return phi0 + phiV * S + phiU * S * S;
}

//傾き
Complex slope(float phi0,float phiV, float phiU, float S)
{
	Complex I(0,Phi(phi0, phiV, phiU, S));
	return I.c_exp();
}

//クロソイドパラメータ計算
Complex CalcParamClothoid(float phiV,float phiU,int8_t n)
{
  Complex integral = (0,0);// 積分結果
  float w = 1/(float)n;   // 積分範囲を n 個に分割したときの幅
  float S;
  // === Simpson 法による積分 (開始） ===
  S = 0;
  for (int8_t i=0; i<n; i++) {
    integral += (slope(0, phiV, phiU, S) + slope(0, phiV, phiU, S+w)) * 0.5 * w;
    S += w;
  }
  return integral;
}

//デバッグ用
void CheckClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,int8_t n)
{
  Complex integral = (0,0);// 積分結果
  float w = 1/(float)n;   // 積分範囲を n 個に分割したときの幅
  float S,cv0,cv1,posCvMax,posCvMin,cvMax,cvMin,maxVelocitySector;
  static float lengthTotal,velocity;
  // === Simpson 法による積分 (開始） ===
  S = 0;
  CvMaxMin(h,phiV,phiU,&posCvMax,&cvMax,&posCvMin,&cvMin);//区間最大曲率
  for (int8_t i=0; i<n; i++){
    lengthTotal += h/n;
    integral += (slope(phi0, phiV, phiU, S) + slope(phi0, phiV, phiU, S+w)) * 0.5 * w;
    CalcCurrentCurvature(h,phiV,phiU,h*(S),&cv0);
    CalcCurrentCurvature(h,phiV,phiU,h*(S+w),&cv1);
    Serial.print(",len,");Serial.print(lengthTotal);
    Serial.print(",x,");Serial.print(x0 + h * integral.modulus()*cos(integral.phase()));
    Serial.print(",y,");Serial.print(y0 + h * integral.modulus()*sin(integral.phase()));
    Serial.print(",PosCvMax,");Serial.print(posCvMax);Serial.print(",CvMax,");Serial.print(cvMax);
    Serial.print(",PosCvMin,");Serial.print(posCvMin);Serial.print(",CvMin,");Serial.print(cvMin);
    Serial.print(",velMax,");Serial.print(MaxVelocitymps(cv0,0.3 * 9.8));
    Serial.print(",velMaxSector,");Serial.print(MaxVelocitymps(cvMax,0.3 * 9.8));
    Serial.println("");
    S += w;    
  }
}

//求根用関数
float FunctoSolve(float psi,float phi0,float phi1,float phiU,int8_t n)
{
  float phiV = phi1 - phi0 - phiU;
  return CalcParamClothoid(phiV,phiU,n).phase() - psi;
}

//求根関数
float Newton(float psi,float phi0,float phi1,float phiU,int8_t n)  /* 初期値 $x$ から $f(x) = 0$ の解を求める */
{
    int i;
    float h, fx, df, phiU_prev, fx_prev;
    fx = FunctoSolve(psi,phi0,phi1,phiU,n);
    while (fabs(fx) >= 0.01f) {
        i = 0;
        df = FunctoSolve(psi,phi0,phi1,(phiU + fx),n) - fx;
        h = fx * fx / df;
        phiU_prev = phiU;
        fx_prev = fx;
        do{
          phiU = phiU_prev - h;
          fx = FunctoSolve(psi,phi0,phi1,phiU,n);
          h /= 2;
          i++;
          } while (fabs(fx) > fabs(fx_prev));
        if (phiU == phiU_prev && i == 1 ) break;
    }
    return phiU;
}

//ある横G以下の旋回限界速度
float MaxVelocitymps(float curvature,float maxAy)
{
  float maxVelocityVehicle = 20.0f;
  float maxVelocityCorner = sqrtf(maxAy/fabs(curvature));
  if(maxVelocityCorner > maxVelocityVehicle){
    maxVelocityCorner = maxVelocityVehicle;
  }
  return maxVelocityCorner;
}

//クロソイド曲線区間における最大曲率と最小曲率、それぞれの区間開始からの距離(旋回速度算出用)
void CvMaxMin(float h,float phiV,float phiU,float* posCvMax,float* cvMax,float* posCvMin,float* cvMin)
{
  float cvStart = phiV / h;  //開始曲率を返す
  float cvEnd = (phiV + (2 * phiU)) / h;  //終了曲率を返す

  if(abs(cvStart) - abs(cvEnd) > 0){
    *cvMax = cvStart;
    *posCvMax = 0.0f;
  }
  else{
    *cvMax = cvEnd;
    *posCvMax = 1.0f;
  }
  if(((cvStart > 0) - (cvStart < 0)) == ((cvEnd > 0) - (cvEnd < 0)))
  {
    if(abs(cvStart) - abs(cvEnd) < 0){
      *cvMin = cvStart;
      *posCvMin = 0.0f;
    }
    else{
      *cvMin = cvEnd;
      *posCvMin = 1.0f;
    }
  }
  else{
    *cvMin = 0;
    *posCvMin = - cvStart / (cvEnd - cvStart);
  }
}

//加減速判定　重要なのはクロソイドセグメントの「距離に対する曲率の傾き」から加速すべきか減速すべきか判定する
float AccelBasedCv(float cv0,float cv1)
{
 float limAxAcc = 0.3,limAxDec = -0.5;
 float ax = MaxVelocitymps(cv1,0.8 * 9.8) - MaxVelocitymps(cv0,0.8 * 9.8);
 if(ax > limAxAcc){
  ax = limAxAcc;
 }
 else if(ax < limAxDec){
  ax = limAxDec;
 }
 return ax;
}

//速度
float VelocityCv(float velCur,float velLim,float axLim)
{
  float velocity = velCur + axLim * 0.001;
  velocity > velLim ? velocity = velLim : 0;
  return velocity;
}