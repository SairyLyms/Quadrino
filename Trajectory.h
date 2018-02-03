//-------------------------
//Definition
//-------------------------

extern float strPwmOffset;

void SetCalcClothoid(float len,float psi,float phi0,float phi1,float *h,float *phiV,float *phiU, int8_t n);
void CalcCurrentCurvature(float h,float phiV,float phiU,float odo);
Complex slope(float phi0,float phiV, float phiU, float S);
Complex CalcParamClothoid(float phiV,float phiU,int8_t n);
Complex CalcParamClothoidGL(float phiV,float phiU);
float FunctoSolve(float psi,float phi0,float phi1,float phiU,int8_t n);
float Newton(float psi,float phi0,float phi1,float phiU,int8_t n);
float MaxVelocitymps(float curvature,float velLimitMax,float maxAy);
void MaxVelocitympsP(float curvature,float velLimitMax,float maxAy,float *maxVelocity);
void CvMaxMin(float valueStart,float valueEnd,float h,float* posMax,float* cvMax,float* cvMin,float* posMin,float* deltaCv);
void CheckClothoid(float h,float phiV,float phiU,int8_t n);
void SetNextCourseData(int16_t* NextCourseID,float* xNext,float* yNext,float *headNext);
void GetLenAndDirection(float x, float y, float head,float xNext,float yNext,float headNext,float* len,float* psi,float* phi1);
void SetVelocityLimInit(float velLimitMax,float muLim,uint16_t *velLimInitp);

/************************************************************************
 * FUNCTION : クロソイド曲線パラメータ導出・設定関数(コース切替時のみ読み込み)
 * INPUT    : 目標までの直線距離、目標までの角度、現在のヨー角(車体滑り角??)、目標でのヨー角、目標までのコース分割数
 * OUTPUT   : 拡大率(ポインタ)、初期曲率(ポインタ)、縮率(ポインタ)
 ***********************************************************************/
void SetCalcClothoid(float len,float psi,float phi0,float phi1,float *h,float *phiV,float *phiU, int8_t n)
{ 
  Complex clothid;
  float calclambda,psicalc;
  *phiU = Newton(psi,phi0,phi1,M_PI,n);
  *phiV = phi1 - phi0 - *phiU;
  //clothid = CalcParamClothoid(*phiV,*phiU,n);
  clothid = CalcParamClothoidGL(*phiV,*phiU); //ガウス-ルジャンドル法
  *h = len / clothid.modulus();
}

/************************************************************************
 * FUNCTION : クロソイド曲線パラメータ導出・設定関数(コース切替時のみ読み込み)
 * INPUT    : 目標までの直線距離、目標までの角度、現在のヨー角(車体滑り角??)、目標までのヨー角、舵角PWM値、目標までのコース分割数
 * OUTPUT   : 拡大率(ポインタ)、初期曲率(ポインタ)、縮率(ポインタ)
 ***********************************************************************/
 void SetCalcClothoidwStrAngle(float len,float psi,float phi0,float phi1,float strPWM,float *h,float *phiV,float *phiU, int8_t n)
 { 
   Complex clothid;
   *phiV = (strPWM - strPwmOffset) / KStrAngle2PWM / WHEELBase;
   *phiU = phi1 - phi0 - *phiV;
   //clothid = CalcParamClothoid(*phiV,*phiU,n);
   clothid = CalcParamClothoidGL(*phiV,*phiU); //ガウス-ルジャンドル法
   *h = len / clothid.modulus();
 }
 

/************************************************************************
 * FUNCTION : 走行距離に応じたクロソイド曲率導出関数
 * INPUT    : 拡大率、初期曲率、縮率、走行距離
 * OUTPUT   : 曲率(ポインタ)
 ***********************************************************************/
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

/************************************************************************
 * FUNCTION : クロソイド曲線式傾き
 * INPUT    : 初期ヨー角(車体滑り角??)、初期曲率、縮率、正規化距離
 * OUTPUT   : クロソイド曲線式傾き(複素数)
 ***********************************************************************/
Complex Slope(float phi0,float phiV, float phiU, float S)
{
  float phi = phi0 + phiV * S + phiU * S * S;
	Complex I(0,phi);
	return I.c_exp();
}

/************************************************************************
 * FUNCTION : クロソイド曲線積分
 * INPUT    : 初期曲率、縮率、分割数
 * OUTPUT   : クロソイド曲線(複素数)
 ***********************************************************************/
Complex CalcParamClothoid(float phiV,float phiU,int8_t n)
{
  Complex integral = (0,0);// 積分結果
  float w = 1/(float)n;   // 積分範囲を n 個に分割したときの幅
  float S;
  // === Simpson 法による積分 (開始） ===
  S = 0;
  for (int8_t i=0; i<n; i++) {
    integral += (Slope(0, phiV, phiU, S) + Slope(0, phiV, phiU, S+w)) * 0.5 * w;
    S += w;
  }
  return integral;
}

/************************************************************************
 * FUNCTION : クロソイド曲線積分(ガウス-ルジャンドル積分法)
 * INPUT    : 初期曲率、縮率、分割数
 * OUTPUT   : クロソイド曲線(複素数)
 ***********************************************************************/
Complex CalcParamClothoidGL(float phiV,float phiU)
{
  Complex integral = (0,0);// 積分結果
    float xGL[4] = {-0.8611f,-0.34f,0.34f,0.8611f};
    float wGL[4] = {0.3479f,0.6521f,0.6521f,0.3479f};
  // === ガウス-ルジャンドル積分 ===
  for (int8_t i=0; i<4; i++){
      float sGL = 0.5 * xGL[i] + 0.5;
      integral += Slope(0, phiV, phiU, sGL) * wGL[i];
    }
  integral *= 0.5;
  return integral;
}

//デバッグ用
void CheckClothoidwVelLimut(float x0,float y0,float phi0,float h,float phiV,float phiU,float velLimit,float velLimitMax,float ayLimit,int8_t n)
{
  Complex integral = (0,0);// 積分結果
  float w = 1/(float)n;   // 積分範囲を n 個に分割したときの幅
  float S,cv0,cv1,posCvMax,posCvMin,cvMax,cvMin,maxVelocitySector,deltaCv;
  static float lengthTotal,velocity;
  // === Simpson 法による積分 (開始） ===
  S = 0;
  //CvMaxMin(h,phiV,phiU,&posCvMax,&cvMax,&posCvMin,&cvMin,&deltaCv);//区間最大曲率
  for (int8_t i=0; i<n; i++){
    lengthTotal += h/n;
    integral += (Slope(phi0, phiV, phiU, S) + Slope(phi0, phiV, phiU, S+w)) * 0.5 * w;
    CalcCurrentCurvature(h,phiV,phiU,h*(S),&cv0);
    Serial.print(",len,");Serial.print(lengthTotal);
    Serial.print(",x,");Serial.print(x0 + h * integral.modulus()*cos(integral.phase()));
    Serial.print(",y,");Serial.print(y0 + h * integral.modulus()*sin(integral.phase()));
    Serial.print(",velMax,");
    velLimit < MaxVelocitymps(cv0,velLimitMax,ayLimit) ? Serial.print(velLimit) : Serial.print(MaxVelocitymps(cv0,velLimitMax,ayLimit));
    Serial.print(",velMaxBase,");
    Serial.print(velLimit);
    Serial.println("");
    S += w;    
  }
#if 0 //ガウス-ルジャンドル積分の確認(180122 OK)
  Complex integralGL = (0,0);// 積分結果
  integralGL = CalcParamClothoidGL(phiV,phiU);
  Serial.print(",xGL,");Serial.print(x0 + h * integralGL.modulus()*cos(integralGL.phase() + phi0));
  Serial.print(",yGL,");Serial.print(y0 + h * integralGL.modulus()*sin(integralGL.phase() + phi0));
  Serial.println("");
#endif
}

/************************************************************************
 * FUNCTION : ニュートン法被求根関数
 * INPUT    : 目標コース角度、初期ヨー角(車体滑り角??)、目標ヨー角、縮率、分割数
 * OUTPUT   : コース角度差分(計算値 - 目標値)
 ***********************************************************************/
float FunctoSolve(float psi,float phi0,float phi1,float phiU,int8_t n)
{
  float phiV = phi1 - phi0 - phiU;
  //return CalcParamClothoid(phiV,phiU,n).phase() - psi;
  return CalcParamClothoidGL(phiV,phiU).phase() - psi;//ガウス-ルジャンドル法
}

/************************************************************************
 * FUNCTION : ニュートン法求根関数
 * INPUT    : 目標コース角度、初期ヨー角(車体滑り角??)、目標ヨー角、縮率、分割数
 * OUTPUT   : 目標縮率
 ***********************************************************************/
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


/************************************************************************
 * FUNCTION : 最高旋回速度計算
 * INPUT    : 曲率、車両最高速度、最大横加速度
 * OUTPUT   : 最高旋回速度(m/s)
 ***********************************************************************/
float MaxVelocitymps(float curvature,float velLimitMax,float maxAy)
{
  float maxVelocityCorner = sqrtf(maxAy/fabs(curvature));
  if(maxVelocityCorner > velLimitMax){
    maxVelocityCorner = velLimitMax;
  }
  return maxVelocityCorner;
}
//ポインタ返し版
void MaxVelocitympsP(float curvature,float velLimitMax,float maxAy,float *maxVelocity)
{
  *maxVelocity = MaxVelocitymps(curvature,velLimitMax,maxAy);  
}

/************************************************************************
 * FUNCTION : クロソイド曲線区間における最大曲率・距離、最小曲率・距離
 * INPUT    : 拡大率、初期曲率、縮率、走行距離
 * OUTPUT   : 最大曲率距離・曲率、最小曲率距離・曲率、曲率変化率(-1.0〜1.0)
 ***********************************************************************/
void CvMaxMin(float h,float phiV,float phiU,float* posCvMax,float* cvMax,float* posCvMin,float* cvMin,float* deltaCv)
{
  float cvStart = phiV / h;  //開始曲率を返す
  float cvEnd = (phiV + (2 * phiU)) / h;  //終了曲率を返す
  float vdeltaCv = (abs(cvEnd) - abs(cvStart));
  if(((cvStart > 0) - (cvStart < 0)) == ((cvEnd > 0) - (cvEnd < 0)))
  {
    if(vdeltaCv <= 0){
      *cvMax = cvStart;
      *posCvMax = 0.0f;
      *cvMin = cvEnd;
      *posCvMin = 1.0f;
    }
    else{
      *cvMax = cvEnd;
      *posCvMax = 1.0f;
      *cvMin = cvStart;
      *posCvMin = 0.0f;
    }
  }
  else{
    *cvMin = 0.0f;
    *posCvMin = - cvStart / (cvEnd - cvStart);
    *cvMax = max(abs(cvStart),abs(cvEnd));
    *posCvMin > 0.5 ?  *posCvMax = 0.0f : *posCvMax = 1.0f; 
  }
  *deltaCv = vdeltaCv / *cvMax;   //曲率変化率(-1.0〜1.0)
}

/************************************************************************
 * FUNCTION : 目標地点データセット関数
 * INPUT    : 目標地点ID(ポインタ)、目標地点x位置(ポインタ)、目標地点y位置(ポインタ)、目標地点方位(ポインタ)
 * OUTPUT   : 目標地点ID(ポインタ)、目標地点x位置(ポインタ)、目標地点y位置(ポインタ)、目標地点方位(ポインタ)
 ***********************************************************************/
void SetNextCourseData(int16_t* NextCourseID,float* xNext,float* yNext,float *headNext)
{
    *NextCourseID >= lenCourseData-1 ? *NextCourseID = 1 : (*NextCourseID)++;
    *xNext = (float)CourseData[*NextCourseID][0] * courseScale;
    *yNext = (float)CourseData[*NextCourseID][1] * courseScale;
    *headNext = (float)CourseData[*NextCourseID][2] * 0.0001;
}

/************************************************************************
 * FUNCTION : 目標地点までの距離、方位導出関数
 * INPUT    : 現在位置・方位、目標位置・方位
 * OUTPUT   : 目標までの直線距離・角度(ポインタ)、ヨー角(ポインタ)
 ***********************************************************************/
void GetLenAndDirection(float x, float y, float head,float xNext,float yNext,float headNext,float* len,float* psi,float* phi1)
{
  float xVec = xNext - x,yVec = yNext - y;

  *phi1 = headNext-head;
  *len = sqrtf(pow(xVec,2) + pow(yVec,2));
  *psi = Pi2pi(atan2f(yVec,xVec) - head);
  if(*phi1 >= (*psi + M_PI)){     //視野角を0degとして接線角の範囲異常を修正する
    *phi1 -= 2 * M_PI;
  }
  else if(*phi1 < (*psi - M_PI)){ //視野角を0degとして接線角の範囲異常を修正する
    *phi1 += 2 * M_PI;
  }
}

/************************************************************************
 * FUNCTION : コース最高速情報セット関数
 * INPUT    : 車両最高速情報、最大横加速度
 * OUTPUT   : 最高速初期情報配列(m/s)
 ***********************************************************************/
void SetVelocityLimInit(float velLimitMax,float muLim,uint16_t *velLimInitp)
{
    int16_t NextCourseID = 0;
    float cvMax=0,deltaCv=0,cvMaxPrev=0,deltaCvPrev=0;
    float x=0,y=0,head=0,xNext=0,yNext=0,headNext=0;
    for(int j=0;j<2;j++){   //スタート軌跡とゴール軌跡を接続させるため2周分演算する
        volatile int i;     //コンパイラの最適化防止でvolatile付ける
        if(j==0){
            NextCourseID = 1;
            i = 0;
        }
        else{     
            NextCourseID = 2;
            i = 3;
        }
        for(int k = 0;k < lenCourseData;k++){
            float len,psi,phi1,h,phiV,phiU;
            SetNextCourseData(&NextCourseID,&xNext,&yNext,&headNext);
            GetLenAndDirection(x, y, head,xNext,yNext,headNext,&len,&psi,&phi1);
            SetCalcClothoid(len,psi,0.0f,phi1,&h,&phiV,&phiU,5);
            CvMaxMin(h,phiV,phiU,NULL,&cvMax,NULL,NULL,&deltaCv);
            cvMax = abs(cvMax);  //速度の算出なので、曲率の絶対値を使用
            if(cvMaxPrev > cvMax && deltaCv < -0.1){//曲率が前回値より小さく、明らかに曲率が減少している場合
                float velLimbyMu = (0.5 * ((velLimInitp[i-1] * 0.001f) + sqrtf(pow(velLimInitp[i-1] * 0.001f,2) + 4 * muLim * h)));
                velLimbyMu < MaxVelLimCourse ? velLimInitp[i] = velLimbyMu * 1000 : velLimInitp[i] = MaxVelLimCourse * 1000;
            }
            else{
                velLimInitp[i] = MaxVelocitymps(cvMax,velLimitMax,muLim) * 1000;
                i > 1 && ((cvMax - cvMaxPrev) / cvMax) > 0.1 ? velLimInitp[i-1] = MaxVelocitymps(cvMax,velLimitMax,muLim) * 1000 : 0;
                i == 1 && ((cvMax - cvMaxPrev) / cvMax) > 0.1 ? velLimInitp[lenCourseData-1] = MaxVelocitymps(cvMax,velLimitMax,muLim) * 1000 : 0;
            }
            cvMaxPrev = cvMax;deltaCvPrev = deltaCv;
            x = xNext;y = yNext;head = headNext;
            i >= lenCourseData-1 ? i = 1 : i++; 
        }
    }
}

