//-------------------------
//Definition
//-------------------------
void CalcClothoid(float len,float psi,float phi0,float phi1,float *h,float *phiV,float *phiU, int8_t n);
float CalcCurvature(float h,float phiV,float phiU,float odo);
float Phi(float phi0, float phiV, float phiU, float S);
Complex slope(float phi0,float phiV, float phiU, float S);
Complex CalcParamClothoid(float phiV,float phiU,int8_t n);
float FunctoSolve(float psi,float phi0,float phi1,float phiU,int8_t n);
float Newton(float psi,float phi0,float phi1,float phiU,int8_t n);

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

//ある地点における曲率を返す
float CalcCurvature(float h,float phiV,float phiU,float odo)
{
  float S = odo / h;
  if(S <= 0){
    return phiV / h;  //開始曲率を返す
  }
  else if(S > 1){
    return (phiV + 2 * phiU) / h;  //終了曲率を返す
  }
  else
  {
    return (phiV + 2 * phiU * S) / h;  //旋回中曲率を返す
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
  float S;
  // === Simpson 法による積分 (開始） ===
  S = 0;
  for (int8_t i=0; i<n; i++) {
    integral += (slope(phi0, phiV, phiU, S) + slope(phi0, phiV, phiU, S+w)) * 0.5 * w;
    S += w;
    Serial.print(",x,");Serial.print(x0 + h * integral.modulus()*cos(integral.phase()));
    Serial.print(",y,");Serial.println(y0 + h * integral.modulus()*sin(integral.phase()));
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