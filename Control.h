extern unsigned long timems;
extern volatile float x,y,heading,velmps;
extern float yawAngle,yawRt,yawRtGPS,sampletimes,headingOffset,strPwmOffset,strPWM,puPWM,gpsSampletimes;
extern Servo FStr,PowUnit;
extern int16_t ID;
extern float head,xNext,yNext,headNext;
extern float h,phiV,phiU,odo;
extern float maxVel;
extern int8_t stateMode;
int8_t StateManager(float x,float y,int8_t stateMode);
int8_t SMLimitArea(float x,float y,int8_t stateMode);
int8_t SMHeadCalib(int8_t stateMode);
int8_t SMChangeRunState(int8_t stateMode);
int8_t SMChangeStopState(int8_t stateMode);
void VehicleMotionControl(int8_t stateMode);
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float hading,float *headingOffsetIMU,float *strPwmOffset,float *strPWM,float *puPWM);
void VMCRunNorm(float *strPWM,float *puPWM);
void VMCStop(float *strPWM,float *puPWM);
void SelectHeadingInfo(float puPWM,float velocityMps,float yawRtGPS,float headingGPS,float yawRt,float yawAngle,float sampletimes,float *headingOffsetIMU,float *headingOut);
float StrControlFF(float cvCul,float strPwmOffset);
float StrControlFFwSF(float cvCul,float strPwmOffset,float velmps);
float StrControlFFFB(int16_t ID,float cvCul,float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset,float velmps);
float StrControlPID(float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset);
float SpdControlPID(int16_t ID,float currentSpeed,float targetSpeed,float sampleTime);
void TxInfo(void);
void splitData(void *dataToSplit,uint8_t dataLen,uint8_t *splitDataPt);


int8_t StateManager(float x,float y,int8_t stateMode){
    int8_t key = 0;
    stateMode = SMLimitArea(x,y,stateMode);
    if(x && y && stateMode & 0x01){                                        //x,yが0でなく、走行可能範囲内(stateModeの0ビット目 == 1)の場合
        switch(stateMode){
            case 0x01:  //Serial.print("Calib : 'c' ,");
                        if(Serial.available() && Serial.read() == 'c'){
                            stateMode = SMHeadCalib(stateMode);
                        }
                        break;
            case 0x03:
            case 0x05:  
            case 0x07:  stateMode = SMHeadCalib(stateMode);
                        break;
            case 0x09:  //Serial.print("Start : 'r' ,");
                        if(Serial.available() && Serial.read() == 'r'){
                            stateMode = SMChangeRunState(stateMode);    //走行開始
                        }
                        break;
            case 0x19:  key = Serial.read();
                        if(key > 0 && key !=0x0A && key != 0x0D){
                            stateMode = SMChangeStopState(stateMode);    //走行終了
                        }
                        break;
            default:    break;
        }
    }
    else{
        stateMode = SMChangeStopState(stateMode);
        //Serial.print(",Out of course,");                      //走行可能範囲外
    }
    return stateMode;
}

/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    : 原点からの位置座標(E,N)
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t SMLimitArea(float x,float y,int8_t stateMode)
{
    float rfromCenter = sqrt(pow(x,2) + pow(y,2));
    if(rfromCenter < LimitAreaRun){//範囲内(ビット0を立てる)
        stateMode |= 0x01;
    }
    else{       //範囲外
        stateMode &= ~0x01;        //範囲外(ビット0下げる)  
    }
    return stateMode;
}

int8_t SMHeadCalib(int8_t stateMode)
{
    static uint16_t calibTime = 0;
    if(calibTime < 200){        //200サイクルまではキャリブ実施せず
        stateMode >> 1 == 0x00 ? stateMode = (stateMode &= ~0x0E) |= 0x01 << 1 : 0;      //方位キャリブレーション事前走行中(0x03) ビットマスクして0x01<<1を書き込み
        calibTime++;
    }
    else if(calibTime < 500){   //200サイクル超でキャリブ実施
        stateMode >> 1 == 0x01 ? stateMode = (stateMode &= ~0x0E) |= 0x02 << 1 : 0;      //方位キャリブレーション走行中(0x05)
        calibTime++;
    }
    else if(calibTime == 500){
        stateMode >> 1 == 0x02 ? stateMode = (stateMode &= ~0x0E) |= 0x03 << 1 : 0;      //方位キャリブレーション書き込み(0x07)
        calibTime++;        
    }
    else{
        stateMode >> 1 == 0x03 ? stateMode = (stateMode &= ~0x0E) |= 0x04 << 1 : 0;      //方位キャリブレーション完了(0x09)
        calibTime = 0;          //再キャリブ用にリセットしておく
    }
    return stateMode;
}

int8_t SMChangeRunState(int8_t stateMode)
{
    stateMode >> 4 == 0x00 ? stateMode = (stateMode &= ~0x10) ^= 0x01 << 4 : 0;         //走行開始(0x19)
    return stateMode;
}

int8_t SMChangeStopState(int8_t stateMode)
{
    stateMode >> 4 == 0x01 ? stateMode = (stateMode &= ~0x10) : 0;                      //停止(0x09)
    return stateMode;
}

void VehicleMotionControl(int8_t stateMode)
{
    switch(stateMode){
        case 0x03:  //キャリブレーション事前走行中
        case 0x05:  //キャリブレーション走行中
        case 0x07:  VMCHeadCalib(stateMode,yawRt,sampletimes,heading,&headingOffset,&strPwmOffset,&strPWM,&puPWM);//キャリブレーション済、停止
                    break;
        case 0x09:  //停止
                    VMCStop(&strPWM,&puPWM);
                    break;
        case 0x19:  VMCRunNorm(&strPWM,&puPWM);//通常走行
                    break;
        default  :  //停止
                    VMCStop(&strPWM,&puPWM);
                    break;
    }
    FStr.write((int)(180 - strPWM));            //操舵指示
    PowUnit.write((int)puPWM);          //駆動指示
}

//キャリブレーション走行  
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float headingGPS,float *IMUheadingOffset,float *strPwmOffset,float *strPWM,float *puPWM)
{
    static float strPWMoffsetFromCenter = 0;
    static float bufx = 0,bufy = 0;
    if(stateMode == 0x03 || stateMode == 0x05){
        *strPWM = StrControlPID(currentYawRate,0.0f,sampleTime,90);        
        *puPWM = 110;
    }
    else{
        *strPWM = 90;
        *puPWM = 90;
    }
    if(stateMode == 0x05){
        bufx += cos(headingGPS - yawAngle);
        bufy += sin(headingGPS - yawAngle);
        strPWMoffsetFromCenter += *strPWM - 90;
    }
    else if(stateMode == 0x07){
        if(!*IMUheadingOffset){
            *IMUheadingOffset = atan2f(bufy,bufx);
            *strPwmOffset = strPWMoffsetFromCenter / 300 + 90;
        }
        bufx = 0;
        bufy = 0;
        strPWMoffsetFromCenter = 0;
    }
}
//通常走行
void VMCRunNorm(float *strPWM,float *puPWM)
{
    float len,psi,phi1,cvCul;
    SelectHeadingInfo(*puPWM,velmps,yawRtGPS,heading,yawRt,yawAngle,sampletimes,&headingOffset,&head);    
    //初回 or オドメータがhに到達した場合、次の目標位置までの軌道・曲率を生成する
    if(odo >= h){
        SetNextCourseData(&ID,&xNext,&yNext,&headNext);
        GetLenAndDirection(x, y, head,xNext,yNext,headNext,&len,&psi,&phi1);//コースに準じてx,y,head設定する必要あり
        SetCalcClothoid(len,psi,0.0f,phi1,&h,&phiV,&phiU,5);
        odo = 0;
#if 0
        Serial.println("");
        Serial.print(",head,");Serial.print(head);Serial.print(",len,");Serial.print(len);
        Serial.print(",psi,");Serial.print(psi);Serial.print(",phi1,");Serial.print(phi1);
        Serial.print(",h,");Serial.print(h);
        Serial.print(",phiV,");Serial.print(phiV);Serial.print(",phiU,");Serial.print(phiU);
        Serial.print(",xNext,");Serial.print(xNext);
        Serial.print(",yNext,");Serial.print(yNext);
        Serial.print(",headNext,");Serial.print(headNext);
        Serial.print(",StrPwm,");Serial.print(*strPWM);
        Serial.print(",StrPwmOffset,");Serial.print(strPwmOffset);
        Serial.println("");
#endif
    }
    CalcCurrentCurvature(h,phiV,phiU,odo,&cvCul);
    MaxVelocitympsP(cvCul,velLimInitp[ID] * 0.001,AyLim,&maxVel);
    //Serial.print(",cvCul,");Serial.print(cvCul);
    //Serial.print(",YawRtTgt,");Serial.print(velmps * cvCul);
    //*strPWM = StrControlFF(cvCul,strPwmOffset);                 //スタビリティファクタ補正なし
    //*strPWM = StrControlFFwSF(cvCul,strPwmOffset,velmps);     //スタビリティファクタ補正あり
    *strPWM = StrControlFFFB(ID,cvCul,yawRt,velmps * cvCul,sampletimes,strPwmOffset,velmps);
    *puPWM = SpdControlPID(ID,velmps,maxVel,sampletimes);
    //*puPWM = SpdControlPID(ID,velmps,maxVel,sampletimes);    
    odo += velmps * sampletimes;
}
//停止
void VMCStop(float *strPWM,float *puPWM)
{
    ID = 0;
    xNext = 0;yNext = 0;headNext = 0;
    h=0;phiV=0;phiU=0;odo=0;   
    *strPWM = 90;
    *puPWM = 90;
}

//GPSのHeading情報飛び防止
void SelectHeadingInfo(float puPWM,float velocityMps,float yawRtGPS,float headingGPS,float yawRt,float yawAngle,float sampletimes,float *headingOffsetIMU,float *headingOut)
{
    //停止していない & 速度が1.0m/s以上 & ほぼ直進 & GPSとIMUのヨーレートもほぼ直進
    if(puPWM > 95 && velocityMps > 1.0f && abs(yawRt) < 0.1 && abs(yawRtGPS-yawRt) < 0.1){
        *headingOut = Pi2pi(headingGPS + yawRt * sampletimes);
        *headingOffsetIMU = Pi2pi(-yawAngle + *headingOut);                
    }
    else{
        *headingOut = Pi2pi(yawAngle + *headingOffsetIMU);
    }
}

void PrintInfo(void)
{
#if 0
Serial.print(",Timems,");Serial.print(timems);
//Serial.print(",Mode,");Serial.print(stateMode,HEX);
Serial.print(",x,");Serial.print(x);
Serial.print(",y,");Serial.print(y);
Serial.print(",heading,");Serial.print(head);
Serial.print(",yawAng,");Serial.print(yawAngle);
Serial.print(",yawRt,");Serial.print(yawRt);
Serial.print(",yawRtGPS,");Serial.print(yawRtGPS);
Serial.print(",velmps,");Serial.print(velmps);
Serial.print(",maxVel,");Serial.print(maxVel);
Serial.print(",ID,");Serial.print(ID);
Serial.print(",odo,");Serial.print(odo);
Serial.print(",strPWM,");Serial.print(strPWM);
Serial.print(",puPWM,");Serial.print(puPWM);
Serial.print(",headOffst,");Serial.print(headingOffset);
Serial.println("");
#endif
}


float StrControlFF(float cvCul,float strPwmOffset)
{
    float pwmFF = strPwmOffset;
    float kFstrPWMLow = 190.0f;
    float kFstrPWMHigh = 288.0f;
    int8_t sign = (cvCul > 0) - (cvCul < 0);                     //符号抽出(sqrt関数対策)
    float strRad = WHEELBase * abs(cvCul);
    if(strRad < 0.05){
        pwmFF = (float)sign * kFstrPWMLow * strRad + strPwmOffset;
    }
    else{
        pwmFF = (float)sign * kFstrPWMHigh * strRad + strPwmOffset;
    }    
    return constrain(pwmFF,40,140);
}
//スタビリティファクタを考慮
float StrControlFFwSF(float cvCul,float strPwmOffset,float velmps)
{
    float pwmFF = strPwmOffset;
    float kFstrPWMLow = 190.0f;
    float kFstrPWMHigh = 288.0f;
    int8_t sign = (cvCul > 0) - (cvCul < 0);                     //符号抽出(sqrt関数対策)
    cvCul *= (1 + StabilityFactor * pow(velmps,2));              //スタビリティファクタ分の曲率補正
    float strRad = WHEELBase * abs(cvCul);
    if(strRad < 0.05){
        pwmFF = (float)sign * kFstrPWMLow * strRad + strPwmOffset;
    }
    else{
        pwmFF = (float)sign * kFstrPWMHigh * strRad + strPwmOffset;
    }  
    return constrain(pwmFF,40,140);
}

float StrControlFFFB(int16_t ID,float cvCul,float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset,float velmps)
{
    float kP = 20,tI = 0.205,tD = 0.02625,bias = StrControlFFwSF(cvCul,strPwmOffset,velmps);
    static uint8_t countCurrentID = 0;
    static int16_t lastID = 0;
    static float error_prior = 0,integral = 0;
    #if 0
    float error = 0,derivative = 0,output = 0;
    if(ID != lastID){
        error_prior = 0;
        integral = 0;
        countCurrentID = 0;
    }
    else if(countCurrentID > 20){
        error = targetYawRate - currentYawRate;
        integral += (error * sampleTime);
        derivative = (error - error_prior)/sampleTime;
    }
    else{
        countCurrentID++;
    }
    #endif
    float error = targetYawRate - currentYawRate;
    integral += (error * sampleTime);
    float derivative = (error - error_prior)/sampleTime;
    float output = kP * (error + integral / tI + tD * derivative) + bias;
    
    output = constrain(output,40,140);
    error_prior = error;
    lastID = ID;
    return output;
}
/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨーレートフィードバック)
 * INPUT    : CPまでの目標ヨーレート、現在ヨーレート
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPID(float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset)
{
   float kP = 20,tI = 0.205,tD = 0.02625,bias = strPwmOffset;
   static float error_prior = 0,integral = 0;
   float error = 0,derivative = 0,output = 0;
   
   error = targetYawRate - currentYawRate;
   integral += (error * sampleTime);
   derivative = (error - error_prior)/sampleTime;
   output = kP * (error + integral / tI + tD * derivative) + bias;
   output = constrain(output,40,140);
   error_prior = error;

   return output;
}

/************************************************************************
 * FUNCTION : 速度コントロール
 * INPUT    : 現在の速度、目標速度
 * OUTPUT   : モータ制御指示値
 ***********************************************************************/
float SpdControlPID(int16_t ID,float currentSpeed,float targetSpeed,float sampleTime)
{
	float kP = 8,tI = 0.5,tD = 0.125,bias = 90;
    static float error_prior = 0,integral = 0;
    static int16_t lastID = 0;
    float error,derivative,output;
	error = targetSpeed - currentSpeed;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = kP * (error + integral / tI + tD * derivative) + bias;
    output = constrain(output,PUPWMLimLWR,PUPWMLimUPR);
	error_prior = error;
    lastID = ID;    
	return output;
}
