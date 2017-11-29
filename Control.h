
#define LimitAreaRun 30.0f  //走行可能範囲(m)
#define PWMInitCenter 90.0f //PWM初期中点値
#define AyLim 9.8 * 0.5f          //限界横G

extern float yawAngle,yawRt,sampletimes,heading,headingOffset,strPwmOffset,strPWM,puPWM,velmps;
extern Servo FStr,PowUnit;
extern int ID;
extern float x,y,head,xNext,yNext,headNext;
extern float h,phiV,phiU,odo;

int8_t StateManager(float e,float n,int8_t stateMode);
int8_t SMLimitArea(float e,float n,int8_t stateMode);
int8_t SMHeadCalib(int8_t stateMode);
int8_t SMChangeRunState(int8_t stateMode);
int8_t SMChangeStopState(int8_t stateMode);
void VehicleMotionControl(int8_t stateMode);
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float hading,float *headingOffset,float *strPwmOffset,float *strPWM,float *puPWM);
void VMCRunNorm(void);
void VMCStop(float *strPWM,float *puPWM);
void SelectHeadingInfo(float velocityMps,float yawAngle,float headingGPS,float *headingOffset,float *headingOut);
float StrControlPID(float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset);
float SpdControlPID(float currentSpeed,float targetSpeed,float sampleTime);

int8_t StateManager(float e,float n,int8_t stateMode){
    int8_t key = 0;
    stateMode = SMLimitArea(e,n,stateMode);
    if(stateMode & 0x01){                                        //走行可能範囲内(stateModeの0ビット目 == 1)の場合
        switch(stateMode){
            case 0x01:  Serial.println("To start hading calib, Send 'c'");
                        while(!Serial.available());
                        while(Serial.read() != 'c');
                        stateMode = SMHeadCalib(stateMode);  
                        break;
            case 0x03:
            case 0x05:  
            case 0x07:  stateMode = SMHeadCalib(stateMode);
                        break;
            case 0x09:  Serial.println("To start a vehicle , Send 'r' , To stop , Send 'Any' key... ");
                        while(!Serial.available());
                        while(Serial.read() != 'r');
                        stateMode = SMChangeRunState(stateMode);    //走行開始
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
        Serial.println("Out of course,");                      //走行可能範囲外
    }
    return stateMode;
}

/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    : 原点からの位置座標(E,N)
 *
 * OUTPUT   : 状態値
 ***********************************************************************/
int8_t SMLimitArea(float e,float n,int8_t stateMode)
{
    float rfromCenter;
    GetLenAndDirection(0,0,NULL,e,n,NULL,&rfromCenter,NULL,NULL);
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
        case 0x19:  //通常走行
                    break;
        default  :  //停止
                    VMCStop(&strPWM,&puPWM);
                    break;
    }
    FStr.write((int)strPWM);            //操舵指示
    PowUnit.write((int)puPWM);          //駆動指示
}

//キャリブレーション走行  
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float headingGPS,float *headingOffset,float *strPwmOffset,float *strPWM,float *puPWM)
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
        strPWMoffsetFromCenter = *strPWM - 90;
    }
    else if(stateMode == 0x07){
        if(!*headingOffset){
            *headingOffset = atan2f(bufy,bufx);
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
    float len,psi,phi1,cvCul,maxVel;
    //初回 or オドメータがhに到達した場合、次の目標位置までの軌道・曲率を生成する
    if(odo >= h){
        SelectHeadingInfo(velmps,yawAngle,heading,&headingOffset,&head);//速度に応じてHeading情報持ちかえる
        SetNextCourseData(&ID,&xNext,&yNext,&headNext);
        GetLenAndDirection(x, y, head,xNext,yNext,headNext,&len,&psi,&phi1);
        SetCalcClothoid(len,psi,0.0f,phi1,&h,&phiV,&phiU,5);
        odo = 0;
    }
    CalcCurrentCurvature(h,phiV,phiU,odo,&cvCul);
    MaxVelocitympsP(cvCul,velLimInitp[ID],AyLim,&maxVel);
    *strPWM = StrControlPID(yawRt,velmps * cvCul,sampletimes,strPwmOffset);  
    *puPWM = SpdControlPID(velmps,maxVel,sampletimes);
    odo += velmps * sampletimes;
}
//停止
void VMCStop(float *strPWM,float *puPWM)
{
    ID = 0;
    x = 0;y = 0;head = 0;xNext = 0;yNext = 0;headNext = 0;
    h=0;phiV=0;phiU=0;odo=0;   
    *strPWM = 90;
    *puPWM = 90;
}

//速度に応じてHeading情報持ちかえる
void SelectHeadingInfo(float velocityMps,float yawAngle,float headingGPS,float *headingOffset,float *headingOut)
{
    if(velocityMps > 1.0f){
        *headingOut = headingGPS;
        *headingOffset = Pi2pi(-yawAngle + headingGPS);
    }
    else{
        *headingOut = yawAngle + *headingOffset;
    }
}
/************************************************************************
 * FUNCTION : 操舵制御指示値演算(ヨーレートフィードバック)
 * INPUT    : CPまでの目標ヨーレート、現在ヨーレート
 * OUTPUT   : 操舵制御指示値(サーボ角)
 ***********************************************************************/
float StrControlPID(float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset)
{
   float kP = 30,tI = 0.105,tD = 0.02625,bias = strPwmOffset;
   static float error_prior = 0,integral = 0;
   float error,derivative,output;

   error = targetYawRate - currentYawRate;
   integral += (error * sampleTime);
   derivative = (error - error_prior)/sampleTime;
   output = kP * (error + integral / tI + tD * derivative) + bias;
   error_prior = error;

   return output;
}

/************************************************************************
 * FUNCTION : 速度コントロール
 * INPUT    : 現在の速度、目標速度
 * OUTPUT   : モータ制御指示値
 ***********************************************************************/
float SpdControlPID(float currentSpeed,float targetSpeed,float sampleTime)
{
	float kP = 8,tI = 0.5,tD = 0.125,bias = 90;
	static float error_prior = 0,integral = 0;
	float error,derivative,output;

	error = targetSpeed - currentSpeed;
	integral += (error * sampleTime);
	derivative = (error - error_prior)/sampleTime;
	output = kP * (error + integral / tI + tD * derivative) + bias;
	output = constrain(output,80,180);
	error_prior = error;

	return output;
}
