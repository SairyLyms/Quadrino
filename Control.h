
#define LimitAreaRun 30.0f //走行可能範囲(m)

extern float yawRt,sampletimes,heading,headingOffset,strPwmOffset,strPWM,puPWM;
extern Servo FStr,PowUnit;

int8_t StateManager(float e,float n,int8_t stateMode);
int8_t SMLimitArea(float e,float n,int8_t stateMode);
int8_t SMHeadCalib(int8_t stateMode);
int8_t SMChangeRunState(int8_t stateMode);
int8_t SMChangeStopState(int8_t stateMode);
void VehicleMotionControl(int8_t stateMode);
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float hading,float *headingOffset,float *strPwmOffset,float *strPWM,float *puPWM);
void VMCRunNorm(void);
void VMCStop(void);
float StrControlPID(float currentYawRate,float targetYawRate,float sampleTime,float strPwmOffset);
float SpdControlPID(float currentSpeed,float targetSpeed,float sampleTime);

int8_t StateManager(float e,float n,int8_t stateMode){
    int8_t key = 0;
    stateMode = SMLimitArea(e,n,stateMode);
    if(stateMode & 0x01){                                        //走行可能範囲内の場合
        switch(stateMode){
            case 0x01:  Serial.println("To start hading calib, Send 'c'");
                        while(!Serial.available());
                        while(Serial.read() != 'c');
                        stateMode = SMHeadCalib(stateMode);  
                        break;
            case 0x03:
            case 0x05:  stateMode = SMHeadCalib(stateMode);
                        break;
            case 0x07:  Serial.println("To start a vehicle , Send 'r' , To stop , Send 'Any' key... ");
                        while(!Serial.available());
                        while(Serial.read() != 'r');
                        stateMode = SMChangeRunState(stateMode);    //走行開始
                        break;
            case 0x17:  key = Serial.read();
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
        stateMode == 0x01 ? stateMode |= 0x02 : 0;      //方位キャリブレーション事前走行中(0x03)
        calibTime++;
    }
    else if(calibTime < 500){   //200サイクル超でキャリブ実施
        stateMode == 0x03 ? stateMode ^= 0x06 : 0;      //方位キャリブレーション走行中(0x05)
        calibTime++;
    }
    else {
        stateMode == 0x05 ? stateMode ^= 0x02 : 0;      //方位キャリブレーション済(0x07)
    }
    return stateMode;
}

int8_t SMChangeRunState(int8_t stateMode)
{
    stateMode == 0x07 ? stateMode ^= 0x10 : 0; 
    return stateMode;
}

int8_t SMChangeStopState(int8_t stateMode)
{
    stateMode &= ~0x10;
    return stateMode;
}

void VehicleMotionControl(int8_t stateMode)
{
    switch(stateMode){
        case 0x03:
        case 0x05:
        case 0x07:  VMCHeadCalib(stateMode,yawRt,sampletimes,heading,&headingOffset,&strPwmOffset,&strPWM,&puPWM);//キャリブレーション
                    break;
        case 0x17:  //通常走行
                    break;
        default  :  strPWM = 90;
                    puPWM = 90;
                    break;
    }
    FStr.write((int)strPWM);
    PowUnit.write((int)puPWM);
}

//キャリブレーション走行  
void VMCHeadCalib(int8_t stateMode,float currentYawRate,float sampleTime,float hading,float *headingOffset,float *strPwmOffset,float *strPWM,float *puPWM)
{
    static float bufx = cos(heading),bufy = sin(heading);
    if(stateMode == 0x03 || stateMode == 0x05){
        *puPWM = 110;
    }
    else{
        *puPWM = 90;
    }
    if(stateMode == 0x03){
        *strPWM = StrControlPID(currentYawRate,0.0f,sampleTime,90);
    }
    else if(stateMode == 0x05){
        *strPWM = StrControlPID(currentYawRate,0.0f,sampleTime,90);
        bufx = 0.5 * (bufx + cos(heading));
        bufy = 0.5 * (bufy + sin(heading));
        *strPwmOffset = 0.5 * (*strPwmOffset + *strPWM);
    }
    else if(stateMode == 0x07){
        if(!*headingOffset){
            *headingOffset = atan2f(bufy,bufx);
            *strPwmOffset = 0.5 * (*strPwmOffset + *strPWM);
        }
        *strPWM = 90;
    }
}
//通常走行
void VMCRunNorm(void)
{

}
//停止
void VMCStop(void)
{
    FStr.write(90);
    PowUnit.write(90);
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
