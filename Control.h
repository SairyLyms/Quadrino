
#define LimitAreaRun 30 //走行可能範囲(m)

int8_t StateManager(float e,float n,int8_t stateMode);
int8_t SMLimitArea(float e,float n,int8_t stateMode);
int8_t SMHeadCalib(int8_t stateMode);


int8_t StateManager(float e,float n,int8_t stateMode){
    stateMode = SMLimitArea(e,n,stateMode);
    stateMode = SMHeadCalib(stateMode);
    return stateMode;
}

/************************************************************************
 * FUNCTION : 状態管理
 * INPUT    : 原点__からの位置座標(E,N)
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
    if(!(stateMode & 0x08)){        //方位キャリブレーション未
        if(calibTime < 500){         //500サイクルキャリブ実施する
            stateMode |= 0x04;      //方位キャリブレーション中
            calibTime++;
        }
        else {
            stateMode ^= 0x0C;      //方位キャリブレーション済
        }
    }
    return stateMode;
}

int8_t SMChangeRunState(int8_t stateMode)
{
    stateMode ^= 0x10; 
    return stateMode;
}