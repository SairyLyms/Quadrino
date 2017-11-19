//-------------------------
//Definition
//-------------------------

// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
int16_t CourseData[][3]    = {{0  , 0, 0         },  //初期位置(0,0,0)
{10,-2,0},
{10,2,M_PI * 10000},
{0,0,atan2(-2,-10)*10000},
{-10,-2,M_PI * 10000},
{-10,2,0},
{0,0,atan2(-2,10)*10000}
};
float courseScale = 1;
const int lenCourseData = sizeof(CourseData)/sizeof(CourseData[0]);
uint16_t velLimInitp[lenCourseData];
// ================================================================
// ===              	Course Data Functions.			        ===
// ================================================================
void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext);
void GetLenAndDirection(float x, float y, float head,float xNext,float yNext,float headNext,float* len,float* psi,float* phi1);
float Pi2pi(float angle);
void SetVelocityLimInit(float velLimitMax,float ayLim);

void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext)
{
    *NextCourseID >= lenCourseData-1 ? *NextCourseID = 1 : (*NextCourseID)++;
    *xNext = (float)CourseData[*NextCourseID][0] * courseScale;
    *yNext = (float)CourseData[*NextCourseID][1] * courseScale;
    *headNext =  (float)CourseData[*NextCourseID][2] * 0.0001;
}

void GetLenAndDirection(float x, float y, float head,float xNext,float yNext,float headNext,float* len,float* psi,float* phi1)
{
 *len = sqrtf(pow(xNext - x,2) + pow(yNext - y,2));
 *psi = Pi2pi(atan2f(yNext - y,xNext - x)-head);
 *phi1 = Pi2pi(headNext - head);
}

/************************************************************************
 * FUNCTION : 角度を-piからpiの範囲に納める
 * INPUT    : 
 * OUTPUT   : 
 ***********************************************************************/
float Pi2pi(float angle)
{
    while(angle >= M_PI) {angle -= M_PI * 2;}
    while(angle < -M_PI){angle += M_PI * 2;}
    return angle;
}

// ================================================================
// ===              	Velocity info Def.  			        ===
// ================================================================
//コース一周分の大体の最高速度を各区間の初期リミット速度としてセットする
void SetVelocityLimInit(float velLimitMax,float ayLim)
{   int NextCourseID = 0;
    float cvMax=0,deltaCv=0,cvMaxPrev=0,deltaCvPrev=0;
    float xL=0,yL=0,headL=0,xNextL=0,yNextL=0,headNextL=0;
    for(int j=0;j<2;j++){//スタート軌跡とゴール軌跡を接続させるため2周分演算する
        int i;
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
#if 1
            SetNextCourseData(&NextCourseID,&xNextL,&yNextL,&headNextL);
            GetLenAndDirection(xL, yL, headL,xNextL,yNextL,headNextL,&len,&psi,&phi1);
            CalcClothoid(len,psi,0.0f,phi1,&h,&phiV,&phiU,5);
            CvMaxMin(h,phiV,phiU,NULL,&cvMax,NULL,NULL,&deltaCv);
            cvMax = abs(cvMax);  //速度の算出なので、曲率の絶対値を使用
            if(cvMaxPrev > cvMax && deltaCv < -0.1){//曲率が前回値より小さく、明らかに曲率が減少している場合
                velLimInitp[i] = velLimitMax * 1000;
            }
            else{
                velLimInitp[i] = MaxVelocitymps(cvMax,ayLim) * 1000;
                i > 1 && ((cvMax - cvMaxPrev) / cvMax) > 0.1 ? velLimInitp[i-1] = MaxVelocitymps(cvMax,ayLim) * 1000 : 0;
                i == 1 && ((cvMax - cvMaxPrev) / cvMax) > 0.1 ? velLimInitp[lenCourseData-1] = MaxVelocitymps(cvMax,ayLim) * 1000 : 0;
            }
            cvMaxPrev = cvMax;deltaCvPrev = deltaCv;
            xL = xNextL;yL = yNextL;headL = headNextL;
            i >= lenCourseData-1 ? i = 1 : i++;
#endif    
        }
    }
#if 1//セグメント毎制限速度確認デバッグ用
    for(int i=0;i<lenCourseData;i++){
        Serial.print(",Segment,");Serial.print(i);
        Serial.print(",VelLim,");Serial.print(velLimInitp[i]);
        Serial.println("");
    }
#endif
}

