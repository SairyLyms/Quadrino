// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
int16_t CourseData[][3]    = {{0  , 0, 0         },  //初期位置(0,0,0)
{10,-5,0},
{10,5,M_PI * 10000},
{0,0,atan2(-5,-10)*10000},
{-10,-5,M_PI * 10000},
{-10,5,0},
{0,0,atan2(-5,10)*10000}
};
float courseScale = 1;
// ================================================================
// ===              	Course Data Functions.			        ===
// ================================================================
void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext);
void GetLenAndDirection(float x, float y, float head,float xNext,float yNext,float headNext,float* len,float* psi,float* phi1);
float Pi2pi(float angle);

void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext)
{
    int numofID = sizeof(CourseData)/sizeof(CourseData[0]); //軌道の分割数
    *NextCourseID >= numofID-1 ? *NextCourseID = 1 : (*NextCourseID)++;
    *xNext = (float)CourseData[*NextCourseID][0] * courseScale;
    *yNext = (float)CourseData[*NextCourseID][1] * courseScale;
    *headNext =  (float)CourseData[*NextCourseID][2] * 0.0001;
}

int16_t GetCourseDataLen(void)
{
    return sizeof(CourseData)/sizeof(CourseData[0]);
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