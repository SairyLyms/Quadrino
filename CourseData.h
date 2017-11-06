// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
float CourseData[][3]    = {{0  , 0, 0         },  //初期位置(0,0,0)
                            {0  ,-5, 0         },  //以下、コース情報
                            {10 , 0, 0.5 * M_PI},
                            {0  , 5, 1.0 * M_PI},
                            {-10, 0,-0.5 * M_PI}   };
// ================================================================
// ===              	Course Data Functions.			        ===
// ================================================================
void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext);
float Pi2pi(float angle);

void SetNextCourseData(int* NextCourseID,float* xNext,float* yNext,float *headNext)
{
    int numofID = sizeof(CourseData)/sizeof(CourseData[0]); //軌道の分割数
    *NextCourseID >= numofID-1 ? *NextCourseID = 1 : (*NextCourseID)++;
    *xNext = CourseData[*NextCourseID][0];
    *yNext = CourseData[*NextCourseID][1];
    *headNext =  CourseData[*NextCourseID][2];
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