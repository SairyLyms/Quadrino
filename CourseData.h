// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
int16_t CourseData[][3]    = {{0  , 0, 0         },  //初期位置(0,0,0)
{12156,7221,3629},
{12736,7438,2780},
{13099,7511,277},
{13353,7475,-2318},
{13571,7402,-4913},
{14224,6894,-7232},
{15022,6096,-8605},
{15530,5406,-9168},
{17381,3084,0},
{21010,1451,-1125},
{21880,1378,-785},
{22860,1306,-369},
{23804,1306,-135},
{24475,1288,0},
{25146,1306,362},
{25945,1342,990},
{26416,1415,2094},
{27215,1632,0},
{31533,3120,1133},
{32004,3120,-2496},
{32404,2902,-7133},
{32621,2612,-10172},
{32767,2322,-13389},
{32767,1995,-15707},
{32767,1669,-16607},
{32694,1270,-19142},
{32512,943,-21837},
{32258,653,-24102},
{31896,399,-26200},
{31424,181,-28490},
{30952,108,-29889},
{30480,36,-30099},
{30154,0,0},
{4354,290,29361},
{3846,435,27825},
{3229,725,26719},
{2431,1161,25407},
{1959,1560,23977},
{1596,1923,23008},
{1016,2648,21740},
{653,3265,20578},
{326,3955,19350},
{145,4572,18125},
{0,5297,16694},
{0,6059,15030},
{108,6858,13573},
{326,7583,11932},
{653,8237,10449},
{1016,8781,9134},
{1596,9434,7649},
{1995,9761,6802},
{2358,10051,0}
};
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
    *xNext = (float)CourseData[*NextCourseID][0];
    *yNext = (float)CourseData[*NextCourseID][1];
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