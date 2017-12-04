//-------------------------
//Definition
//-------------------------

// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
volatile const int16_t CourseData[][3]    =
{{0  , 0, 0         },  //初期位置(0,0,0)
{10,-5,0},
{10,5,M_PI * 10000},
{0,0,atan2(-5,-10)*10000},
{-10,-5,M_PI * 10000},
{-10,5,0},
{0,0,atan2(-5,10)*10000}
};

volatile const float latlonCp[2][2] = {{36.568130, 139.995850},{36.567968, 139.995850}};
//volatile const float latlonCp[2][2] = {{36.567874, 139.995864},{36.567874, 139.995764}};
volatile const float heightCenter = 188.0f;
const float latlonCenter[2] = {0.5 * (latlonCp[0][0] + latlonCp[1][0]),0.5 * (latlonCp[0][1] + latlonCp[0][1])};
const float courseScale = 1;
const int lenCourseData = sizeof(CourseData)/sizeof(CourseData[0]);
volatile uint16_t velLimInitp[lenCourseData];
volatile float directionCp;