//-------------------------
//Definition
//-------------------------

// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
#if 1
volatile const int16_t CourseData[][3]    =
{{0  , 0, 0         },  //初期位置(0,0,0)
{8,-4,0},
{8,4,M_PI * 10000},
{0,0,atan2(-4,-8)*10000},
{-8,-4,M_PI * 10000},
{-8,4,0},
{0,0,atan2(-4,8)*10000}
};
#endif
#if 0
volatile const int16_t CourseData[][3]    =
{{0 , 0, 0          },  //初期位置(0,0,0)
{5  , 0, 0          },
{8  , 2, 0.25 * M_PI * 10000},
{10  , 5, 0.5 * M_PI * 10000},
{8  , 8, 0.75 * M_PI * 10000},
{5  , 10, M_PI * 10000},
{2  , 8, -0.75 * M_PI * 10000},
{0  , 5, -0.5 * M_PI * 10000},
{2  , 2, -0.25 * M_PI * 10000},
};
#endif
//volatile const float latlonCp[2][2] = {{35.66743000, 139.79096500},{35.66730667, 139.79081167}};
volatile const float latlonCp[2][2] = {{36.56782532, 139.99629211},{36.56782532, 139.99629211}};
volatile const float heightCenter = 166.309998;
const float latlonCenterRad[2] = {M_PI / 180 * 0.5 * (latlonCp[0][0] + latlonCp[1][0]),M_PI / 180 * 0.5 * (latlonCp[0][1] + latlonCp[0][1])};
const float courseScale = 1;
const int lenCourseData = sizeof(CourseData)/sizeof(CourseData[0]);
volatile uint16_t velLimInitp[lenCourseData];
volatile float directionCp;