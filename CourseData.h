//-------------------------
//Definition
//-------------------------

// ================================================================
// ===              	Course Data Def.			            ===
// ================================================================
volatile const int16_t CourseData[][3]    =
{{0  , 0, 0         },  //初期位置(0,0,0)
{10,-2,0},
{10,2,M_PI * 10000},
{0,0,atan2(-2,-10)*10000},
{-10,-2,M_PI * 10000},
{-10,2,0},
{0,0,atan2(-2,10)*10000}
};

const float courseScale = 1;
const int lenCourseData = sizeof(CourseData)/sizeof(CourseData[0]);
volatile uint16_t velLimInitp[lenCourseData];
