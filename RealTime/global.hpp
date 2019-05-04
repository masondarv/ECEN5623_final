#ifndef  __GLOBAL__
#define  __GLOBAL__
#include <iostream>
#include <string>     // std::string, std::to_string
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "steering.hpp"
#include "util.hpp"


#define FOV_WIDTH (640)
#define FOV_HEIGHT (480)
#define FOV_CUT_UPPERBOUND (235)

#define MIN_BLUE_DETECTED_TO_SPEEDUP (100)
#define HIGH_ERROR (150)

#define MAX_THROT (120)


//#define PROPORTIONAL (110.0f)
//#define INTEGRAL (3.5f)
//#define DERIVATIVE (900.0f)
#define PROPORTIONAL (95.0f)
#define INTEGRAL (3.0f)
#define DERIVATIVE (550.0f)
//known good PID values:
//P = 95
//I = 3
//D = 550



using namespace cv;
using namespace std;


vector<Mat> RGB_shared;
Size ROI;
Mat frame,labeled_area,blank;
ExpMovingAverage k,b;
steering cntl;
int fd;
VideoCapture cap;
int blue_arrow_cnt;
Point point1,point2;
int exit_flag;
int display_flag;
#endif
