/*
 *
 *  Example by Sam Siewert 
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>     // std::string, std::to_string


#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
using namespace cv;
using namespace std;
#define ORI_X (229)
#define ORI_Y (73)
#define FOV_WIDTH (640)
#define R_WIDTH ((FOV_WIDTH/2-ORI_X)*2)
#define R_HEIGHT (42)

void annotate(Mat& src)
{
	line(src, Point(ORI_X,ORI_Y), 
			Point(ORI_X,ORI_Y+R_HEIGHT), Scalar(0, 0, 255), 1);
	line(src, Point(ORI_X+R_WIDTH,ORI_Y), 
		Point(ORI_X,ORI_Y), Scalar(0, 0, 255), 1);
	line(src, Point(ORI_X+R_WIDTH,ORI_Y), 
		Point(ORI_X+R_WIDTH,ORI_Y+R_HEIGHT), Scalar(0, 0, 255), 1);
	line(src, Point(ORI_X+R_WIDTH,ORI_Y+R_HEIGHT), 
		Point(ORI_X,ORI_Y+R_HEIGHT), Scalar(0, 0, 255), 1);
}
int main( int argc, char** argv )
{
	Mat frame;
    // use default camera as video source
    VideoCapture cap(atoi(argv[1]));
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
	int i=0;
	bool flag=0;
	Mat src;
	string name,base;
	base = "pic";
	cout<<cap.get(CAP_PROP_FRAME_WIDTH)<<"  "<<cap.get(CAP_PROP_FRAME_HEIGHT)<<endl;
	namedWindow("cap",WINDOW_NORMAL);
	setWindowProperty("cap",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
	setWindowProperty("cap",WND_PROP_AUTOSIZE,WINDOW_NORMAL);
   while(1)
    {
         cap >> frame;
     
        if(frame.empty()) break;
		annotate(frame);
		setWindowProperty("cap",WND_PROP_FULLSCREEN,WINDOW_NORMAL);
		imshow("cap",frame);
		
		if(waitKey(1)=='q')
			break;
     
        /* char c = cvWaitKey(0);
		
		if( c =='s') flag =1;
        if( c == 27 ) break;
		if(flag)
		{
			name = base+ std::to_string(i)+".ppm";
			imwrite(name,frame);
			i++;
		} */
		

    }
	return -1;
    
};
