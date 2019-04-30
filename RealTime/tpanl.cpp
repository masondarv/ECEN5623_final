
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h> 

#include <termios.h>
#include <unistd.h>

#define PI 3.14159265

#include <iostream>
#include <string>     // std::string, std::to_string
#include <vector>
#include "steering.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>


#define K_VARY_FACTOR 30
#define B_VARY_FACTOR 500


using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
	Mat frame_ori,frame,kernel;
	string name;
	int cnt=0;
	int TP=0;
	int FP=0;
	int TN=0;
	int FN=0;
	uchar key;
	for(cnt=0;cnt<200;cnt++){
		name ="cap"+to_string(1+cnt*31)+".ppm";
		frame = imread(name);
		imshow(name,frame);
		key = waitKey();
		if(key=='q')
			TP++;
		else if(key=='w')
			FP++;
		else if(key=='e')
			TN++;
		else if (key=='r')
			FN++;
		destroyAllWindows();
	}
	cout <<"TP:"<<TP<<"  FP"<<FP<<"  TN"<<TN<<"  FN"<<FN<<endl;
	return -1;
    
}
