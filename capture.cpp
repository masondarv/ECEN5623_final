
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

double k_to_theta(double k){
	return atan (k) * 180 / PI;
}

double get_error (double k,double b,Mat& image){
	double pos1 = (-b/k) - (double)(image.cols/2.0f);
	double pos2 = ((double)image.rows-b)/k - (double)(image.cols/2.0f);
	return pos1*0.7+pos2*0.3;
}
class ExpMovingAverage {
public:
    ExpMovingAverage() {
        this->alpha = 0.2;
		unset = true;
    }
	void clear() {
		unset = true;
	}
	
    void add(double value) {
        if (unset) {
            oldValue = value;
			unset = false;
        }
        double newValue = oldValue + alpha * (value - oldValue);
        oldValue = newValue;
    }

	double get() {
		return oldValue;
	}
	
	double alpha; // [0;1] less = more stable, more = less stable
    double oldValue;
	bool unset;
};

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}


int main( int argc, char** argv )
{
	Mat frame_ori,frame,kernel;
	vector<Mat> RGB;
	Mat G,B,R;
	Mat store;
	Mat max,min,inter,comp;
	Scalar averg;
	uchar key;
	int debug_flag=0;
	kernel = getStructuringElement(MORPH_CROSS, Size(3, 3));
	//////////////////
	Mat gray,binary,mfblur;
	Mat temp;
	Mat eroded;
    //Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	bool done;
	int iterations=0;
    Mat element = (Mat_<uchar>(3,3) << 0, 0, 0, 1, 1, 1, 0, 0, 0);
	Mat skel;
	////////////////track detection/////////////////////
	vector<Point> thre_points;
	Vec4f line_para; 
	Point point0,point1, point2;
	double k_current,k_pre;
	double b_current,b_pre;
	int area_cnt=0;
	Mat label,stats,centroids;
	Mat labeled_area;
	ExpMovingAverage k,b;
	///////////////////////////
	 vector<Vec4i> lines_labeled;
	 
	 ///////////////////steering sys///////////////////////
	 steering cntl(75.0f,0.3f,500.0f,70); //set PID parameters
	 double error;
	 
	////////////////////serial port declaration////////////
	char *portname = "/dev/ttyUSB0";
	int fd;
	int wlen;
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
		
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
	
	
	
	// use default camera as video source
    VideoCapture cap(atoi(argv[1]));
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
	int i=0;
	bool flag=0;
	Mat src,sobel;
	string name,base;
	base = "Y";
	cout<<cap.get(CAP_PROP_FRAME_WIDTH)<<"  "<<cap.get(CAP_PROP_FRAME_HEIGHT)<<endl;
	namedWindow("cap");
	Mat comb;
	while(1)
    {
         cap >> frame_ori;
		
        if(frame_ori.empty()) break;
		frame = frame_ori.rowRange(235,480);
		if(mfblur.empty()){
			mfblur =Mat(frame.rows,frame.cols,CV_8UC1);
			
		}
		skel =Mat(mfblur.size(), CV_8UC1, Scalar(0));	
		labeled_area = Mat(frame.size(), CV_8UC1, Scalar(0));
		medianBlur(frame,frame,3);
		split(frame,RGB);
		merge(RGB,comb);
		imshow("comb",comb);
		imshow("R",RGB[2]);
		imshow("G",RGB[1]);
		imshow("B",RGB[0]);
		
		store = RGB[0].rowRange(126,190);
		store = frame.colRange(187,380);
		averg = mean(store);
		//cout << averg.val[0] << endl;
		
		R = RGB[0]/2+RGB[1]/2-RGB[2];
		threshold(RGB[0],RGB[0],120,255,THRESH_BINARY_INV);
		threshold(RGB[2],RGB[2],170,255,THRESH_BINARY_INV);
		threshold(RGB[1],RGB[1],120,255,THRESH_BINARY);
		GaussianBlur(R,R,Size(5,5),0,0);
		GaussianBlur(R,R,Size(5,5),0,0);
		GaussianBlur(R,R,Size(5,5),0,0);
		GaussianBlur(R,R,Size(5,5),0,0);
		threshold(R,max,30,255,THRESH_BINARY); 
		threshold(R,min,25,255,THRESH_BINARY); 
		max = max & RGB[2] & RGB[1] ;
		imshow("max_thre",RGB[1]);
		for(;;){
			dilate(max,max,element,Point(-1,-1),1);
			inter = max & min;
			absdiff(max,inter,comp);
			if(countNonZero(comp)!=0)
				max = inter; 
			else{
				break;
			}	
		}
	
		
		connectedComponentsWithStats(max,label,stats,centroids,8);
		area_cnt =0;
		thre_points.clear();
		for(int i=1;i<centroids.rows;i++){
				
			if(stats.at<int>(i,CC_STAT_AREA) >= 30){
				area_cnt++;
				circle( labeled_area, Point((int)centroids.at<double>(i,0),(int)centroids.at<double>(i,1)),   2, Scalar(255,0,0), 1 );
				thre_points.push_back(Point((int)centroids.at<double>(i,0),(int)centroids.at<double>(i,1)));
			}	
		}
		
		if(area_cnt >=2){
			fitLine(thre_points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
			point0.x = line_para[2];
			point0.y = line_para[3];

			
			
			
			if(  (fabs(line_para[1])<=1) && (fabs(line_para[1])>=0.25)  ){
				k_current = line_para[1] / line_para[0];
				point1.x = 0;
				b_current = k_current * (0 - point0.x) + point0.y;
				
				
				double k_diff = fabs(k_to_theta(k_current) - k_to_theta(k.get()));
				double b_diff = fabs(b_current - b.get());

				bool update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR || (k.unset || b.unset)) ;
				if(update_ok){
					k.add(k_current);
					b.add(b_current);
				}
				else{
					k_diff = fabs(k_to_theta(k_current) - k_to_theta(k_pre));
					b_diff = fabs(b_current - b_pre);
					update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR) ;
					if(update_ok){
						k.clear();
						b.clear();
						k.add(k_current);
						b.add(b_current);
					}
				}
				
				k_pre = k_current;
				b_pre = b_current;
				point1.y= b.get();
				point2.x = frame.cols;
				point2.y = k.get() * (frame.cols) + b.get();

			}
		}
		
		if(k.unset || b.unset){
			cout<<"error:0"<<endl;
			error = 0;
		}
		else{
			error = get_error(k.get(),b.get(),frame);
			cout<<"error:"<<error<<endl;
		}
		 
		if(countNonZero(max)>=100){
				cv::line(frame, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
				cntl.pid(error);
				cntl.speed_up();
		}
		else{
			k.clear();
			b.clear();
			cntl.speed_down();
		}
		
		//////////////////updata_serial//////////////////
		if(debug_flag==0)		
			cntl.serial_update(fd);
		cntl.annotate(frame,frame_ori);
		///////////////////////////////////////////
		// imshow("his",max);
		// imshow("min_thre",min);
		// imshow("com_thre",max);
		// imshow("label",labeled_area);
 
		imshow("cap",frame_ori);
		key = waitKey(1);
		if(key=='q') 
			break;
		if(key=='s'){
			name = base+ std::to_string(i)+".ppm";
			imwrite(name,frame);
			name = base+ std::to_string(i)+".pgm";
			imwrite(name,R);
			name = base+ std::to_string(i)+"skel"+".pgm";
			imwrite(name,skel);
			i++;
		}  
		if(key=='p'){
			cntl.clear();
			waitKey();
		}
		if(key=='d'){
			if(debug_flag){
				cntl.clear();
				debug_flag=0;
			}
			else
				debug_flag=1;
		}
    }
	return -1;
    
}
