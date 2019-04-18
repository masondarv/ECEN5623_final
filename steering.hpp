#pragma once

#include <list>
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace cv;
using namespace std;
int steer_convert(double steer_original){
	if( (int)steer_original >=32760)
		return 32760;
	else if( (int)steer_original <= -32760)
		return -32760;
	else
		return (int)steer_original;
		
}

class steering {
private:
	double intergal; // [0;1] less = more stable, more = less stable
	double pre;
    double i_param;
	double p_param;
	double d_param;
	double steer_original;
	int steering_control;
	unsigned char throttle;
	unsigned char braking;
	unsigned char max_throttle;
	bool unset;
	bool annotate_flag;
	int frame_cnt;
public:
    steering(double p, double i, double d,unsigned char max) {
		this->intergal = 0;
		this->steering_control = 0;
		this->pre =0;
		this->i_param = i;
		this->p_param = p;
		this->d_param = d;
		this-> max_throttle =max;
		this->frame_cnt =0;
		unset = true;
    }

	void clear() {
		unset = true;
		steering_control =0;
		braking =0;
		throttle =0;
	}
	
	void set_param(double p, double i, double d){
		i_param = i;
		p_param = p;
		d_param = d;
	}
	
	void set_max_throttle(unsigned char max){
		max_throttle = max;
	}
	
    void pid(double error) {
        if (unset) {
            this->intergal = 0;
			this->pre = error;
			unset = false;
        }
		intergal += error;
        steer_original = d_param*(error-pre) + p_param * error + intergal * i_param;
		steering_control = steer_convert(steer_original);
		pre = error;
    }
	
	void speed_up(){
		throttle = max_throttle;
		braking = 0;
		
	}
	
	void speed_down(){
		throttle = 0;
		braking =0xe0;
		steer_original *=32768;
		unset = true;
	}	
	void serial_update(int fd){
		string data_stream ="!"+to_string(steering_control)+"@"+to_string(throttle)+"#"+to_string(braking)+"$\n";
		int wlen = write(fd, data_stream.c_str(), data_stream.size());
		if (wlen != data_stream.size()) {
			printf("Error from write: %d, %d\n", wlen, errno);
		}
	}
	
	void annotate(Mat& src,Mat& src_ori){
		line(src,Point(120,211),Point(  (int)(((float)steering_control/32768.0)*100.0+120.0),211),Scalar(0,0,255),10);
		line(src,Point(120,200),Point( 120,222),Scalar(255,255,255),2);
		
		if(throttle>0)
			putText(src,"throttle",Point(385,211),FONT_HERSHEY_COMPLEX,1.0,Scalar(0,0,255),2);
		else if(braking>0)	
			putText(src,"braking",Point(385,211),FONT_HERSHEY_COMPLEX,1.0,Scalar(0,0,255),2);
		string name ="cap"+to_string(frame_cnt)+".ppm";
		imwrite(name,src_ori);
		frame_cnt++;
	}
	

};

