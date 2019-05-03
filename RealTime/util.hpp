#ifndef  __UTIL__
#define  __UTIL__
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>

#include <termios.h>

#define PI 3.14159265

#include <iostream>
#include <string>     // std::string, std::to_string
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>


#define K_VARY_FACTOR 30
#define B_VARY_FACTOR 500

#define USEC_PER_MSEC (1000)
#define USEC_PER_SEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (4+1)

using namespace cv;
using namespace std;

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

int set_interface_attribs(int fd, int speed);
void print_scheduler(void);
double k_to_theta(double k);
double get_error (double k,double b,Mat& image);


#endif