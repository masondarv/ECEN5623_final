
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <errno.h>
#include <fcntl.h>

// #include <termios.h>

#define PI 3.14159265

#include <iostream>
#include <string>     // std::string, std::to_string
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2//imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>

#include "steering.hpp"
#include "util.hpp"
#include "global.hpp"

#define K_VARY_FACTOR 30
#define B_VARY_FACTOR 500

#define USEC_PER_MSEC (1000)
#define USEC_PER_SEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (4+1)

//semaphores used to sequence the capture, processing, and control threads, and synchronize data sharing betweeen them
sem_t semCapture, semProcess, semControl, semShow, semRGB, semLine,semCntl;
struct timeval start_time_val;

using namespace cv;
using namespace std;

//thread used to control the timing of the threads that implement the application
void *Sequencer(void *threadp);

//performs image capture and preprocessing
void *Capture_Service(void *threadp);
//Performs the bulk of image processing used to find the optimal path through the course
void *ImgProc_Service(void *threadp);
//Implements closed-loop control of the race car and sends control signals to the video game
void *Control_Service(void *threadp);

//show the result of image processing
void *showProc_Service(void *threadp);


void *Sequencer(void *threadp)
{
	struct timeval current_time_val;
	struct timespec delay_time = {0,33333333}; // delay for 33.33 msec, 30 Hz
	struct timespec remaining_time;
	double residual;
	int rc, delay_cnt=0;
	unsigned long long seqCnt=0;

	//-printf("Sequencer thread started\n");


	while(1)
	{
		do
		{
				rc=nanosleep(&delay_time, &remaining_time);

				if(rc == EINTR)
				{
						residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

						if(residual > 0.0) //-printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);

						delay_cnt++;
				}
				else if(rc < 0)
				{
						perror("Sequencer nanosleep");
						exit(-1);
				}

		} while((residual > 0.0) && (delay_cnt < 100));

		seqCnt++;

		gettimeofday(&current_time_val, (struct timezone *)0);
		//-printf("Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

		//sequence the capture and process threads at a frequency of 30 Hz
		//post the capture thread at a frequency of 30 Hz
		sem_post(&semCapture);
		sem_post(&semProcess);

		//post the control thread at a frequency of 10 Hz
		if((seqCnt % 3) == 0) sem_post(&semControl);

		//post the image show thread at a frequency of 5 Hz
		if((seqCnt % 6) == 0) sem_post(&semShow);

	}

	pthread_exit((void *)0);



}

void *Capture_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	struct timeval currTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms
	Mat frame_original,fram_roi;
	//RGB_shared.resize(3);
	////-printf("Capture thread started\n");
	while(1)
	{
		sem_wait(&semCapture);
		
		////-printf("Capture Frame release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Capture Frame release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		
		//once the shared RGB data structure most be updated, wait for the semaphore to become available
		sem_wait(&semRGB);
		gettimeofday(&startTime, (struct timezone *)0);
		
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "Capture semRGB taken @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		/*service code*/
		cap >> frame_original;
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "Frame Captured @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		
        if(frame_original.empty()) break;
		fram_roi = frame_original.rowRange(FOV_CUT_UPPERBOUND,FOV_HEIGHT);
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "ImageCut @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		medianBlur(fram_roi,fram_roi,3);
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "medianBlur @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);





		
		split(fram_roi,RGB_shared);
		frame_original.copyTo(frame);
		//after the RGB data structure has been updated, post the sempahore
		sem_post(&semRGB);
		
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "Capture semRGB posted @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		
		//imshow("line",frame_original);
		//waitKey(10);
		/*service code end*/
		
		gettimeofday(&finishTime, (struct timezone *)0);
		
		if((finishTime.tv_sec- startTime.tv_sec)>0)
			C_us = (USEC_PER_SEC - startTime.tv_usec)+ finishTime.tv_usec;
		else
			C_us = finishTime.tv_usec - startTime.tv_usec;
			
		C_ms = C_us / USEC_PER_MSEC;
		syslog(LOG_CRIT, "Capture Frame C = %lu us or %d ms", C_us, C_ms);


	
	}

	pthread_exit((void *)0);

}

void *ImgProc_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	struct timeval currTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms
	
	//-printf("Image Processing thread started\n");
	
	Mat G,B,R;
	Mat max,min,inter,comp;
	
	vector<Mat> RGB;
	RGB.resize(3);
	for(int i=0;i<3;i++)
		RGB[i].create(ROI,CV_8UC1);
	Mat element = (Mat_<uchar>(3,3) << 0, 0, 0, 1, 1, 1, 0, 0, 0);
	//////////////////////////////////////////
	Mat label,stats,centroids;
	vector<Point> thre_points;
	vector<Vec4i> lines_labeled;
	Vec4f line_para; 
	int area_cnt=0;
	double k_current,k_pre;
	double b_current,b_pre;	
	Point point0;
	while(1)
	{
		sem_wait(&semProcess);
		
		////-printf("Image Processing release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Image Processing release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);



		//once the shared RGB data structure most be read and used, wait for the semaphore to become available
		sem_wait(&semRGB);
		gettimeofday(&startTime, (struct timezone *)0);
		
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "ImgProc semRGB take @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		
		for(int i=0;i<3;i++)
			RGB_shared[i].copyTo(RGB[i]);

		//after the RGB data structure has been read and used, post the sempaphore
		sem_post(&semRGB);
		gettimeofday(&currTime, (struct timezone *)0);
		syslog(LOG_CRIT, "ImgProc semRGB post @ sec=%d, msec=%d\n", (int)(currTime.tv_sec-start_time_val.tv_sec), (int)currTime.tv_usec/USEC_PER_MSEC);
		
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
		//imshow("max_thre",RGB[1]);
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
		blue_arrow_cnt = countNonZero(max);
		//imshow("max_thre",max);
		//waitKey(1);
		
		connectedComponentsWithStats(max,label,stats,centroids,8);
		area_cnt =0;
		thre_points.clear();
		for(int i=1;i<centroids.rows;i++){
				
			if(stats.at<int>(i,CC_STAT_AREA) >= 30){
				area_cnt++;
				//circle( labeled_area, Point((int)centroids.at<double>(i,0),(int)centroids.at<double>(i,1)),   2, Scalar(255,0,0), 1 );
				thre_points.push_back(Point((int)centroids.at<double>(i,0),(int)centroids.at<double>(i,1)));
			}	
		}
		
		if(area_cnt >=2){
			fitLine(thre_points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
			point0.x = line_para[2];
			point0.y = line_para[3];

			
			
			
			if(  (fabs(line_para[1])<=1) && (fabs(line_para[1])>=0.25)  ){
				k_current = line_para[1] / line_para[0];
				b_current = k_current * (0 - point0.x) + point0.y;
				
				
				double k_diff = fabs(k_to_theta(k_current) - k_to_theta(k.get()));
				double b_diff = fabs(b_current - b.get());
				
				bool update_ok = (k_diff <= K_VARY_FACTOR && b_diff <= B_VARY_FACTOR || (k.unset || b.unset)) ;
				//once the shared optimal path line data structure most be updated, wait for the semaphore to become available
				sem_wait(&semLine);
				if(blue_arrow_cnt <MIN_BLUE_DETECTED_TO_SPEEDUP){
					k.clear();
					b.clear();
				}
				else if(update_ok){
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
				//after the optimal path line data structure has been updated, post the sempahore
				sem_post(&semLine);
				k_pre = k_current;
				b_pre = b_current;
			}
			
			
		}
		



		gettimeofday(&finishTime, (struct timezone *)0);
		
		if((finishTime.tv_sec- startTime.tv_sec)>0)
			C_us = (USEC_PER_SEC - startTime.tv_usec)+ finishTime.tv_usec;
		else
			C_us = finishTime.tv_usec - startTime.tv_usec;
			
		C_ms = C_us / USEC_PER_MSEC;
		syslog(LOG_CRIT, "ImgProc C = %lu us or %d ms", C_us, C_ms);

		



	}
	pthread_exit((void *)0);

}

void *Control_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms
	Mat frame_roi;
	double k_current,b_current,error;
	bool unset;
	steering cntl_temp;
	//-printf("Control thread started\n");
	while(1)
	{
		sem_wait(&semControl);
		gettimeofday(&startTime, (struct timezone *)0);
		////-printf("Control release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Control release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);


		point1.x=0;
		//once the shared optimal path line data structure most be read and used, wait for the semaphore to become available
		sem_wait(&semLine);
		k_current=k.get();
		b_current=b.get();
		unset = k.unset || b.unset;
		//after the optimal path line data structure has been read and used, post the sempahore
		sem_post(&semLine);
		
		
		if(unset){
			cout<<"error:0"<<endl;
			error = 0;
		}
		else{
			error = get_error(k_current,b_current);
			cout<<"error:"<<error<<endl;
		}
		
		sem_wait(&semCntl);
		point1.y= b_current;
		point2.x = FOV_WIDTH;
		point2.y = k_current* (FOV_WIDTH) + b_current;
		if(blue_arrow_cnt>=MIN_BLUE_DETECTED_TO_SPEEDUP){
			cntl.pid(error);
			cntl.speed_up();
		}
		else{
			cntl.speed_down();
		}
		cntl_temp = cntl;
		sem_post(&semCntl);
		//cout<<(int)cntl_temp.get_throttle()<<endl;
		cntl_temp.serial_update(fd);
		
		//imshow("line",frame);
		//waitKey(1);
		/*service code end*/
		
		gettimeofday(&finishTime, (struct timezone *)0);
		
		if((finishTime.tv_sec- startTime.tv_sec)>0)
			C_us = (USEC_PER_SEC - startTime.tv_usec)+ finishTime.tv_usec;
		else
			C_us = finishTime.tv_usec - startTime.tv_usec;
			
		C_ms = C_us / USEC_PER_MSEC;
		syslog(LOG_CRIT, "Control C = %lu us or %d ms", C_us, C_ms);
		
		

	}

	pthread_exit((void *)0);

}

//show the output of the image processing service
void *showProc_Service(void *threadp)
{

	struct timeval startTime;
	struct timeval finishTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms
	Mat frame_temp,frame_roi;
	unsigned char key;
	//-printf("Image show thread started\n");

	while(1)
	{
		sem_wait(&semShow);
		gettimeofday(&startTime, (struct timezone *)0);
		////-printf("Image show release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Image show release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		//sem_wait(&semRGB);
		//after the RGB data structure has been updated, post the sempahore
		//frame.copyTo(frame_temp);
		//sem_post(&semRGB);
		if(blue_arrow_cnt>=MIN_BLUE_DETECTED_TO_SPEEDUP){
			frame_roi = frame.rowRange(FOV_CUT_UPPERBOUND,FOV_HEIGHT);
			line(frame_roi, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);	
		}
		//cout <<"blue arrow "<<blue_arrow_cnt << endl;
		sem_wait(&semCntl);
		//cntl.annotate(frame_roi);
		sem_post(&semCntl);
		imshow("line",frame);
		key=waitKey(1);
		if(key == 'q')
			exit(0);
			
			
		gettimeofday(&finishTime, (struct timezone *)0);
		
		if((finishTime.tv_sec- startTime.tv_sec)>0)
			C_us = (USEC_PER_SEC - startTime.tv_usec)+ finishTime.tv_usec;
		else
			C_us = finishTime.tv_usec - startTime.tv_usec;
			
		C_ms = C_us / USEC_PER_MSEC;
		syslog(LOG_CRIT, "ShowProc C = %lu us or %d ms", C_us, C_ms);
		

	}

	pthread_exit((void *)0);

}




int main( int argc, char** argv )
{

	gettimeofday(&start_time_val, (struct timezone *)0);
	syslog(LOG_CRIT, "Self-Drive Program Start @ sec=%d, msec=%d \n \n \n \n \n \n \n", (int)(start_time_val.tv_sec), (int)start_time_val.tv_usec/USEC_PER_MSEC);
	
	/*serial port declaration*/
	char *portname = "/dev/ttyUSB0";
	int wlen;
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		//-printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}		
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
	/*serial port declaration end*/
	
	/*camera initialization*/
	cap =  VideoCapture(atoi(argv[1]));
	// check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
	cout<<cap.get(CAP_PROP_FRAME_WIDTH)<<"  "<<cap.get(CAP_PROP_FRAME_HEIGHT)<<endl;
	/*camera initialization end*/
	ROI = Size(FOV_WIDTH, FOV_HEIGHT-FOV_CUT_UPPERBOUND);
	RGB_shared.resize(3);
	for(int i=0;i<3;i++)
		RGB_shared[i].create(ROI,CV_8UC1);
	frame.create(Size(FOV_WIDTH,FOV_HEIGHT),CV_8UC3);
	blue_arrow_cnt =0;
	cntl=steering(85.0f,0.0f,330.0f,95);
	/*set up pThreads*/
	struct timeval current_time_val;
	int i, rc, scope;
	pthread_t sequencer_thread, capture_thread, process_thread, control_thread, show_thread;
	pthread_attr_t sequencer_sched_attr, capture_sched_attr, process_sched_attr, control_sched_attr, show_sched_attr;
	int rt_max_prio, rt_min_prio;
	struct sched_param sequencer_sched_param, capture_sched_param, process_sched_param, control_sched_param, show_sched_param;
	struct sched_param main_param;
	pthread_attr_t main_attr;
	pid_t mainpid;


	if (sem_init (&semCapture, 0, 0)) { printf ("Failed to initialize capture semaphore\n"); exit (-1); }
	if (sem_init (&semProcess, 0, 0)) { printf ("Failed to initialize process semaphore\n"); exit (-1); }
	if (sem_init (&semControl, 0, 0)) { printf ("Failed to initialize control semaphore\n"); exit (-1); }
	if (sem_init (&semShow, 0, 0)) { printf ("Failed to initialize capture semaphore\n"); exit (-1); }
	if (sem_init (&semRGB, 0, 1)) { printf ("Failed to initialize process semaphore\n"); exit (-1); }
	if (sem_init (&semLine, 0, 1)) { printf ("Failed to initialize Line param semaphore\n"); exit (-1); }
	if (sem_init (&semCntl, 0, 1)) { printf ("Failed to initialize PID semaphore\n"); exit (-1); }


	mainpid=getpid();

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	rt_min_prio = sched_get_priority_min(SCHED_FIFO);

	rc=sched_getparam(mainpid, &main_param);
	main_param.sched_priority=rt_max_prio;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
	if(rc < 0) perror("main_param");

	pthread_attr_getscope(&main_attr, &scope);

	if(scope == PTHREAD_SCOPE_SYSTEM)
		printf("PTHREAD SCOPE SYSTEM\n");
	else if (scope == PTHREAD_SCOPE_PROCESS)
		printf("PTHREAD SCOPE PROCESS\n");
	else
		printf("PTHREAD SCOPE UNKNOWN\n");

	printf("rt_max_prio=%d\n", rt_max_prio);
	printf("rt_min_prio=%d\n", rt_min_prio);

	print_scheduler();

	//constrain the threads with only on CPU core.
	cpu_set_t threadcpu;
	CPU_ZERO(&threadcpu);
	CPU_SET(0, &threadcpu);
	CPU_SET(1, &threadcpu);
	CPU_SET(2, &threadcpu);
	//CPU_SET(3, &threadcpu);
	//configure sequencer thread
	rc=pthread_attr_init(&sequencer_sched_attr);
	rc=pthread_attr_setinheritsched(&sequencer_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&sequencer_sched_attr, SCHED_FIFO);

	sequencer_sched_param.sched_priority=rt_max_prio-1;
	pthread_attr_setschedparam(&sequencer_sched_attr, &sequencer_sched_param);
	pthread_attr_setaffinity_np(&sequencer_sched_attr, sizeof(cpu_set_t), &threadcpu);

	//configure image capture thread
	rc=pthread_attr_init(&capture_sched_attr);
	rc=pthread_attr_setinheritsched(&capture_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&capture_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&capture_sched_attr, sizeof(cpu_set_t), &threadcpu);

	capture_sched_param.sched_priority=rt_max_prio-2;
	pthread_attr_setschedparam(&capture_sched_attr, &capture_sched_param);

	//configure image processing thread
	rc=pthread_attr_init(&process_sched_attr);
	rc=pthread_attr_setinheritsched(&process_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&process_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&process_sched_attr, sizeof(cpu_set_t), &threadcpu);

	process_sched_param.sched_priority=rt_max_prio-3;
	pthread_attr_setschedparam(&process_sched_attr, &process_sched_param);

	//configure closed-loop control thread
	rc=pthread_attr_init(&control_sched_attr);
	rc=pthread_attr_setinheritsched(&control_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&control_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&control_sched_attr, sizeof(cpu_set_t), &threadcpu);

	control_sched_param.sched_priority=rt_max_prio-4;
	pthread_attr_setschedparam(&control_sched_attr, &control_sched_param);

	//configure closed-loop control thread
	rc=pthread_attr_init(&show_sched_attr);
	rc=pthread_attr_setinheritsched(&show_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&show_sched_attr, SCHED_FIFO);
	pthread_attr_setaffinity_np(&show_sched_attr, sizeof(cpu_set_t), &threadcpu);

	control_sched_param.sched_priority=rt_max_prio-5;
	pthread_attr_setschedparam(&show_sched_attr, &show_sched_param);


	rc=pthread_create(&capture_thread,               // pointer to thread descriptor
										&capture_sched_attr,         // use specific attributes
										Capture_Service,                 // thread function entry point
										NULL 												// parameters to pass in
									 );
	 if(rc < 0)
			 perror("pthread_create for capture service");
	 else
			 //-printf("pthread_create successful for capture service\n");

	 rc=pthread_create(&process_thread,               // pointer to thread descriptor
 										&process_sched_attr,         // use specific attributes
 										ImgProc_Service,                 // thread function entry point
 										NULL 												// parameters to pass in
 									 );
	 if(rc < 0)
			 perror("pthread_create for image processing service");
	 else
			 //-printf("pthread_create successful for image processing service\n");
	 rc=pthread_create(&control_thread,               // pointer to thread descriptor
 										&control_sched_attr,         // use specific attributes
 										Control_Service,                 // thread function entry point
 										NULL 												// parameters to pass in
 									 );

	 if(rc < 0)
			 perror("pthread_create for control service");
	 else
			 //-printf("pthread_create successful for control service\n");

	 rc=pthread_create(&show_thread,               // pointer to thread descriptor
										 &show_sched_attr,         // use specific attributes
										 showProc_Service,                 // thread function entry point
										 NULL 												// parameters to pass in
										);

	if(rc < 0)
			perror("pthread_create for control service");
	else
			//-printf("pthread_create successful for control service\n");

	 rc=pthread_create(&sequencer_thread,               // pointer to thread descriptor
											&sequencer_sched_attr,         // use specific attributes
											Sequencer,                 // thread function entry point
											NULL 												// parameters to pass in
										 );

	 if(rc < 0)
			 perror("pthread_create for sequencer service");
	 else
			 //-printf("pthread_create successful for sequencer service\n");

	 pthread_join(capture_thread, NULL);
	 pthread_join(process_thread, NULL);
	 pthread_join(control_thread, NULL);
	 pthread_join(show_thread, NULL);
	 pthread_join(sequencer_thread, NULL);



	return -1;

}
