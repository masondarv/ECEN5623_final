
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
// #include "steering.hpp"
// #include <opencv2/core/core.hpp>
// #include <opencv2/core/core_c.h>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui/highgui_c.h>

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

//semaphores used to sequence the capture, processing, and control threads, and synchronize data sharing betweeen them
sem_t semCapture, semProcess, semControl, semShow, semRGB, semLine;
struct timeval start_time_val;

// using namespace cv;
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

// double k_to_theta(double k){
// 	return atan (k) * 180 / PI;
// }

// double get_error (double k,double b,Mat& image){
// 	double pos1 = (-b/k) - (double)(image.cols/2.0f);
// 	double pos2 = ((double)image.rows-b)/k - (double)(image.cols/2.0f);
// 	return pos1*0.7+pos2*0.3;
// }
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

// int set_interface_attribs(int fd, int speed)
// {
//     struct termios tty;
//
//     if (tcgetattr(fd, &tty) < 0) {
//         printf("Error from tcgetattr: %s\n", strerror(errno));
//         return -1;
//     }
//
//     cfsetospeed(&tty, (speed_t)speed);
//     cfsetispeed(&tty, (speed_t)speed);
//
//     tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8;         /* 8-bit characters */
//     tty.c_cflag &= ~PARENB;     /* no parity bit */
//     tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
//     tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */
//
//     /* setup for non-canonical mode */
//     tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
//     tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
//     tty.c_oflag &= ~OPOST;
//
//     /* fetch bytes as they become available */
//     tty.c_cc[VMIN] = 1;
//     tty.c_cc[VTIME] = 1;
//
//     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//         printf("Error from tcsetattr: %s\n", strerror(errno));
//         return -1;
//     }
//     return 0;
// }

void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

void *Sequencer(void *threadp)
{
	struct timeval current_time_val;
	struct timespec delay_time = {0,33333333}; // delay for 33.33 msec, 30 Hz
	struct timespec remaining_time;
	double residual;
	int rc, delay_cnt=0;
	unsigned long long seqCnt=0;

	printf("Sequencer thread started\n");

	//post the RGB and line semaphores initially to unlock the capture and image processing threads
	sem_post(&semRGB);
	sem_post(&semLine);

	while(1)
	{
		do
		{
				rc=nanosleep(&delay_time, &remaining_time);

				if(rc == EINTR)
				{
						residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

						if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);

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
		printf("Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

		//sequence the capture and process threads at a frequency of 30 Hz
		sem_post(&semCapture);
		sem_post(&semProcess);


		//post the control thread at a frequency of 10 Hz
		if((seqCnt % 3) == 0) sem_post(&semControl);

		//post the image show thread at a frequency of 10 Hz
		if((seqCnt % 3) == 0) sem_post(&semShow);

	}

	pthread_exit((void *)0);



}

void *Capture_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms

	printf("Capture thread started\n");

	while(1)
	{
		sem_wait(&semCapture);
		gettimeofday(&startTime, (struct timezone *)0);
		printf("Capture Frame release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Capture Frame release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);




		//once the shared RGB data structure most be updated, wait for the semaphore to become available
		sem_wait(&semRGB);






		//after the RGB data structure has been updated, post the sempahore
		sem_post(&semRGB);

	}

	pthread_exit((void *)0);

}

void *ImgProc_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms

	printf("Image Processing thread started\n");

	while(1)
	{
		sem_wait(&semProcess);
		gettimeofday(&startTime, (struct timezone *)0);
		printf("Image Processing release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Image Processing release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);



		//once the shared RGB data structure most be read and used, wait for the semaphore to become available
		sem_wait(&semRGB);




		//after the RGB data structure has been read and used, post the sempaphore
		sem_post(&semRGB);



		//once the shared optimal path line data structure most be updated, wait for the semaphore to become available
		sem_wait(&semLine);




		//after the optimal path line data structure has been updated, post the sempahore
		sem_post(&semLine);



	}

	pthread_exit((void *)0);

}

void *Control_Service(void *threadp)
{
	struct timeval startTime;
	struct timeval finishTime;
	unsigned long C_us;				//execution time in us
	uint32_t C_ms;						//execution time in ms

	printf("Control thread started\n");

	while(1)
	{
		sem_wait(&semControl);
		gettimeofday(&startTime, (struct timezone *)0);
		printf("Control release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Control release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);



		//once the shared optimal path line data structure most be read and used, wait for the semaphore to become available
		sem_wait(&semLine);




		//after the optimal path line data structure has been read and used, post the sempahore
		sem_post(&semLine);



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

	printf("Image show thread started\n");

	while(1)
	{
		sem_wait(&semShow);
		gettimeofday(&startTime, (struct timezone *)0);
		printf("Image show release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);
		syslog(LOG_CRIT, "Image show release @ sec=%d, msec=%d\n", (int)(startTime.tv_sec-start_time_val.tv_sec), (int)startTime.tv_usec/USEC_PER_MSEC);





	}

	pthread_exit((void *)0);

}




int main( int argc, char** argv )
{
	struct timeval current_time_val;
	int i, rc, scope;
	cpu_set_t threadcpu;
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
	if (sem_init (&semRGB, 0, 0)) { printf ("Failed to initialize process semaphore\n"); exit (-1); }
	if (sem_init (&semLine, 0, 0)) { printf ("Failed to initialize control semaphore\n"); exit (-1); }


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


	//configure sequencer thread
	rc=pthread_attr_init(&sequencer_sched_attr);
	rc=pthread_attr_setinheritsched(&sequencer_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&sequencer_sched_attr, SCHED_FIFO);

	sequencer_sched_param.sched_priority=rt_max_prio;
	pthread_attr_setschedparam(&sequencer_sched_attr, &sequencer_sched_param);

	//configure image capture thread
	rc=pthread_attr_init(&capture_sched_attr);
	rc=pthread_attr_setinheritsched(&capture_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&capture_sched_attr, SCHED_FIFO);

	capture_sched_param.sched_priority=rt_max_prio-1;
	pthread_attr_setschedparam(&capture_sched_attr, &capture_sched_param);

	//configure image processing thread
	rc=pthread_attr_init(&process_sched_attr);
	rc=pthread_attr_setinheritsched(&process_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&process_sched_attr, SCHED_FIFO);

	process_sched_param.sched_priority=rt_max_prio-2;
	pthread_attr_setschedparam(&process_sched_attr, &process_sched_param);

	//configure closed-loop control thread
	rc=pthread_attr_init(&control_sched_attr);
	rc=pthread_attr_setinheritsched(&control_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&control_sched_attr, SCHED_FIFO);

	control_sched_param.sched_priority=rt_max_prio-3;
	pthread_attr_setschedparam(&control_sched_attr, &control_sched_param);

	//configure closed-loop control thread
	rc=pthread_attr_init(&show_sched_attr);
	rc=pthread_attr_setinheritsched(&show_sched_attr, PTHREAD_EXPLICIT_SCHED);
	rc=pthread_attr_setschedpolicy(&show_sched_attr, SCHED_FIFO);

	control_sched_param.sched_priority=rt_max_prio-4;
	pthread_attr_setschedparam(&show_sched_attr, &show_sched_param);


	rc=pthread_create(&capture_thread,               // pointer to thread descriptor
										&capture_sched_attr,         // use specific attributes
										Capture_Service,                 // thread function entry point
										NULL 												// parameters to pass in
									 );
	 if(rc < 0)
			 perror("pthread_create for capture service");
	 else
			 printf("pthread_create successful for capture service\n");

	 rc=pthread_create(&process_thread,               // pointer to thread descriptor
 										&process_sched_attr,         // use specific attributes
 										ImgProc_Service,                 // thread function entry point
 										NULL 												// parameters to pass in
 									 );
	 if(rc < 0)
			 perror("pthread_create for image processing service");
	 else
			 printf("pthread_create successful for image processing service\n");
	 rc=pthread_create(&control_thread,               // pointer to thread descriptor
 										&control_sched_attr,         // use specific attributes
 										Control_Service,                 // thread function entry point
 										NULL 												// parameters to pass in
 									 );

	 if(rc < 0)
			 perror("pthread_create for control service");
	 else
			 printf("pthread_create successful for control service\n");

	 rc=pthread_create(&show_thread,               // pointer to thread descriptor
										 &show_sched_attr,         // use specific attributes
										 showProc_Service,                 // thread function entry point
										 NULL 												// parameters to pass in
										);

	if(rc < 0)
			perror("pthread_create for control service");
	else
			printf("pthread_create successful for control service\n");

	 rc=pthread_create(&sequencer_thread,               // pointer to thread descriptor
											&sequencer_sched_attr,         // use specific attributes
											Sequencer,                 // thread function entry point
											NULL 												// parameters to pass in
										 );

	 if(rc < 0)
			 perror("pthread_create for sequencer service");
	 else
			 printf("pthread_create successful for sequencer service\n");

	 pthread_join(capture_thread, NULL);
	 pthread_join(process_thread, NULL);
	 pthread_join(control_thread, NULL);
	 pthread_join(show_thread, NULL);
	 pthread_join(sequencer_thread, NULL);



	return -1;

}
