#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cv_face_internal.h>
#include "time_helper.h"
#include "model_helper.hpp"
#include <time.h>
#include <opencv2/opencv.hpp>
#include<thread>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <fstream>
#ifndef USE_SHARED_INTERNAL_SDK
#include <internal_engine_reg.h>
#ifdef CONFIG_SDK_MODEL_CUTTABLE
#include <cv_common_image_funcs_reg.hpp>
#include <protector_reg.hpp>
#endif
#endif
using namespace std;
using namespace cv;

struct termio

{   unsigned short  c_iflag; /* 输入模式标志 */ 
    unsigned short  c_oflag;     /* 输出模式标志 */ 
    unsigned short  c_cflag;     /* 控制模式标志*/  
    unsigned short  c_lflag;     /* local mode flags */  
    unsigned char   c_line;           /* line discipline */
    unsigned char   c_cc[515];    /* control characters */
};

int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                  B38400, B19200, B9600, B4800, B2400, B1200, B300, };

int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 
                  19200,  9600, 4800, 2400, 1200,  300, };

void set_speed(int fd, int speed){
    int i;
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for( i= 0;  i < sizeof(speed_arr) / sizeof(int);i++) {
		if (speed == name_arr[i]) {
			tcflush(fd, TCIOFLUSH);//设置前flush    
			cfsetispeed(&Opt, speed_arr[i]); 
			cfsetospeed(&Opt, speed_arr[i]);  
			status = tcsetattr(fd, TCSANOW, &Opt); 
			if  (status != 0){       
			  perror("tcsetattr fd1"); 
			  return;
			}
			tcflush(fd,TCIOFLUSH);  //设置后flush
		}
	}
}

int set_Parity(int fd, int databits, int stopbits, int parity){
    struct termios options;
    if (tcgetattr(fd, &options) != 0){
       perror("SetupSerial 1");
       return -1;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits){
		case 7:
		   options.c_cflag |= CS7;
		   break;
		case 8:
		   options.c_cflag |= CS8;
		   break;
		default:
		   fprintf(stderr,"Unsupported data size\n");
		   return -1;
    }
    switch (parity){
		case 'n':
		case 'N':
		   options.c_cflag &= ~PARENB; /* Clear parity enable */
		   options.c_iflag &= ~INPCK; /* Enable parity checking */
		   break;
		case 'o':
		case 'O':
		   options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		   options.c_iflag |= INPCK; /* Disnable parity checking */
		   break;
		case 'e':
		case 'E':
		   options.c_cflag |= PARENB; /* Enable parity */
		   options.c_cflag &= ~PARODD; /* 转换为偶效验*/
		   options.c_iflag |= INPCK; /* Disnable parity checking */
		   break;
		case 'S':
		case 's': /*as no parity*/
		   options.c_cflag &= ~PARENB;
		   options.c_cflag &= ~CSTOPB;
		   break;
		default:
		   fprintf(stderr,"Unsupported parity\n");
		   return -1;
    }
    /* 设置停止位*/
    switch (stopbits){
		case 1:
		   options.c_cflag &= ~CSTOPB;
		   break;
		case 2:
		   options.c_cflag |= CSTOPB;
		   break;
		default:
		   fprintf(stderr,"Unsupported stop bits\n");
		   return -1;
    }
    /* Set input parity option */
    if (parity != 'n')
       options.c_iflag |= INPCK;
       
    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
       perror("SetupSerial 3");
       return -1;
    }
    return 0;
}

void setTermios(struct termios * pNewtio, int uBaudRate)
{
	bzero(pNewtio, sizeof(struct termios)); /* clear struct for new port settings */
	//8N1
	pNewtio->c_cflag = uBaudRate | CS8 | CREAD | CLOCAL;
	pNewtio->c_iflag = IGNPAR;
	pNewtio->c_oflag = 0;
	pNewtio->c_lflag = 0; //non ICANON
	pNewtio->c_cc[VINTR] = 0; /* Ctrl-c */
	pNewtio->c_cc[VQUIT] = 0; /* Ctrl-\ */
	pNewtio->c_cc[VERASE] = 0; /* del */
	pNewtio->c_cc[VKILL] = 0; /* @ */
	pNewtio->c_cc[VEOF] = 4; /* Ctrl-d */
	pNewtio->c_cc[VTIME] = 5; /* inter-character timer, timeout VTIME*0.1 */
	pNewtio->c_cc[VMIN] = 0; /* blocking read until VMIN character arrives */
	pNewtio->c_cc[VSWTC] = 0; /* '\0' */
	pNewtio->c_cc[VSTART] = 0; /* Ctrl-q */
	pNewtio->c_cc[VSTOP] = 0; /* Ctrl-s */
	pNewtio->c_cc[VSUSP] = 0; /* Ctrl-z */
	pNewtio->c_cc[VEOL] = 0; /* '\0' */
	pNewtio->c_cc[VREPRINT] = 0; /* Ctrl-r */
	pNewtio->c_cc[VDISCARD] = 0; /* Ctrl-u */
	pNewtio->c_cc[VWERASE] = 0; /* Ctrl-w */
	pNewtio->c_cc[VLNEXT] = 0; /* Ctrl-v */
	pNewtio->c_cc[VEOL2] = 0; /* '\0' */
}

void sendData(int a,int b,int c,int d){
	int fd;
	int nCount, nTotal, i;
	struct termios oldtio, newtio;
	stringstream aa,bb,cc,dd;
	aa<<a;
	bb<<b;
	cc<<c;
	dd<<d; 
	string sa = aa.str();
	string sb = bb.str();
	string sc = cc.str();
	string sd = dd.str();
	string s1 = "A"+sa+"B"+sb+"C"+sc+"D"+sd+"T10EN";
	
	int len = s1.length();
	char *buffer;
	buffer = (char *)malloc((len+1)*sizeof(char));
	s1.copy(buffer,len,0);
	char *dev;
	string devstr ="/dev/ttyS0";
	len = devstr.length();
	dev = (char *)malloc((len+1)*sizeof(char));
	devstr.copy(dev,len,0);
	if ((fd = open(dev, O_RDWR |O_NDELAY | O_NONBLOCK))<0)
	{
		printf("err: can't open serial port!\n");
	}

	tcgetattr(fd, &oldtio); /* save current serial port settings */
	setTermios(&newtio, B9600);
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
	nCount=write(fd, buffer, strlen(buffer));
	printf("send data\n");
	tcsetattr(fd, TCSANOW, &oldtio);
	//close(fd);
}

bool openCamera(cv::VideoCapture &cap, const int cameraDrive)
{
	cap.open(cameraDrive);
	if (!cap.isOpened()) {
		printf("The camera is not opened.\n");
		return false;
	}
	else {
		printf("The %dth camera is opened.\n", cameraDrive + 1);

		printf("The %dth camera is opened.\n", cameraDrive + 1);
		// 设置相机捕获画面尺寸大小
		//cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
 		printf("The %dth camera is opened.\n", cameraDrive + 1);
		// 曝光   min:-8  max:0
		cap.set(CV_CAP_PROP_EXPOSURE, (-4));
		// 帧数   30 60
		cap.set(CV_CAP_PROP_FPS, 60);
		// 亮度    min:-64  max:-60
		//cap.set(CV_CAP_PROP_BRIGHTNESS, (-brightness));  
		// 对比度  min:0  max:100
		//cap.set(CV_CAP_PROP_CONTRAST, 60);
		// 饱和度  min:0  max:128
		//cap.set(CV_CAP_PROP_SATURATION, 50);
		// 色调   min:-180  max:180
		//cap.set(CV_CAP_PROP_HUE, hue); 
		
		return true;
	}
}
 
bool readCamera(cv::VideoCapture &cap, cv::Mat &src)
{
	cap.read(src);
	usleep(150000);
	if (src.empty()) {
		printf("The frame is empty.\n");
		return false;
	}
	return true;
}

int main(int argc, char *argv[]) {
	AddLicense();
	int main_return = -1;
	int video = 0;
	if (video == 0){
	cv_handle_t handle_detect = NULL, handle_align = NULL;
	cv_detection_result_t *results = nullptr;
	int count = 0;
	cv_result_t st_result = CV_OK;
	cv::Mat frame_1, frame_2;
	cv::VideoCapture capture_1, capture_2;
	const int cameraDrive1 = 0;
	// VideoCapture cap(0);
	std::thread extraT(openCamera, std::ref(capture_1), cameraDrive1);
	std::cout << "there 1";
	if (!openCamera(capture_1, cameraDrive1))
		return 0; 
	extraT.detach(); 
	std::cout << "there 2";
	// init detect handle
	cv_model_t model_spider;
	cv_common_load_model(model_path(m_detect_spider).c_str(), &model_spider);
	cv_common_detection_spider_create(model_spider, &handle_detect);
	if (!handle_detect) {
		cv_common_unload_model(model_spider);

		fprintf(stderr, "fail to init model_spider handle\n");
		return -1;
	}

	fprintf(stderr, "success to create spider detect handler\n");
	clock_t start, finish;
	double duration;
	std::cout << "there 3";
	
	for(;;){
			// load image
		std::cout << "there 4";
		if (!readCamera(capture_1, frame_1))
			break;
		std::cout << "there 5";
		start = clock();
		Mat image_gray;
		Mat image_color;
		
		// Size dsize = Size(640,480);		
		resize(frame_1,image_color,Size(),0.65,0.65,INTER_AREA);
		if (!image_color.data) {
			fprintf(stderr, "fail to read %s\n", argv[1]);
			return -1;
		}
		cvtColor(image_color, image_gray, CV_BGR2GRAY);


		cv_image_t img = { image_gray.data, CV_PIX_FMT_GRAY8, image_gray.cols, image_gray.rows, image_gray.cols };
		// detect
		st_result = cv_common_detection_spider_detect(handle_detect, &img, &results, &count);
		if (st_result != CV_OK) {
			cv_common_detection_spider_destroy(handle_detect);
			cv_common_unload_model(model_spider);

			fprintf(stderr, "failed to detect face\n");
			return -1;
		}

		fprintf(stderr, "success to detect %d faces\n", count);



		// init align handle
		cv_model_t model_align;
		cv_common_load_model(model_path(m_align_3d).c_str(), &model_align);
		ppl_context_t ppl_context = {1, 0, ppl_context_t::InferFastest};
		//cv_model_net_config_t align_config = {ST_CUDA, &ppl_context};
		//cv_common_set_model_config(model_align, nullptr, align_config);

		cv_common_alignment_3d_create(model_align, &handle_align);
		if (!handle_align) {
			cv_common_unload_model(model_align);
			cv_common_detection_spider_release_result(results, count);

			fprintf(stderr, "fail to init align_3d handle\n");
			return -1;
		}

		fprintf(stderr, "success to init deep align handle\n");

		int points_num = -1;
		cv_common_alignment_get_points_count(handle_align, &points_num);
		fprintf(stderr, "the alignment points number is: %d \n", points_num);
		float result[106][2] = {0};
		if (count > 0) {
	//		for (int k=0;k<10000;k++) {
				// draw result
				for (int i = 0; i < count; i++) {
					rectangle(image_color, Point(results[i].rect.left, results[i].rect.top),
						  Point(results[i].rect.right, results[i].rect.bottom),
						  Scalar(0, 255, 0), 2, 8, 0);
					fprintf(stderr, "face number: %d\n", i);
					fprintf(stderr, "face rect: [%d, %d, %d, %d]\n",
						results[i].rect.left, results[i].rect.top,	results[i].rect.right, results[i].rect.bottom);
					fprintf(stderr, "score: %f\n", results[i].score);
					// alignment
					cv_landmarks_t* lms = nullptr, *lms2 = nullptr;
					__TIC__();
					st_result = cv_common_alignment_align_by_rect(handle_align, &img, &results[i].rect, 1, 0, &lms);
					__TOC__();
					if (st_result != CV_OK) {
						fprintf(stderr, "failed to align\n");
						main_return = -1;
						break;
					}

					fprintf(stderr, "align by rect score: %f\n", lms[0].score);

					for(int k=0;k<lms[0].points_count;k++) {
						//printf("%d (%.2f, %.2f, %.2f)\n", k, lms[0].points_array[k].x, lms[0].points_array[k].y, lms[0].points_array[k].z);
					}

					if (CV_OK != cv_common_alignment_align_by_pose(handle_align, &img, lms, 1, &lms2)){
						fprintf(stderr, "failed to align by pose\n");
					}

					fprintf(stderr, "align by pose score: %f\n", lms2[0].score);
					float min_depth = 1e8, max_depth = -1;
					for (int k = 0; k < lms2[0].points_count; k++) {
						float depth = lms2[0].points_array[k].z;
						max_depth = max_depth > depth ? max_depth : depth;
						min_depth = min_depth < depth ? min_depth : depth;
					}
					float inside_min_depth = 1e8, inside_max_depth = -1;
					for (int k = 33; k < lms2[0].points_count; k++) {
						float depth = lms2[0].points_array[k].z;
						inside_max_depth = inside_max_depth > depth ? inside_max_depth : depth;
						inside_min_depth = inside_min_depth < depth ? inside_min_depth : depth;
					}
					for(int k=0;k<lms2[0].points_count;k++) {
						int color = 255 - floor(255.0 * (lms2[0].points_array[k].z - min_depth)/(max_depth - min_depth));
						int rcolor = 0;
						if (k >= 33) {
							rcolor = 255 - floor(255.0 * (lms2[0].points_array[k].z - inside_min_depth)/(inside_max_depth - inside_min_depth));
						}
						int iii = 0;
						circle(image_color, Point2f(lms2[0].points_array[k].x, lms2[0].points_array[k].y), 2.5f, Scalar(0, color, rcolor), 1);
						//printf("%d (%.2f, %.2f, %.2f)\n", k, lms2[0].points_array[k].x, lms2[0].points_array[k].y, lms2[0].points_array[k].z);
						result[k][0] = lms2[0].points_array[k].x;
						result[k][1] = lms2[0].points_array[k].y;
					}

					cv_common_alignment_release_result(lms, 1);
					cv_common_alignment_release_result(lms2, 1);
				}
	//		}
			
			main_return = 0;  // success
		} else {
			fprintf(stderr, "can't find face in %s\n", argv[1]);
		}

		imshow("face",image_color);
		char c = (char)waitKey(1);
		if(c == 32){
			
			if(result != nullptr){
				ofstream outfile;
				outfile.open("myfile.txt");
				for(int i = 0;i<106;i++)
					outfile<<result[i][0]<<" "<<result[i][1]<<endl;
				outfile.close(); 
				for(int i = 0;i<106;i++){
					printf("%.2f ,%.2f \n" ,result[i][0],result[i][1]);
				}
			}
			while(1){
				char bb = (char)waitKey(1);
				if(bb == 32)
					break;
			}
		}
		if (c == 27)
			break;
		finish = clock();
		duration = double(finish - start) / CLOCKS_PER_SEC;
		printf("*****************%f seconds\n\n",duration);
	}


	fprintf(stderr, "test finish!\n");
	}
	return main_return;
}
