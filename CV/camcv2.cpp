/*
 *  Пример захвата с камеры и вывод в окно для OpenCV версии 2  
 *  Заменил IplImage на Mat и команды, которые работают с Mat
 *  Выход из программы по нажатию на кнопку ESC
 
 Компилировать в Linux:

g++ `pkg-config --cflags opencv` -o camcv2 camcv2.cpp `pkg-config --libs opencv`
 
 */

#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <iostream>
using namespace cv;


int main(int argc, char** argv){

    VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Mat frame; 
  while(true){
	cap >> frame;
    //imwrite("file.jpg", frame);
    imshow("frame", frame);
    char c = cvWaitKey(33);
	if (c == 27) { // если нажата ESC - выходим
			break;
	}
  }
  cvDestroyAllWindows();
}
