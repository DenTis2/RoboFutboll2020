/*
 *  Пример захвата с камеры, бинаризации и вывода в окно для OpenCV версии 2  
 *  Заменил IplImage на Mat и команды, которые работают с Mat
 *  Выход из программы по нажатию на кнопку ESC
 
 Компилировать в Linux:

g++ `pkg-config --cflags opencv` -o binarytest binarytest.cpp `pkg-config --libs opencv`
 
 */

#include <cv.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <highgui.h>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

int main() {
	VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Mat frame; 
    cap >> frame;
    cv::imshow("Bin", frame);
    
    int hMin = 0;
    int hMax = 0;

    int sMin = 0;
    int sMax = 0;

    int vMin = 0;
    int vMax = 0;

    int chMax = 255;
    
    
    char hMinName[50];
    char hMaxName[50];
    
    char sMinName[50];
    char sMaxName[50];
    
    char vMinName[50];
    char vMaxName[50];
    
    std::sprintf(hMinName, "H min %d", hMin);
    std::sprintf(hMaxName, "H max %d", hMax);
    
    std::sprintf(sMinName, "S min %d", sMin);
    std::sprintf(sMaxName, "S max %d", sMax);
    
    std::sprintf(vMinName, "V min %d", vMin);
    std::sprintf(vMaxName, "V max %d", vMax);

    cv::createTrackbar(hMinName, "Bin", &hMin, chMax);
    cv::createTrackbar(hMaxName, "Bin", &hMax, chMax);

    cv::createTrackbar(sMinName, "Bin", &sMin, chMax);
    cv::createTrackbar(sMaxName, "Bin", &sMax, chMax);

    cv::createTrackbar(vMinName, "Bin", &vMin, chMax);
    cv::createTrackbar(vMaxName, "Bin", &vMax, chMax);

	cv::Mat image;
	
    while (true) {

        //cv::Mat image = mur.getCameraOneFrame();
        cap >> image;
        cv::imshow("Image", image);
        cv::cvtColor(image, image, CV_BGR2HSV);
        cv::Scalar lower(hMin, sMin, vMin);
        cv::Scalar upper(hMax, sMax, vMax);
        cv::inRange(image, lower, upper, image);
        cv::imshow("Bin", image);
        char c = cv::waitKey(10);
        if (c == 27) { // если нажата ESC - выходим
			break;
	}
    }
    cvDestroyAllWindows();
}
