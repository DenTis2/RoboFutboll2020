#define _CRT_SECURE_NO_WARNINGS
#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include "mySerial.h"
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;


// Функция возвращает красную компоненту пикселя с координатами "c" на изображение "frame"
int red_c(Mat frame, Point c) {
    // Получаем цвет пикселя
    // Vec3b
    // [0] - синяя компонента
    // [1] - зелёная компонента
    // [2] - красная компонента
    Vec3b point = frame.at<Vec3b>(c);
    // Возвращаем красную компоненту
    // Красная компонента последняя в массиве, так как используется пространство BGR (Blue-Green-Red)
    // point[0] - синяя компонента
    // point[1] - зелёная компонента
    // point[2] - красная компонента
    return point[2];
}

int main() {
	mySerial serial("/dev/ttyUSB0",115200);
    // Номер строки изображения, на которой будет происходить распознавание чёрной линии
    int scan_row = 470;
    // X координата правого края чёрной линии
    int center = -1;
    
    std::string message;
    
    Mat frame;
    VideoCapture cap(2);
    //CvCapture* capture = cvCreateCameraCapture(2);
    //assert( capture );
    
    if(!cap.isOpened()) {
        cout << "Unable to open video source" << endl;
        return 1;
    }
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    unsigned char buffer[100] = {0};
	unsigned char* abc;
	abc = buffer;
	int k = 0;
	serial.Receive(abc, 1);
	serial.NumberByteRcv(k);
	cout << " "<< k <<"\n";
    //cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
	//IplImage* g = 0;
	/*while (k){
			serial.Receive(abc, 1);
			printf("%c", abc[0]);
			serial.NumberByteRcv(k);
			cout << " "<< k <<"\n";
	}*/
    while(true) {
		cap.read(frame);
		 //g = cvQueryFrame( cap );
		// показываем картинку в созданном окне
        
        
        if(frame.empty())
            continue;

        // ***** Обнаружение чёрной линии на изображении *****
        // Предполагаем, то что правый край чёрной линии находится в этой x координате
        int x = 640;
        int xLeft = 5;
        // Идём до начала изображения, пока не найдём чёрный пиксель
        // Чёрным пикселем считается, тот пиксель у которого красная компонента меньше 40
        for(; x > 0; x--){
            if(red_c(frame, Point(x, scan_row)) < 70)
                break;
        }
        for(; xLeft < 640; xLeft++){
            if(red_c(frame, Point(xLeft, scan_row)) < 70)
                break;
        }
        cout << "xLeft = " << red_c(frame, Point(xLeft, scan_row)) << " " << "x = " << red_c(frame, Point(x, scan_row)) << "\n";
        line(frame, Point(xLeft, scan_row), Point(xLeft, scan_row - 40), Scalar(0, 255, 0), 2);
        line(frame, Point(x, scan_row), Point(x, scan_row - 40), Scalar(0, 255, 0), 2);
        line(frame, Point((xLeft + x) / 2, scan_row), Point((xLeft + x) / 2, scan_row - 40), Scalar(0, 255, 0), 2);
        imshow("capture", frame);
        center = x;
        // Устанавливаем x координату правого края чёрной линии
        

        // ================= Релейный регулятор ==================
        /*
         * if(abs(center - 320) < 10) angle = 90;
         * // Если робот отклонился влево
         * if(center - 320 > 10) angle = 60;
         * // Если робот отклонился вправо
         * else angle = 120;
         */

        // ===================== П регулятор =====================
        // Рассчитываем отклонение работа от желаемого центра
        float error = ((x + xLeft) / 2) - (640/2);
        // Рассчитываем пропорциональную часть регулятора
        //angle = 90 + (error * 1.2);
        // Ограничиваем угол поворота сервопривода
        //if(angle < 60) angle = 60;
        //if(angle > 120) angle = 120;

		int b = 130 - (error * 0.3); int a = 130 + (error * 0.3);
		
		if(a > 254) { a = 254; }
		if(a < 1) { a = 1; }
		
		if(b > 254) { b = 254; }
		if(b < 1) { b = 1; }

        //message = "SPD " + std::to_string(angle) + "," + std::to_string(direction) + "," + std::to_string(speed) + " ";

        // Отправляем команду на Arduino
		
		string s = "";
		s = "A" + to_string(a) + " B" + to_string(b) + " ";
		
		//string s2 = "";
		//s2 = "A" + to_string(a) + "\r" + "B" + to_string(b) + "\r";
		//cout << s;
		serial.Send(s);
		//usleep(500000);
		char c = cvWaitKey(33);
		if (c == 27) { // нажата ESC
			break;
		}
	}
    //cvReleaseCapture( &capture );
    serial.Send("A0 B0 ");
    cvDestroyAllWindows();		
    return 0;
}
