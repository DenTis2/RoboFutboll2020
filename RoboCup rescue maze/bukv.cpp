#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <torch/script.h>
#include "detect.h"
#include "mySerial.h"
#include <unistd.h>
#include <spidev_lib++.h>
#include <csignal>
#include <pigpio.h>
#include <ctime>
using namespace cv;
using namespace std;
Mat img,gray,src,cheb,crop,crop1,imgHSV;
int number = 0;
int tur,gmin,gmax,numberg;
long long sumg;
double Width,Height,middle;
int dich = 0;
volatile sig_atomic_t signal_received = 0;
mySerial serial("/dev/serial0", 9600);
vector<torch::jit::IValue> inputs;
at::Tensor outputs;
vector<vector<int>> myColors(2,vector<int>(6,0));/*{/*{68,89,129,255,60,207}, //Green
                             { 173,179,130,186,175,255},   //Red
                             {left[0],left[1],left[2],left[3],left[4],left[5]}, //Yealowleft
                             {right[0],right[1],right[2],right[3],right[4],right[5]}, //Yealowright
                             //{0,173,0,156,23,63},   //Black
};*/

vector<Scalar>myColorValues{ {0,255,0},
                             {255,0,0},
                             {255,255,0},
                             {0,0,0},
};
int red_c(Mat frame, Point c) {
    // Получаем цвет пикселя
    // Vec3b19'
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

void waitings(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"S";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_HIGH);
        gpioWrite(26,num);
        gpioWrite(21, PI_LOW);      
    }
    return;
}
void waitingh(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"H";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_HIGH);
        gpioWrite(26,num);
        gpioWrite(21, PI_HIGH);      
    }
    return;
}
void waitingu(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"U";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_LOW);
        gpioWrite(26,num);
        gpioWrite(21, PI_HIGH);      
    }
    return;
}
void waitingy(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"Y";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_HIGH);
        gpioWrite(26,num);
        gpioWrite(21, PI_LOW);      
    }
    return;
}
void waitingr(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"R";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_HIGH);
        gpioWrite(26,num);
        gpioWrite(21, PI_HIGH);      
    }
    return;
}
void waitingg(float seconds,int num){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead){
        cout<<num<<"G";
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_LOW);
        gpioWrite(26,num);
        gpioWrite(21, PI_HIGH);      
    }
    return;
}
void sigint_handler(int signal) {
   signal_received = signal;
}

int getConturs(Mat imgDil){

 vector<vector<Point>> contours;
 vector<Vec4i> hierarchy;
 findContours(imgDil,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

 for(int i = 0; i <contours.size(); i++){
  int area = contourArea(contours[i]);
  vector<vector<Point>>conPoly(contours.size());
  vector<Rect>boundRect(contours.size());
  string objectType;
   tur = area;
   cout << "! " << tur << endl;
   if(tur >= 50){
    dich = 1;
   } 
   if(area > 7000 && area < 30000){
    number=1; 
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.003 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);
    //cout<<"AREA: "<<area<<endl;
    drawContours(img, conPoly,i,Scalar(0,255,0),2);
    return 1;
  }
 }
 return 0;
}
//todo: калибровка на уровень чёрного буквы на поле
void Bukva(Mat imgB){
 vector<vector<Point>> contours;
 vector<Vec4i> Hierarchy;
 findContours(imgB, contours, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
// findContours(imgB, contours, Hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
 float ratio=0.;
 for(int o = 0; o < contours.size(); o++){
  int place = contourArea(contours[o]);
  vector<vector<Point>>conPoly(contours.size());
  vector<Rect>boundRect(contours.size());
  cout << "PLACE!:"<< place << endl;
  if(place > 0) { 
	 //cout<<"AREA: "<<place<<endl;
     float peri = arcLength(contours[o],true);
     approxPolyDP(contours[o], conPoly[o],0.004 * peri, true);

     boundRect[o] =  boundingRect(conPoly[o]);
	 ratio = (float)(boundRect[o].br().y - boundRect[o].tl().y)/(float)(boundRect[o].br().x - boundRect[o].tl().x); 
	 cout<<"RATIO: " << ratio << endl;
     if(ratio <1.2 && ratio > 0.4){
     drawContours(img, conPoly,o,Scalar(0,255,0),2);

   /*float peri = arcLength(contours[o],true);
   approxPolyDP(contours[o], conPoly[o],0.004 * peri, true);

   boundRect[o] =  boundingRect(conPoly[o]);
   drawContours(img, conPoly,o,Scalar(0,255,0),2);*/
   //drawContours(imgB, contours,o,Scalar(0,255,0),2);
   //imshow("d",img);
//   float ratio = (float)(boundRect[o].br().y - boundRect[o].tl().y)/(float)(boundRect[o].br().x - boundRect[o].tl().x);
   cout<<"RATIO: " << ratio << endl;
   if(ratio >1.2 || ratio < 0.4){
	   cout<<"BAD CONTOR!";
   }
   int up = 0;
   int down = 0;
   cout << " Yo " << endl;
   cout << boundRect[o].tl().x << " " <<  boundRect[o].tl().y << " " <<  boundRect[o].br().x << " " <<  boundRect[o].br().y << endl;
   cout << floor(boundRect[o].tl().x + 0.4 * (boundRect[o].br().x - boundRect[o].tl().x )) << " " <<  boundRect[o].tl().y << " " << floor(boundRect[o].tl().x + 0.6 * (boundRect[o].br().x - boundRect[o].tl().x)) << " " << floor(boundRect[o].tl().y + 0.1 * (boundRect[o].br().y - boundRect[o].tl().y)) << endl;
   Rect roi(floor(boundRect[o].tl().x + 0.45 * (boundRect[o].br().x - boundRect[o].tl().x )) , floor(boundRect[o].br().y - 0.1 * (boundRect[o].br().y - boundRect[o].tl().y)), floor(0.1 * (boundRect[o].br().x - boundRect[o].tl().x )),floor(0.05 * (boundRect[o].br().y - boundRect[o].tl().y)));
   crop = imgB(roi);
   dich=0;
   getConturs(crop);
   //cout << "! " << tur << endl;
   //imshow("down",crop);
   if(dich == 1){
    cout << "black_down" << endl;
    dich = 0;
    down = 1;
   }
   else { 
    down = 0;
    cout << "white_down" << endl;
   }
   Rect roi1(floor(boundRect[o].tl().x + 0.45 * (boundRect[o].br().x - boundRect[o].tl().x )) , floor(boundRect[o].tl().y + 0.05 * (boundRect[o].br().y - boundRect[o].tl().y)), floor(0.1 * (boundRect[o].br().x - boundRect[o].tl().x )),floor(0.05 * (boundRect[o].br().y - boundRect[o].tl().y)));
   crop1 = imgB(roi1);
   dich=0;
   getConturs(crop1);
   //cout << "! " << tur << endl;
   //imshow("up",crop1);
   if(dich == 1){
    cout << "black_up" << endl;
    dich = 0;
    up = 1;
   }
   else {
    up = 0;
    cout << "white_up" << endl;
   }
   if(up == 1 && down == 1){
    cout << "S" << endl;
  //  serial.Se'nd("S\n");
   }
   else if(up == 0 && down == 1){
    cout << "U" << endl;
   // serial.Send("U\n");
   }
   else if(up==0 && down == 0){
    cout << "H" << endl;
    //serial.Send("H\n");
   }
   }  
 }
}
}
void pressFColor(Mat img,int f,int num){  
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
  Scalar lower(myColors[num][0],myColors[num][2],myColors[num][4]);
  Scalar upper(myColors[num][1],myColors[num][3],myColors[num][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  //imshow(to_string(u),mask);
   //Bukva(mask);
  //getConturs(mask);
  if (getConturs(mask)){
   number = 0;
   waitingy(2.0,num);   
  }
}
void binary(Mat img){
 gmin=255;
 gmax=0;
 sumg=0;
 numberg=0;
 cheb=img;
 for(int x = 0; x < Width; x++){
  for(int y = 0; y < Height; y++){
   sumg+=img.at<uchar>(x,y);
   numberg++;
   if(img.at<uchar>(x,y) < gmin){
    gmin=img.at<uchar>(x,y);
   } 
   else if(img.at<uchar>(x,y)>gmax){
    gmax=img.at<uchar>(x,y);
   }
  }
 }
 //middle=gmin+(gmax/2); 
 middle= sumg/numberg;
 //cout<<"G_MIN: "<< gmin <<endl;
 for(int x = 0; x < Width; x++){
  for(int y = 0; y < Height; y++){
   if(img.at<uchar>(x,y) <gmin+middle){ //магия вне хогвартса, но работает
   //if(i'mg.at<uchar>(x,y) <((gmin+middle)/2+gmin)/2){ //магия вне хогвартса, но работает
    //red_c(img,Point(x,y)) < 20
    img.at<uchar>(x,y)=254;

 /*   Vec3b & point = img.at<Vec3b>(x,y);
    point[0]=0;
    point[1]=0;
    point[2]=0;*/
    //point = img.at<Vec3b>(y,x);
   } 
   else{
    img.at<uchar>(x,y)=0; 

/*    Vec3b & point = img.at<Vec3b>(x,y);
    point[0]=255;
    point[1]=255;
    point[2]=255;*/
    //point = img.at<Vec3b>(x,y);
}
  }
 }  
 cheb=img;
// return img;
}
void colors(Mat nocolor,Mat img,int num){

 vector<vector<Point>> contours;
 vector<Vec4i> hierarchy;
 findContours(nocolor,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

 for(int i = 0; i <contours.size(); i++){
   int area = contourArea(contours[i]);
   vector<vector<Point>>conPoly(contours.size());
   vector<Rect>boundRect(contours.size());
   tur = area;
   if(area > 7000&&area<15000){//20000/50000
    cout<<area<<endl;
    number = 1;
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.02 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);
    cout<<"FIGURE: "<<conPoly[i].size()<<endl;
    drawContours(img, conPoly,i,Scalar(0,255,0),2);
    //TODO: sqrt
    if(conPoly[i].size()==4&&((float)boundRect[i].width/(float)boundRect[i].height> 0.7||(float)boundRect[i].width/(float)boundRect[i].height<1.3)){
      long long sum0=0,sum1=0,sum2=0;
      for (int y = boundRect[i].tl().y; y < boundRect[i].br().y; y++){
       for (int x = boundRect[i].tl().x; x < boundRect[i].br().x; x++){
        Vec3b& color = img.at<Vec3b>(y,x);
        sum0+=(int)color[0];sum1+=(int)color[1];sum2+=(int)color[2];
       }
      }
      int r=sum2/(boundRect[i].height*boundRect[i].width),g=sum1/(boundRect[i].height*boundRect[i].width),b=sum0/(boundRect[i].height*boundRect[i].width);
      for (int y = 0; y < boundRect[i].tl().y-5; y++){
       for (int x = 0; x < img.cols; x++){
        Vec3b& color = img.at<Vec3b>(y,x);
        sum0+=(int)color[0];sum1+=(int)color[1];sum2+=(int)color[2];
       }
      }
      for (int y = boundRect[i].br().y+5; y < img.rows; y++){
       for (int x = 0; x < img.cols; x++){
        Vec3b& color = img.at<Vec3b>(y,x);
        sum0+=(int)color[0];sum1+=(int)color[1];sum2+=(int)color[2];
       }
      }
      int r1=sum2/((boundRect[i].tl().y-5 + img.rows- boundRect[i].br().y- 5)*img.cols),g1=sum1/((boundRect[i].tl().y-5 + img.rows- boundRect[i].br().y- 5)*img.cols),b1=sum0/((boundRect[i].tl().y-5 + img.rows- boundRect[i].br().y- 5)*img.cols);
      cout<<(float)b/(float)b1<<" "<<(float)g/(float)g1<<" "<<(float)r/(float)r1<<endl;
      if((float)r/(float)r1>(float)g/(float)g1&&(float)r/(float)r1>(float)b/(float)b1){
         //waitingr(5.0,num);   
      }
      else if((float)g/(float)g1>(float)r/(float)r1&&(float)g/(float)g1>(float)b/(float)b1){
          //waitingg(5.0,num);
      }
    } 
   }
 }
}
void neir(Mat img,int w,torch::jit::script::Module &model,int num){//0-left,1-right
  Mat gr,bl,kernel,res;
  Mat imgNorm;
  Mat crop = img(Rect(w,12,96,96));
  cv::cvtColor(crop, imgNorm, cv::COLOR_BGR2RGB);
  cvtColor(img,gr,COLOR_BGR2GRAY);
  GaussianBlur(gr,bl,Size(3,3),3,0);
  Canny(bl,gr,25,75);
  kernel = getStructuringElement(MORPH_RECT,Size(3,3));
  dilate(gr,res,kernel);
  colors(res,img,num);
  Mat col = crop;
  Mat buk;
  cvtColor(crop,buk,COLOR_BGR2GRAY);
 pressFColor(col,0,num);  
  cv::normalize(imgNorm, imgNorm, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
  cout<<endl<<"!"<<endl;  
  inputs = {
    torch::from_blob(
      imgNorm.data,
      {96,96,3},
      torch::kFloat32
    ).permute({2,0,1}).unsqueeze(0)
  };
  cout<<endl<<"!"<<endl;
  outputs = model.forward(inputs).toTensor();
  cout<<endl<<"!"<<endl;
  vector<Box> boxes = getBoxes(outputs);
  cout<<endl<<"!"<<boxes.size()<<endl;

  if(boxes.size()!=0){
    cout<<"1: "<<boxes[0].conf<<" 2: "<<boxes[0].conf2<<" 3: "<<boxes[0].conf3<<endl;
    if(boxes[0].conf2>boxes[0].conf&&boxes[0].conf2>boxes[0].conf3){
        waitings(4.0,num);
    }
    else if(boxes[0].conf>boxes[0].conf3){
        waitingh(4.0,num);
    }   
    else{
        waitingu(4.0,num);
    }
   }
   else{
        gpioWrite(21, PI_LOW);
        gpioWrite(20,PI_LOW);
        gpioWrite(16,PI_LOW);
        gpioWrite(26,PI_LOW);     
   }
}
int main(){
   char c = 0; 
 // mySerial serial("/dev/ttyUSB0", 9600);
//  std::string message;
//  unsigned char buffer[100] = {0};
//  unsigned char* abc;
//  abc = buffer;
  //serial.Receive(abc,1);
   if (gpioInitialise() == PI_INIT_FAILED) {
      std::cout << "ERROR: Failed to initialize the GPIO interface." << std::endl;
      return 1;
   }
  FILE *fp; // указатель на файл
  fp = fopen( "/home/grin/Videos/built/bin.txt", "rt" );
  if(fp!=NULL){ 
    for(int y=0;y<6;++y){ fscanf ( fp, "%d", &myColors[1][y] ); cout<<"1"<<y<<": "<<myColors[1][y]<<endl;}
    for(int y=0;y<6;++y){ fscanf ( fp, "%d", &myColors[0][y] ); cout<<"0"<<y<<": "<<myColors[0][y]<<endl;}
  }
  else	cout<<"FAIL TO READ FILE!bin.txt"<<endl;
  
  int k = 0;
  cout<<endl<<"!"<<endl;
  //serial.NumberByteRcv(k);
  torch::jit::script::Module
  model=torch::jit::load("/home/grin/Videos/96pn24e.torchscript");
  VideoCapture cap(0);
  VideoCapture cap1(2);
  cout<<endl<<"!"<<endl;
  cap.set(CAP_PROP_FRAME_WIDTH, 160);
  cap.set(CAP_PROP_FRAME_HEIGHT, 120);
  cap1.set(CAP_PROP_FRAME_WIDTH, 160);
  cap1.set(CAP_PROP_FRAME_HEIGHT, 120);
 
  cout<<endl<<"!"<<endl;
   gpioSetMode(21, PI_OUTPUT);//21-1
   gpioSetMode(20,PI_OUTPUT);//20-2
   gpioSetMode(16,PI_OUTPUT);//16-3//16-2
   gpioSetMode(26,PI_OUTPUT);//26-4//26-1
   signal(SIGINT, sigint_handler);
 
//  cap.set(CAP_PROP_FPS, 31);
  //cap >> src;
  Width=src.rows;
  Height=src.cols;
//  double width = GetCaptureProperty(cap, CAP_PROP_FRAME_WIDTH);
//  double height = GetCaptureProperty(cap, CAP_PROP_FRAME_HEIGHT);
  //http://www.bim-times.com/opencv/4.3.0/df/d94/samples_2cpp_2videowriter_basic_8cpp-example.html#a8  - src.size();
  //Mat imgf;
  cout<<endl<<"!"<<endl;
  //Rect roi(0,0,640,640);
  //imgNorm = img(roi);
  //imshow("121",img);
  //Mat3b whiteimg(640, 640,Vec3b(255,255,255));
 Mat img;  
 while(!signal_received){
  //right_cam
  if(cap.grab()){cap>>img;
  neir(img,64,model,1);
  //second frane
  neir(img,0,model,1);}
  else cout<<"cheebureck11";
  //left_cam
  if(cap1.grab()){cap1>>img;  
  neir(img,64,model,0);
  //second frane
  neir(img,0,model,0);}
  else cout<<"cheebureck00"; 
  }
   gpioWrite(20,PI_LOW);
   gpioWrite(16,PI_LOW);
   gpioWrite(26,PI_LOW);
   gpioWrite(21,PI_LOW);
   gpioSetMode(20,PI_INPUT);
   gpioSetMode(16,PI_INPUT);
   gpioSetMode(26,PI_INPUT);  
   gpioSetMode(21,PI_INPUT);  
  //highlightBoxes(img, boxes);
  //cv::imshow("Result", img);
  /*while(c!=27){
   cap.read(img);
   cvtColor(img,gray,COLOR_BGR2GRAY);
   //Mat cop;
   gray.copyTo(cop);
   Width=cop.rows;
   Height=cop.cols;
   //cout<<"W: " <<src.size().width << "H: " << src.size().height;
 /*int line[960];
 for(int u = 0; u < 960; u++){
  line[u] = 0;
 }
 for(int x = 481; x < 1441; x++){
  cout << red_c(img,Point(x,1079)) << endl;
  if(red_c(img,Point(x,1079)) < 40){
   line[x-481] = 1;
  }
 }   
  pressFColor(img);
   //imshow("g",img);
   //imshow("b",binary(cop));
   binary(cop);
   //imshow("b",cheb);
   Bukva(cheb);
  // imshow("o",imgHSV);
  // imshow("y",img);
   c=waitKey(1);
 }*/
 return 0;
}
