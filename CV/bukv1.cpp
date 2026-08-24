#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <torch/script.h>
#include "detect.h"
#include "mySerial.h"
#include <unistd.h>
#include <spidev_lib++.h>
using namespace cv;
using namespace std;
Mat img,gray,src,cheb,crop,crop1,imgHSV;
int number = 0;
int tur,gmin,gmax,numberg;
long long sumg;
double Width,Height,middle;
int dich = 0;
mySerial serial("/dev/serial0", 9600);
vector<vector<int>> myColors{{29,68,78,225,60,179}, //Green
                             { 0,10,175,255,126,204},   //Red
                             {10,32,213,255,208,251}, //Yealow
                             {0,173,0,156,23,63},   //Black
};

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
void getConturs(Mat imgDil){

 vector<vector<Point>> contours;
 vector<Vec4i> hierarchy;
 findContours(imgDil,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

 for(int i = 0; i <contours.size(); i++){
  int area = contourArea(contours[i]);
  vector<vector<Point>>conPoly(contours.size());
  vector<Rect>boundRect(contours.size());
  string objectType;
   tur = area;
   //cout << "! " << tur << endl;
   if(tur >= 50){
    dich = 1;
   } 
   if(area > 2500 && area < 20000){
    number = 1;
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.003 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);
    //cout<<"AREA: "<<area<<endl;
    drawContours(img, conPoly,i,Scalar(0,255,0),2);
  }
 }
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
void pressFColor(Mat img,int f){  
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
 if(f==0){
 for(int u=0; u < myColors.size()-1; u++){
  Scalar lower(myColors[u][0],myColors[u][2],myColors[u][4]);
  Scalar upper(myColors[u][1],myColors[u][3],myColors[u][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  //imshow(to_string(u),mask);
   //Bukva(mask);
  getConturs(mask);
  if (number == 1){
   number = 0;
   if( u == 0){
    cout << "G" << endl;
    serial.Send("G");
   }
   else if( u == 1){
    cout << "R" << endl;
    serial.Send("R");
   }
   else if( u == 2){
    cout << "Y" << endl;
    serial.Send("Y");
   }
  }
 }
 }
 else{
  Scalar lower(myColors[3][0],myColors[3][2],myColors[3][4]);
  Scalar upper(myColors[3][1],myColors[3][3],myColors[3][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  Bukva(mask);   
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
int main(){
   char c = 0; 
 // mySerial serial("/dev/ttyUSB0", 9600);
//  std::string message;
//  unsigned char buffer[100] = {0};
//  unsigned char* abc;
//  abc = buffer;
  //serial.Receive(abc,1);
  int k = 0;
  cout<<endl<<"!"<<endl;
  //serial.NumberByteRcv(k);
 torch::jit::script::Module
  model=torch::jit::load("/home/grin/Videos/128pn24e.torchscript");
  VideoCapture cap(0);
  cout<<endl<<"!"<<endl;
  cap.set(CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CAP_PROP_FRAME_HEIGHT, 240);
  cout<<endl<<"!"<<endl;
 
//  cap.set(CAP_PROP_FPS, 31);
  /*cap >> src;
  Width=src.rows;
  Height=src.cols;*/
//  double width = GetCaptureProperty(cap, CAP_PROP_FRAME_WIDTH);
//  double height = GetCaptureProperty(cap, CAP_PROP_FRAME_HEIGHT);
  //http://www.bim-times.com/opencv/4.3.0/df/d94/samples_2cpp_2videowriter_basic_8cpp-example.html#a8  - src.size();
  //Mat imgf;
  cout<<endl<<"!"<<endl;
  //Rect roi(0,0,640,640);
  //imgNorm = img(roi);
  //imshow("121",img);
  //Mat3b whiteimg(640, 640,Vec3b(255,255,255));
  while(true){
  Mat img;
  cap>>img;
  //imshow("14656r5645465454554454621",whiteimg);
  //img.copyTo(whiteimg(Rect(0, 80, 640, 480)));
  //resize(img, imgf, Size(640, 640));
  //imshow("12",imgf);
  Mat imgNorm;
  Mat crop = img(Rect(0,56,128,128));
  cv::cvtColor(crop, imgNorm, cv::COLOR_BGR2RGB);
  //imshow("12",imgNorm);
  Mat col = crop;
  Mat buk;
  cvtColor(crop,buk,COLOR_BGR2GRAY);
  pressFColor(col,0);  
  cv::normalize(imgNorm, imgNorm, 0.0, 1.0, cv::NORM_MINMAX, CV_32F);
  //imshow("1",imgNorm);
  cout<<endl<<"!"<<endl;  
  vector<torch::jit::IValue> inputs = {
    torch::from_blob(
      imgNorm.data,
      {128,128,3},
      torch::kFloat32
    ).permute({2,0,1}).unsqueeze(0)
  };
  cout<<endl<<"!"<<endl;
  at::Tensor outputs = model.forward(inputs).toTensor();
  cout<<endl<<"!"<<endl;
  vector<Box> boxes = getBoxes(outputs);
  cout<<endl<<"!"<<boxes.size()<<endl;
  //if(boxes.size()>0)cout<<endl<<"&"<<to_string(boxes[0].conf)<<"& "<<to_string(boxes[0].y2);
  //else cout<<endl<<"&"<<"NOBUKV";
  if(boxes.size()!=0){
    //binary(buk);/
    //Bukva(cheb);
    //pressFColor(img,1);
    //cout<<endl<<"&"<<to_string(boxes[0].conf)<<"& "<<to_string(boxes[0].conf2)<<"& "<<to_string(boxes[0].conf3);
    if(boxes[0].conf2>boxes[0].conf&&boxes[0].conf2>boxes[0].conf3){
        cout<<"S";
        serial.Send("S");
    }
    else if(boxes[0].conf>boxes[0].conf3){
        cout<<"H";
        serial.Send("H");
    }
    else{
        cout<<"U";
        serial.Send("U");
    }
   }
  }
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
