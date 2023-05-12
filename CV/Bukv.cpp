#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;
Mat img;
int number = 0;
int tur;
int dich = 0;
vector<vector<int>> myColors{{61,83,209,255,137,198}, //Green
                             {0,8,200,255,146,192},   //Red
                             {16,41,134,200,190,241}, //Yealow
                             {11,110,0,160,0,67},   //Black
};

vector<Scalar>myColorValues{ {0,255,0},
                             {255,0,0},
                             {255,255,0},
                             {0,0,0},
};
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
   cout << "! " << tur << endl;
   if(tur >= 1){
    dich = 1;
   } 
   if(area > 2000){
    number = 1;
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.02 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);

    drawContours(img, conPoly,i,Scalar(0,255,0),2);
  }
 }
}

void Bukva(Mat imgB){
 
 vector<vector<Point>> contours;
 vector<Vec4i> Hierarchy;
 findContours(imgB, contours, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

 for(int o = 0; o < contours.size(); o++){
  int place = contourArea(contours[o]);
  vector<vector<Point>>conPoly(contours.size());
  vector<Rect>boundRect(contours.size());
  cout << place << endl;
  if(place > 2000){
   float peri = arcLength(contours[o],true);
   approxPolyDP(contours[o], conPoly[o],0.02 * peri, true);

   boundRect[o] =  boundingRect(conPoly[o]);
   int up = 0;
   int down = 0;
   cout << " Yo " << endl;
   cout << boundRect[o].tl().x << " " <<  boundRect[o].tl().y << " " <<  boundRect[o].br().x << " " <<  boundRect[o].br().y << endl;
   cout << floor(boundRect[o].tl().x + 0.4 * (boundRect[o].br().x - boundRect[o].tl().x )) << " " <<  boundRect[o].tl().y << " " << floor(boundRect[o].tl().x + 0.6 * (boundRect[o].br().x - boundRect[o].tl().x)) << " " << floor(boundRect[o].tl().y + 0.1 * (boundRect[o].br().y - boundRect[o].tl().y)) << endl;
   Rect roi(floor(boundRect[o].tl().x + 0.45 * (boundRect[o].br().x - boundRect[o].tl().x )) , floor(boundRect[o].br().y - 0.1 * (boundRect[o].br().y - boundRect[o].tl().y)), floor(0.1 * (boundRect[o].br().x - boundRect[o].tl().x )),floor(0.05 * (boundRect[o].br().y - boundRect[o].tl().y)));
   Mat crop = imgB(roi);
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
   Mat crop1 = imgB(roi1);
   getConturs(crop1);
   //cout << "! " << tur << endl;
   // imshow("up",crop1);
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
   }
   else if(up == 0 && down == 1){
    cout << "U" << endl;
   }
   else {
    cout << "H" << endl;
   }
  }
 }
}
void pressFColor(Mat img){
 Mat imgHSV;
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
 for(int u=0; u < myColors.size(); u++){
  Scalar lower(myColors[u][0],myColors[u][2],myColors[u][4]);
  Scalar upper(myColors[u][1],myColors[u][3],myColors[u][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  //imshow(to_string(u),mask);
  if (u == 3){
   Bukva(mask);
  }
  else{
   getConturs(mask);
  }
  if (number == 1){
   number = 0;
   if( u == 0){
    cout << "green" << endl;
   }
   else if( u == 1){
    cout << "red" << endl;
   }
   else if( u == 2){
    cout << "yellow" << endl;
   }
  }
 }
}
int main(){
  VideoCapture cap(0);
  while(true){
   cap.read(img);
   pressFColor(img);
  // imshow("g",img);
  // imshow("o",imgHSV);
  // imshow("y",img);
   waitKey(1);
 }
 return 0;
}