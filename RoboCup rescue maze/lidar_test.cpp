// File:          test.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <unistd.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
//#include <webots/distance_sensor.h>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <webots/Receiver.hpp> // Include Receiver.hpp and Emitter.hpp
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/Lidar.hpp>
#include <iostream>
//#include <unistd.h>
#include <cstring>
#define TIME_STEP 64
#define MAX_SPEED 3.14 
//6.28
// All the webots classes are defined in the "webots" namespace
int number = 0;
int red = 0;
int yeallow = 0;
double last;
double last_top_l;
double last_left_l;
double last_right_l;
double NE_point;
double SE_point;
double NW_point;
double SW_point;
double NE1_point;
double NW1_point;
double lastgyro = 0.;
double oldgpsx;  
double oldgpsy ;
int counter = 0;
int leftrotate = 0;
int rightrotate = 0;
int backrotate = 0;
int tur;
int dich = 0;
int white = 0;
int black = 0;
int firsttile = 0;
int firstgyro = 0;
int xmnext;
int ymnext;
int xmleft;
int ymleft;
int xmlast;
int ymlast;
int firstx;
int firsty;
int firstntplay= 0;
int hand = 0; //0 - right; 1 - left;
float tilesize = 11.5;
int mapdirection = 10; // ==0 - Nourth; ==1 - Earst; ==2 - SOUTH; ==3 -WEST;
long tpause = 1536;
using namespace webots;
using namespace std; 
using namespace cv;
vector<vector<int>> myColors{{61,83,209,255,137,198}, //Green
                             {110,178,149,254,124,200},   //Red
                             {20,30,100,255,100,255}, //Yealow
                             {0,172,0,111,168,255},//white
                             {11,110,0,160,0,67},   //Black
};
vector<Scalar>myColorValues{ {0,255,0},
                             {255,0,0},
                             {255,255,0},
                             {0,0,0},
};
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
struct myNode{
char wall_up;
char wall_down;
char wall_left;
char wall_right;
char tile;
char victim;
int count;
//string victim;
int coordinata_x, coordinata_y;
};
typedef myNode *PNode;
PNode mymap[50][50];
PNode CreateNode ( char tile, int x, int y)
{
  PNode NewNode = new myNode; // указатель на новый узел
  NewNode->tile = tile; // записать пол
  NewNode->coordinata_x = x; // координата x GPS
  NewNode->coordinata_y = y; // координата y GPS
  NewNode->count = 0;
  return NewNode; // результат функции – адрес узла
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
   //tur = area;
   //cout << "! " << area << endl;
   /*if(tur >= 1){
    dich = 1;
   } */
   if(area > 100 && area < 300){
    white = 1;
   }
   if(area > 10){
    black = 1;
   }
   if(area > 5){
    number = 1;
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.02 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);

    //drawContours(frame, conPoly,i,Scalar(0,255,0),2);
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
  if(place > 500){
   float peri = arcLength(contours[o],true);
   approxPolyDP(contours[o], conPoly[o],0.02 * peri, true);

   boundRect[o] =  boundingRect(conPoly[o]);
   cout << " Yo " << endl;
   //cout << oundRect[o].tl() << endl;
   //cout << boundRect[o].br() << endl;
   Rect roi(boundRect[o].tl(), boundRect[o].br());
   Mat crop = imgB(roi);
   //cout << "! " << tur << endl;
   //imshow("down",crop);
   Scalar lower(myColors[4][0],myColors[4][2],myColors[4][4]);
   Scalar upper(myColors[4][1],myColors[4][3],myColors[4][5]);
   Mat musk;
   inRange(crop,lower,upper,musk); 
   black = 0;
   getConturs(musk);
   imshow("Bukva",musk);
   cout << "!!!!!!!!!!!!" << endl;
   if(black == 1){
    cout << "black" << endl;
   }
  }
 }
}
void pressFColor(Mat img){
 Mat imgHSV;
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
 for(int u=3; u > 0; u--){
  Scalar lower(myColors[u][0],myColors[u][2],myColors[u][4]);
  Scalar upper(myColors[u][1],myColors[u][3],myColors[u][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  //imshow(to_string(u),mask);
  imshow("frame1", mask);
  getConturs(mask);
  if (number == 1){
/*for (int j=0; j< 16; j++)
mymap[i][j] = CreateNode(' ', 32000, 32000);*/
   number = 0;
   if( u == 2){
    yeallow = yeallow + 1;
    cout << "yeallow" << endl;
    red = 0;
   }
   else if( u == 1){
    cout << "red" << endl;
    red = red + 1;
   }
  }
  if(white == 1){
   cout << "white" << endl;
   Bukva(mask);
  }
 }
}
//угол гироскопа привязвается к gps

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  //Motor *left_motor = robot->getMotor("left wheel motor");
  //Motor *right_motor = robot->getMotor("right wheel motor");
  Motor *left_motor = robot->getMotor("wheel2 motor");
  Motor *right_motor = robot->getMotor("wheel1 motor");
   
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);
 
  for (int i=0; i< 50; i++)
  for (int j=0; j< 50; j++)
  mymap[i][j] = CreateNode(' ', 32000, 32000);
  
  //WbDeviceTag ps[8];
  //ps[1] = wb_robot_get_device("ps0");
  //wb_distance_sensor_enable(ps[1],TIME_STEP);
  /*DistanceSensor *top_l = robot->getDistanceSensor("l0");
  DistanceSensor *left_l = robot->getDistanceSensor("l1");
  DistanceSensor *right_l = robot->getDistanceSensor("l2");*/
  Emitter* emitter = robot->getEmitter("emitter"); // Retrieve the receiver and emitter by device name
  Receiver* receiver = robot->getReceiver("receiver");
  GPS* gps = robot->getGPS("gps"); 
  Camera *cam2 = robot->getCamera("camera2");
  Camera *cam1 = robot->getCamera("camera1");
  DistanceSensor *top_l = robot->getDistanceSensor("distance sensor3");
  DistanceSensor *left_l = robot->getDistanceSensor("distance sensor1");
  DistanceSensor *right_l = robot->getDistanceSensor("distance sensor2");
  DistanceSensor *down = robot->getDistanceSensor("distance sensor4");
  PositionSensor *leftEncoder = left_motor->getPositionSensor();
  PositionSensor *rightEncoder = right_motor->getPositionSensor();  
  Camera* colorSensor = robot->getCamera("colour_sensor"); 
  Lidar*lidar = robot->getLidar("lidar");
  Gyro* gyro = robot->getGyro("gyro");
  lidar->enable(TIME_STEP);      
  lidar -> enablePointCloud();
  top_l->enable(TIME_STEP);
  left_l->enable(TIME_STEP);
  right_l->enable(TIME_STEP);
  down->enable(TIME_STEP);
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  leftEncoder->enable(timeStep);
  rightEncoder->enable(timeStep);
  colorSensor->enable(timeStep);
  cam2->enable(timeStep);
  cam1->enable(timeStep);
  receiver->enable(timeStep);
  gps->enable(timeStep); 
  gyro->enable(timeStep);
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  double down_val = down->getValue();
  double top_l_val = top_l->getValue();
  double left_l_val = left_l->getValue();
  double right_l_val = right_l->getValue();
  last_left_l = left_l_val;
  last_right_l = right_l_val;
  last_top_l = top_l_val;
  
/*  int width = 5, height = 5;
  string map[width][height] ={ // Test map array for world1.wbt
        {"1", "1", "1", "1", "1"},
        {"1", "5", "0", "5", "0"},
        {"1", "0", "0", "0", "0"},
        {"1", "5", "0", "5", "0"},
        {"1", "0", "0", "0", "0"} 
  };
  string flattened = "";
  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {

      flattened += map[i][j] + ","; // Flatten the array with comma separators
    }
  }
  
  flattened.pop_back(); // Remove the last unnecessary comma

  char message[8 + flattened.size()]; // message array

  memcpy(message, &width, sizeof(width)); // The first 2 integers in the message array are width, height
  memcpy(&message[4], &height, sizeof(height));

  memcpy(&message[8], flattened.c_str(), flattened.size()); // Copy in the flattened map afterwards

  while (robot->step(timeStep) != -1) {
    
    emitter->send(message, sizeof(message)); // Send map data

    char msg = 'M'; // Send map evaluate request
    emitter->send(&msg, sizeof(msg));

    msg = 'E'; // Send an Exit message to get Map Bonus
    emitter->send(&msg, sizeof(msg));
  }*/

  while (robot->step(timeStep) != -1) {

    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
   /* const unsigned char* image = colorSensor->getImage(); // Step 4: Retrieve the image frame.
    
    // Get the individual RGB color channels from the pixel (0,0)
    // Note that these functions require you to pass the width of the overall image in pixels.
    // Since this is a 1px by 1px color sensor, the width of the image is just 1. 
    int r = colorSensor->imageGetRed(image, 1, 0, 0);
    int g = colorSensor->imageGetGreen(image, 1, 0, 0);
    int b = colorSensor->imageGetBlue(image, 1, 0, 0);
    
    cout << "r: " << r << " g: " << g << " b: " << b << endl;*/
    //check_LK();
    if(receiver->getQueueLength() > 0) { // If receiver queue is not empty
     char *m = (char *)receiver->getData(); // Grab data as a string
     if (m[0] == 'L') { // 'L' means a lack of progress occurred
        firsttile = 0;
        firstntplay = 1;
        receiver->nextPacket(); // Discard the current data packet
     }
    }

    const float *lid = lidar -> getRangeImage();
    //const float *lid = lidar -> getLayerPointCloud(2);
    down_val = down->getValue();
    top_l_val = top_l->getValue();
    left_l_val = left_l->getValue();
    right_l_val = right_l->getValue();
    //down_val = lid[128*8 + 256] * 1.5;
    top_l_val = lid[128*8] * 1.5;
    left_l_val = (lid[128*8 + 128*3] +0)* 1.5;
    right_l_val = (lid[128*8 + 128]  + 0) * 1.5;
    NE_point = lid[128*8 + 120] * 1.5;//64
    SE_point = lid[128*8 + 136] * 1.5;//192 
    NW_point = lid[128*8 + 128*3 + 8] * 1.5;//64
    SW_point = lid[128*8 + 128*3 - 8] * 1.5;//192 
    NE1_point = lid[128*8 + 42] * 1.5;
    NW1_point = lid[128*8 + 128*3 + 86] * 1.5;
      
    //cout<< down_val << endl;
     //rightEncoder->getValue(); 
     // leftEncoder->getValue();
    //double ps = wb_distance_sensor_get_value(ps[1]);
     //cout << rightEncoder->getValue() << " " << leftEncoder->getValue() <<" ";//
     //cout << left_l_val << " " << top_l_val << " " <<right_l_val << " " <<endl;
     //cout << down_val << " "<< top_l_val<< endl;
     cout <<"LIDAR: " << lidar -> getNumberOfLayers() << " " << lidar -> getHorizontalResolution() << endl;
     //for (int i = 128*8; i < 128*12; ++ i) cout << lid[i] * 10 << " ";
     cout << "UPL: "<< lid[128*8] * 2 << endl;
     //cout << "UPD: " << top_l->getValue() << endl;
     cout << "RightL: "<< right_l_val<< endl;
     //cout << "RightD: "<< right_l->getValue() << endl;
     cout << "LeftL: "<< left_l_val << endl;
     //cout << "LeftD: "<< left_l->getValue() << endl; 
     cout << "GPSX: " << (int)(gps->getValues()[0] * 100.) << "GPSY: " << (int)(gps->getValues()[2] * 100.) << endl;
     lastgyro = lastgyro + gyro->getValues()[1] * timeStep * 0.001;
     cout <<"Gyro: "  << (int)(lastgyro * 180/3.1416) % 360 << " FG: " << firstgyro<< endl;
     //cout << "Down: "<< lid[128*8 + 256] << endl;     
     
     //зполняем тайл
   /*  if(firsttile == 0){
      firsttile = 1;
      firsty = gps->getValues()[2] * 100.;
      firstx = gps->getValues()[0] * 100.;      
     }*/
     int xm,ym;
     if(firsttile == 0){
      firstgyro = (int)(lastgyro * 180/3.1416) % 360;
      if(firstntplay == 0){
       firsty = gps->getValues()[2] * 100.;
       firstx = gps->getValues()[0] * 100.;   //
       xm = (int )((gps->getValues()[0] * 100. - firstx - (tilesize*0.5) + tilesize*8) / tilesize);
       ym = (int)((gps->getValues()[2] * 100. - firsty -(tilesize*0.5) + tilesize*8) / tilesize);
       mymap[xm][ym]->tile='5';
       if(lid[128*8]*1.5 < 0.15){ mymap[xm][ym]->wall_up='1';} else mymap[xm][ym]->wall_up='0';
       if(lid[128*8 + 128] * 1.5 < 0.15){ mymap[xm][ym]->wall_right='1';} else mymap[xm][ym]->wall_right='0';
       if(lid[128*8 + 128*3]* 1.5 < 0.15){ mymap[xm][ym]->wall_left='1';} else mymap[xm][ym]->wall_left='0';
       if(lid[128*8 + 128*2] * 1.5 < 0.15){ mymap[xm][ym]->wall_down='1';} else mymap[xm][ym]->wall_down='0';
      }
     oldgpsx = gps->getValues()[0];  
     oldgpsy = gps->getValues()[2];
     //lastgyro = lastgyro + gyro->getValues()[1] * timeStep * 0.001;     
     firstgyro = (int)(lastgyro * 180/3.1416) % 360;
     //firsttile = 1;
     }
     if(firsttile == 1){
      if(abs(gps->getValues()[0] - oldgpsx) > abs(gps->getValues()[2] - oldgpsy)){
       if(gps->getValues()[0] > oldgpsx) mapdirection = 0;//+x
       else mapdirection = 2;//-x
      }
      else{
       if(gps->getValues()[2] > oldgpsy) mapdirection = 1;//+y
       else mapdirection = 3;//-y
      }
     }    
     xm = (int )((gps->getValues()[0] * 100. - firstx -(tilesize*0.5) + tilesize*8) / tilesize);
     ym = (int)((gps->getValues()[2] * 100. - firsty -(tilesize*0.5) + tilesize*8) / tilesize);
     cout << "XM: " << xm << " " << "YM:" << ym << " MAPD: "<< mapdirection <<  endl;
     //координаты следующего тайла
     int gyronow = (int)(lastgyro * 180/3.1416) % 360;
     gyronow = (gyronow + 360 - 90*mapdirection) % 360;
     if(gyronow > 315 || gyronow < 45) {xmnext = xm+1; ymnext = ym; xmleft = xm; ymleft = ym-1;}
     if(gyronow >= 45 && gyronow < 135) {ymnext = ym-1; xmnext = xm; xmleft = xm-1; ymleft = ym;}
     if(gyronow >= 135 && gyronow < 225) {xmnext = xm-1; ymnext = ym; xmleft = xm; ymleft = ym+1;}
     if(gyronow >= 225 && gyronow <= 315) {ymnext = ym+1; xmnext = xm; xmleft = xm+1; ymleft = ym;}
     cout << "XMNEXT: " << xmnext << " " << "YMNEXT:" << ymnext << " XMLEFT: " << xmleft << " " << "YLEFT:" << ymleft << endl;
          
     if(mymap[xm][ym]->tile == ' '){
     mymap[xm][ym]->tile='0';
     if(firsty == gps->getValues()[2] * 100. && firstx == gps->getValues()[0] * 100.) mymap[xm][ym]->tile='5';
     if(lid[128*8]*1.5 < 0.15){ mymap[xm][ym]->wall_up='1';} else mymap[xm][ym]->wall_up='0';
     if(lid[128*8 + 128] * 1.5 < 0.15){ mymap[xm][ym]->wall_right='1';} else mymap[xm][ym]->wall_right='0';
     if(lid[128*8 + 128*3]* 1.5 < 0.15){ mymap[xm][ym]->wall_left='1';} else mymap[xm][ym]->wall_left='0';
     if(lid[128*8 + 128*2] * 1.5 < 0.15){ mymap[xm][ym]->wall_down='1';} else mymap[xm][ym]->wall_down='0';
     }     
     for (int i=0; i< 20; i++){
     for (int j=0; j< 20; j++)
     if(i==xm && j == ym) cout <<"*";
     else if(i==xmleft && j == ymleft) cout << "+";
     else cout << mymap[i][j]->tile;
     cout << endl;
     }
     
     if(mymap[xmnext][ymnext]->count > mymap[xm][ym]->count){
      mymap[xmnext][ymnext]->count -= 1;
     }
     
     if(xmlast != xm || ymlast != ym){
      xmlast = xm;
      ymlast = ym;
      mymap[xm][ym]->count++;
      if(mymap[xm][ym]->count % 2 == 0){
       hand = 1;
      }
      else{
       hand = 0;
      }
     }  
     
     float sp1;//
     float sp2; 
     if(firsttile < 2){
      sp1 = 3.14; sp2 = 3.14;
      if(firsttile == 1) {sp1 = -3.14; sp2 = -3.14;}
      firsttile += 1;
     }
     //отъезд назад
     else if(backrotate == 1){
      if(-leftEncoder->getValue() + last < 0.5){
       left_speed = -MAX_SPEED * 2;
       right_speed = -MAX_SPEED * 2;
       leftEncoder->getValue();      
      }
      else{
       backrotate = 0;
       left_motor->setVelocity(0.0);
       right_motor->setVelocity(0.0);
       leftEncoder->getValue(); 
      }       
     }
     //поворот налево на 90 градусов
     else if (leftrotate == 1){
      if(leftEncoder->getValue() - last > -3.075){
       left_speed = -MAX_SPEED * 2;
       right_speed = MAX_SPEED * 2;
       leftEncoder->getValue();      
      }
      else{
       leftrotate = 0;
       //toprotate = 1;
       last = leftEncoder->getValue();
       left_motor->setVelocity(0.0);
       right_motor->setVelocity(0.0);
      }    
     }
     //поворот направо на 90 градусов
     else if (rightrotate == 1){
      if(leftEncoder->getValue() - last < 3.075){
       left_speed = MAX_SPEED * 2;
       right_speed = -MAX_SPEED * 2;
       leftEncoder->getValue();      
      }
      else{
       rightrotate = 0;
       //toprotate = 1;
       last = leftEncoder->getValue();
       left_motor->setVelocity(0.0);
       right_motor->setVelocity(0.0);
      }    
     }
     else if(hand == 0){
      //правая рука
      //обычная езда
      //todo проверка впереди по карте тайла (были там/нет/есть ли яма?)
      if(top_l_val > 0.09 && NE1_point > 0.09 && NW1_point > 0.09 && down_val < 0.2){
      //top_l_val
 /*      float sp1 = MAX_SPEED + right_l_val * 7.5;
       float sp2 = MAX_SPEED - right_l_val * 7.5;
  */     
       sp1 = MAX_SPEED + (NE_point-SE_point) * 75;
       sp2 = MAX_SPEED - (NE_point-SE_point) * 75;    
       if((abs(NE_point-SE_point) > 0.5) || (right_l_val > 0.3 && left_l_val > 0.3 && top_l_val > 0.3)){sp1 = 6.28; sp2 = 6.28;} 
       if(sp1 > 6.28){
        sp1 = 6.28;
       }
       if(sp2 < -6.28){
        sp2 = -6.28;
       }
       if(sp2 > 6.28){
        sp2 = 6.28;
       }
       if(sp1 < -6.28){
        sp1 = -6.28;
       }
       left_speed = sp1;
       right_speed = sp2;
       
       if (right_l_val < 0.03)  left_speed =  left_speed * 0.7;
       if (right_l_val > 0.1)  right_speed =  right_speed * 0.7;
       if (left_l_val < 0.03)  right_speed =  right_speed * 0.7;
/*       if(mymap[xmnext][ymnext]->tile == '0' && mymap[xmleft][ymleft]->tile == ' ' && left_l_val > 0.2){
       last = leftEncoder->getValue();
       leftrotate = 1;
       backrotate = 1;       
       }*/
      }
      else {
       last = leftEncoder->getValue();
       leftrotate = 1;
       backrotate = 1;
       //left_speed = -MAX_SPEED * 2;
       //right_speed = MAX_SPEED * 2;
      }
     }
     else{     
      //левая рука
      //обычная езда
      //todo проверка впереди по карте тайла (были там/нет/есть ли яма?)
      if(top_l_val > 0.09 && NE1_point > 0.09 && NW1_point > 0.09 && down_val < 0.2){
      //top_l_val
       float sp1 = MAX_SPEED + right_l_val * 7.5;
       float sp2 = MAX_SPEED - right_l_val * 7.5;
       
       sp1 = MAX_SPEED - (NW_point-SW_point) * 75;
       sp2 = MAX_SPEED + (NW_point-SW_point) * 75;    
       if(abs(NW_point-SW_point) > 0.5){sp1 = 6.28; sp2 = 6.28;} 
       if(sp1 > 6.28){
        sp1 = 6.28;
       }
       if(sp2 < -6.28){
        sp2 = -6.28;
       }
       if(sp2 > 6.28){
        sp2 = 6.28;
       }
       if(sp1 < -6.28){
        sp1 = -6.28;
       }
       left_speed = sp1;
       right_speed = sp2;
       
       if (left_l_val < 0.03)  right_speed =  right_speed * 0.7;
       if (left_l_val > 0.1)  left_speed =  left_speed * 0.7;
       if (right_l_val < 0.03)  left_speed =  left_speed * 0.7;
      }
      else {
       last = leftEncoder->getValue();
       rightrotate = 1;
       backrotate = 1;
       //left_speed = -MAX_SPEED * 2;
       //right_speed = MAX_SPEED * 2;
     }
    }
//
      /*while(leftEncoder->getValue() - last < 35.0){
       left_motor->setVelocity(-left_speed);
       right_motor->setVelocity(right_speed);
       leftEncoder->getValue();
      } */   
      //left_motor->setVelocity(left_speed);
      //right_motor->setVelocity(right_speed); 
   
    Mat frame2(cam2->getHeight(), cam2->getWidth(), CV_8UC4, (void*)cam2->getImage());
   // Rect roi1(boundRect[o].tl(), boundRect[o].br());    
   // Mat img2 = frame2(roi1);2
    imshow("frame2", frame2);
    pressFColor(frame2);
/*    Mat frame1(cam1->getHeight(), cam1->getWidth(), CV_8UC4, (void*)cam1->getImage());
    //Rect roi2(boundRect[o].tl(), boundRect[o].br());    
    //Mat img1 = frame2(roi2);
    imshow("frame2", frame1);    
    pressFColor(frame1);*/    
    if(yeallow > 0 && right_l_val < 0.1){
     //cout << "YES!!!" << endl;
     red = 0;
     left_speed = 0;
     right_speed = 0;
     tpause -= timeStep;
     if (tpause < 0){
     //sleep(1);
     char message1[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
  
     const double* position1 = gps->getValues(); // Get the current gps position of the robot
     int x = (int) (position1[0] * 100.); // Get the xy coordinates, multiplying by 100 to convert from meters to cm 
     int y = (int) (position1[2] * 100.); // We will use these coordinates as an estimate for the victim's position
     //x = (int) (gps->getValues()[0] * 100.);
     //y = (int) (gps->getValues()[2] * 100.);
     int victim_pos1[2] = {x, y};
 
     memcpy(message1, victim_pos1, sizeof(victim_pos1)); // Copy the victim position into the message array
     message1[8] = 'O'; // The victim type
     
     emitter->send(message1, sizeof(message1)); 
     
     cout << "X: " << (int )(gps->getValues()[0] * 100.) << "Y: " << gps->getValues()[2] << endl;
     cout << "X: " << victim_pos1[0] << "Y: " << (int) position1[2] * 100 << endl;
     cout << message1[8]  << " " << sizeof(message1) << endl;
     yeallow = -988;
     }
    }
    else if (red > 0 && right_l_val<0.1){
     Mat frame2(cam2->getHeight(), cam2->getWidth(), CV_8UC4, (void*)cam2->getImage());
     imshow("frame2", frame2);
     pressFColor(frame2);
     Mat frame1(cam1->getHeight(), cam1->getWidth(), CV_8UC4, (void*)cam1->getImage());
     imshow("frame11", frame1);
     pressFColor(frame1);      
     if(yeallow <= 0){
      left_speed = 0;
      right_speed = 0;
      tpause -= timeStep;
      if (tpause < 0){
     //sleep(1);
       char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
  
       const double* position = gps->getValues(); // Get the current gps position of the robot
       int x = (int) (position[0] * 100.); // Get the xy coordinates, multiplying by 100 to convert from meters to cm 
       int y = (int) (position[2] * 100.); // We will use these coordinates as an estimate for the victim's position
     //x = (int) (gps->getValues()[0] * 100.);
     //y = (int) (gps->getValues()[2] * 100.);
       int victim_pos[2] = {x, y};
 
       memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim posit3ion into the message array
       message[8] = 'F'; // The victim type
     
       emitter->send(message, sizeof(message)); 
     
       cout << "X: " << (int )(gps->getValues()[0] * 100.) << "Y: " << gps->getValues()[2] << endl;
       cout << "X: " << victim_pos[0] << "Y: " << (int) position[2] * 100 << endl;
       cout << message[8]  << " " << sizeof(message) << endl;
       red = -988;
      }
     }
    }           
   
    //left_motor->setVelocity(0.0);
    //right_motor->setVelocity(0.0);
//   left_motor->setVelocity(left_speed);
//    right_motor->setVelocity(right_speed);   
    //cout << counter  << "  " << top_l_val << " " << right_l_val << " " << left_l_val << endl;
    if((abs(left_l_val - last_left_l) < 0.00001) && (abs(last_right_l - right_l_val) < 0.0001 ) && (abs(last_top_l- top_l_val) < 0.00001)){
     counter = counter + 1;  
    }
    else if(counter > 0){
     counter = 0;
    }
    if(counter > 100){
     backrotate = 1;
     leftEncoder->getValue(); 
     cout << "1";
     counter = 0;    
    }
    last_left_l = left_l_val;
    last_right_l = right_l_val;
    last_top_l = top_l_val;
    
    waitKey(1);
  }

  // Enter here exit cleanup code.

  //delete robot;
  return 0;
}
/*void map(){
  string flattened = "";
  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {

      flattened += map[i][j] + ","; // Flatten the array with comma separators
    }
  }
  
  flattened.pop_back(); // Remove the last unnecessary comma

  char message[8 + flattened.size()]; // message array

  memcpy(message, &width, sizeof(width)); // The first 2 integers in the message array are width, height
  memcpy(&message[4], &height, sizeof(height));

  memcpy(&message[8], flattened.c_str(), flattened.size()); // Copy in the flattened map afterwards

  while (robot->step(timeStep) != -1) {
    
    emitter->send(message, sizeof(message)); // Send map data

    char msg = 'M'; // Send map evaluate request
    emitter->send(&msg, sizeof(msg));

    msg = 'E'; // Send an Exit message to get Map Bonus
    emitter->send(&msg, sizeof(msg));
  }
 }*/

