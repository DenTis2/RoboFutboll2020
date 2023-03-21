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
int counter = 0;
int leftrotate = 0;
int rightrotate = 0;
int backrotate = 0;
int tur;
int dich = 0;
long tpause = 2048;
using namespace webots;
using namespace std; 
using namespace cv;
vector<vector<int>> myColors{{61,83,209,255,137,198}, //Green
                             {110,178,149,254,124,200},   //Red
                             {20,30,100,255,100,255}, //Yealow
                             {11,110,0,160,0,67},   //Black
                             {0,172,0,111,168,255},//white
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
   if(area > 5){
    number = 1;
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.02 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);

    //drawContours(frame, conPoly,i,Scalar(0,255,0),2);
  }
 }
}

/*void Bukva(Mat imgB){
 
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
}*/
void pressFColor(Mat img){
 Mat imgHSV;
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
 for(int u=2; u > 0; u--){
  Scalar lower(myColors[u][0],myColors[u][2],myColors[u][4]);
  Scalar upper(myColors[u][1],myColors[u][3],myColors[u][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  //imshow(to_string(u),mask);
  imshow("frame1", mask);
  getConturs(mask);
  if (number == 1){
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
 }
}
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
    down_val = down->getValue();
    top_l_val = top_l->getValue();
    left_l_val = left_l->getValue();
    right_l_val = right_l->getValue();
    //cout<< down_val << endl;
     //rightEncoder->getValue(); 
     // leftEncoder->getValue();
    //double ps = wb_distance_sensor_get_value(ps[1]);
     //cout << rightEncoder->getValue() << " " << leftEncoder->getValue() <<" ";//
     //cout << left_l_val << " " << top_l_val << " " <<right_l_val << " " <<endl;
     //cout << down_val << " "<< top_l_val<< endl;
     if(backrotate == 1){
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
     else{
      if(top_l_val > 0.057 && down_val < 0.2){
       int sp1 = MAX_SPEED + right_l_val * 5;
       int sp2 = MAX_SPEED - right_l_val * 5;
       if(sp1 > 6.28){
        sp1 = 6.28;
       }
       if(sp2 < -6.28){
        sp2 = -6.28;
       }
       left_speed = sp1;
       right_speed = sp2;
       
       if (right_l_val < 0.07)  left_speed =  left_speed * 0.6;
       if (left_l_val < 0.07)  right_speed =  right_speed * 0.6;
      }
      else {
       last = leftEncoder->getValue();
       leftrotate = 1;
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
    imshow("frame2", frame2);
    pressFColor(frame2);
    Mat frame1(cam1->getHeight(), cam1->getWidth(), CV_8UC4, (void*)cam1->getImage());
    imshow("frame11", frame1);
    pressFColor(frame1);    
    if(yeallow > 0){
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
    else if (red > 0){
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
 
       memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
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
    left_motor->setVelocity(left_speed);
    right_motor->setVelocity(right_speed);   
    cout << counter  << "  " << top_l_val << " " << right_l_val << " " << left_l_val << endl;
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
