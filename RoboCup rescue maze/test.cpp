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
double last;
double last_top_l;
double last_left_l;
double last_right_l;
int count = 0;
int leftrotate = 0;
int rightrotate = 0;
int toprotate = 0;
long tpause = 2000;
using namespace webots;
using namespace std; 
using namespace cv;
vector<vector<int>> myColors{{61,83,209,255,137,198}, //Green
                             {110,178,149,254,124,200},   //Red
                             {16,41,134,200,190,241}, //Yealow
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
void getConturs(Mat imgDil){

 vector<vector<Point>> contours;
 vector<Vec4i> hierarchy;
 findContours(imgDil,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

 for(int i = 0; i < contours.size(); i++){
  int area = contourArea(contours[i]);
  vector<vector<Point>>conPoly(contours.size());
  vector<Rect>boundRect(contours.size());
  string objectType;
   //cout << "! " << area << endl;
   if(area > 5){
    number = number + 1;
      
    float peri = arcLength(contours[i],true);
    approxPolyDP(contours[i],conPoly[i],0.02 * peri,true);
    boundRect[i] = boundingRect(conPoly[i]);

    drawContours(imgDil, conPoly,i,Scalar(0,255,0),2);
  }
 }
}

void pressFColor(Mat img){
 Mat imgHSV;
 cvtColor(img, imgHSV,COLOR_BGR2HSV);
  Scalar lower(myColors[1][0],myColors[1][2],myColors[1][4]);
  Scalar upper(myColors[1][1],myColors[1][3],myColors[1][5]);
  Mat mask;
  inRange(imgHSV,lower,upper,mask);
  imshow("chebureck",mask);
  getConturs(mask);
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
     //cout << rightEncoder->getValue() << " " << leftEncoder->getValue() <<" ";
     //cout << left_l_val << " " << top_l_val << " " <<right_l_val << " " <<endl;
     

     if(top_l_val > 0.07 && down_val < 0.13){
       left_speed = MAX_SPEED + right_l_val * 5;
       right_speed = MAX_SPEED - right_l_val * 5;
       
       if (right_l_val < 0.07)  left_speed =  left_speed * 0.7;
       if (left_l_val < 0.07)  right_speed =  right_speed * 0.7;
     }
     else {
     
       left_speed = -MAX_SPEED;
       right_speed = MAX_SPEED;
     
     }

      /*while(leftEncoder->getValue() - last < 35.0){
       left_motor->setVelocity(-left_speed);
       right_motor->setVelocity(right_speed);
       leftEncoder->getValue();
      } */   
      //left_motor->setVelocity(left_speed);
      //right_motor->setVelocity(right_speed); 
   
    Mat frame(cam2->getHeight(), cam2->getWidth(), CV_8UC4, (void*)cam2->getImage());
    
    imshow("frame", frame);
    pressFColor(frame);
    if (number > 0){
     left_speed = 0;
     right_speed = 0;
     tpause -= timeStep;
     if (tpause < 0){
     //sleep(1);
     char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
  
     const double* position = gps->getValues(); // Get the current gps position of the robot
     int x = (int) position[0] * 100; // Get the xy coordinates, multiplying by 100 to convert from meters to cm 
     int y = (int) position[2] * 100; // We will use these coordinates as an estimate for the victim's position
 
     int victim_pos[2] = {x, y};
 
     memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
     message[8] = 'F'; // The victim type is harmed
 
     emitter->send(message, sizeof(message)); 
     
     cout << "X: " << gps->getValues()[0] << "Y: " << gps->getValues()[2] << endl;
     number = -988;
     }
    }           
   
    //left_motor->setVelocity(0.0);
    //right_motor->setVelocity(0.0);
    left_motor->setVelocity(left_speed);
    right_motor->setVelocity(right_speed);
   /* if((last_left_l == left_l_val) && (last_right_l = right_l_val) && (last_top_l == top_l_val)){
     count = count + 1;  
    }
    else if(count > 0){
     count = 0;
    }
    if(count > 100){
     left_motor->setVelocity(-2 * MAX_SPEED);
     right_motor->setVelocity(-2 * MAX_SPEED);
     cout << "1";    
    }
    last_left_l = left_l_val;
    last_right_l = right_l_val;
    last_top_l = top_l_val;
    */
    waitKey(1);
  }

  // Enter here exit cleanup code.

  //delete robot;
  return 0;
}
