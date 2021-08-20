// File:          z5206762_MTRN4110_PhaseA.cpp
// Date:          18/06/21
// Description:   Controller of E-puck for Phase A - Driving and Perception
// Author:        Stefanie Sos (z5206762)
// Modifications: N/A
// Platform:      Windows
// Notes:         Thanks for marking my project! I hope you have a wonderful week!

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <Windows.h>
#include <dos.h>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>

#define PI 3.14159 
#define FWD_DISTANCE 165
#define WHEEL_RADIUS 20.1
#define AXLE_LENGTH 57

#define TIME_STEP 64
#define TURN AXLE_LENGTH * PI / 4 / WHEEL_RADIUS
//2.2350
#define FORWARD FWD_DISTANCE/WHEEL_RADIUS
//8.253
#define MAX_SPEED 6.28



using namespace webots;
using namespace std;

void runBangBangMode(Robot *robot) {
  const auto epuckRobotName = "e-puck";
  if (robot->getName() == epuckRobotName) {
    cout << "Running Bang Bang Mode" << std::endl;
    int timeStep = (int)robot->getBasicTimeStep();
    
    DistanceSensor *ds[3]; 
    char dsNames[3][4] = {
      "dsf", "dsr", "dsl"
    };
    
    for (int i = 0; i < 3; i++) {
      ds[i] = robot->getDistanceSensor(dsNames[i]);
      ds[i]->enable(TIME_STEP);
    }
    
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(0.0);
    rightMotor->setPosition(0.0);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    
    cout << "[z5206762_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << endl;
    
    fstream inputDirections;
    inputDirections.open("../../MotionPlan.txt", ios::in);
    
    string map;
    
    if (inputDirections.is_open()) {
        while(getline(inputDirections, map)) {
           cout << "[z5206762_MTRN4110_PhaseA] Motion Plan: "<<  map << "\n";
        }
        inputDirections.close();
    }
    cout << "[z5206762_MTRN4110_PhaseA] Motion plan read in!" << endl << "[z5206762_MTRN4110_PhaseA] Executing motion plan..." << endl;
    
    ofstream motionEx;
    motionEx.open("../../MotionExecution.csv");
    motionEx << "Step, Row, Column, Heading, Left Wall, Front Wall, Right Wall \n";
    
    int sum = 0;
    int time = 55;
    char left_sensor = ' ';
    char right_sensor = ' ';
    char front_sensor = ' ';
    int Step = 0;
    int row = (int) map[0] - 48;
    int column = (int) map[1] - 48;
    char heading = map[2];
    double previousL = 0;
    double previousR = 0;
    double newL = 0;
    double newR = 0;
    
    while (robot->step(timeStep) != -1) {
      // 00SFLFFLFRFRFFFLFRFLFFLFRFLFLFFF 
      if (map[Step + 3] == '\0') {
        break;
      }
      
      double dsValues[3];
      for (int i = 0; i < 3 ; i++)
        dsValues[i] = ds[i]->getValue();
      
      bool front_obstacle =
        dsValues[0] < 1000;
      bool left_obstacle =
        dsValues[2] < 1000;
      bool right_obstacle =
        dsValues[1] < 1000;
  
      if (front_obstacle) {
        front_sensor = 'Y';
      } else {
        front_sensor = 'N';
      }
      if (left_obstacle) {
        left_sensor = 'Y';
      } else {
        left_sensor = 'N';
      }
      if (right_obstacle) {
        right_sensor = 'Y';
      } else {
        right_sensor = 'N';
      }
      
      
      if (Step == 0 && sum == 1) {
        cout << "[z5206762_MTRN4110_PhaseA] Step: " << setfill('0') << setw(3) << Step << ", Row: " << (int) map[0] - 48 << ", Column: " << (int) map[1] - 48 << ", Heading: " << heading << ", Left Wall: " << left_sensor << ", Front Wall: " << front_sensor << ", Right Wall: " << right_sensor << endl;
        motionEx << Step << "," << (int) map[0] - 48 << "," << (int) map[1] - 48 << "," << heading << "," << left_sensor << "," << front_sensor << "," << right_sensor << endl;
      }
      
      if (sum <= time) {    
        //cout << sum << endl;
        if ((time - sum) == 20 && map[Step + 3] == 'F') {
          leftMotor->setVelocity(0.5*MAX_SPEED);
          rightMotor->setVelocity(0.5*MAX_SPEED);
        } else if ((time-sum) == 15 && map[Step + 3] != 'F') {
          leftMotor->setVelocity(0.15*MAX_SPEED);
          rightMotor->setVelocity(0.15*MAX_SPEED);
        }
        if (sum == 0 ) {
          if (map[Step + 3] == 'F') {
            time = 55;
            leftMotor->setVelocity(6.28);
            rightMotor->setVelocity(6.28);
            newL = FORWARD;
            newR = FORWARD;
            while (1) {
              if (map[Step + 4] == 'F') {
                newL += FORWARD;
                newR += FORWARD;
                time += 40;
                Step++;
              } else {
                break;
              }
            }
            if (heading == 'S') {
              row++;
            } else if (heading == 'N') {
              row--;
            } else if (heading == 'E') {
              column++;
            } else if (heading == 'W') {
              column--;
            }
          } else if(map[Step + 3] == 'L') {
            time = 40;
            leftMotor->setVelocity(2.5);
            rightMotor->setVelocity(2.5);
            newL = -TURN;
            newR = TURN;
            if (map[Step + 4] == 'L') {
              newL += -TURN;
              newR += TURN;
              time += 40;
              Step++;
            }
            if (heading == 'S') {
              heading = 'E';
            } else if (heading == 'N') {
              heading = 'W';
            } else if (heading == 'E') {
              heading = 'N';
            } else if (heading == 'W') {
              heading = 'S';
            }
          } else if(map[Step + 3] == 'R') {
            time = 40;
            leftMotor->setVelocity(2.5);
            rightMotor->setVelocity(2.5);
            newL = TURN;
            newR = -TURN;
            if (map[Step + 4] == 'R') {
              newL += TURN;
              newR += -TURN;
              time += 40;
              Step++;
            }
            if (heading == 'S') {
              heading = 'W';
            } else if (heading == 'N') {
              heading = 'E';
            } else if (heading == 'E') {
              heading = 'S';
            } else if (heading == 'W') {
              heading = 'N';
            }
          } 
          previousL = previousL + newL;
          previousR = previousR + newR;
        }
        
        leftMotor->setPosition(previousL);
        rightMotor->setPosition(previousR);
          
        if (sum == time) {
          time = 55;
          Step++;
          cout << "[z5206762_MTRN4110_PhaseA] Step: " << setfill('0') << setw(3) << Step << ", Row: " << row << ", Column: " << column << ", Heading: " << heading << ", Left Wall: " << left_sensor << ", Front Wall: " << front_sensor << ", Right Wall: " << right_sensor << endl;
          motionEx << Step << "," << row << "," << column << "," << heading << "," << left_sensor << "," << front_sensor << "," << right_sensor << endl;
          sum = -1;
        }
      }
      sum++;
    };
    motionEx.close();
    cout << "[z5206762_MTRN4110_PhaseA] Motion plan executed!" << endl;
  }
  
}
