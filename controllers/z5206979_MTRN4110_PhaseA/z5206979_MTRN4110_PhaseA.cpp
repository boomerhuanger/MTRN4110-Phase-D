/*
 * File:          z5206979_MTRN4110_PhaseA.cpp
 * Date:          06/06/2021
 * Description:   Controller of E-puck for Phase A - Driving and Perception
 * Author:        Noah Correa
 * Modifications: 
 * Platform:      Windows
 * Notes:         The motion is sequence is NOT hard-coded into the program
 */


// Includes
#include <iostream>
#include <fstream>
#include <string>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

// Namespaces
using namespace std;
using namespace webots;

// Constants
#define PI 3.14159 
#define FWD_DISTANCE 165
#define WHEEL_RADIUS 20.1
#define AXLE_LENGTH 57
#define MAX_SPEED 6.28
#define TIME_STEP 64
#define NUM_SENSORS 3
#define LEFT_SENSOR 0
#define FRONT_SENSOR 1
#define RIGHT_SENSOR 2

const string PREFIX = "[z5206979_MTRN4110_PhaseA] ";
const string MOTION_PLAN_PATH = "../../MotionPlan.txt";
const string MOTION_EXECUTION_PATH = "../../MotionExecution.csv";

// Helper function prototypes
string readMotionPlan(string path);
void moveForward(Robot *robot, double *ld, double *rd);
void turnRobot(char turn, Robot *robot, double *ld, double *rd);
void adjustRowCol(int *row, int *col, char heading);
char getHeading(char h, char turn);
void detectWalls(char walls[], DistanceSensor *sensors[]);


// This is the main program of your controller.
int main(int argc, char **argv) {

  // Create the Robot instance.
  Robot *robot = new Robot();
   
  // Enable left, front and right distance sensors
  // and create array to store detected walls
  char walls[NUM_SENSORS];
  char dsNames[NUM_SENSORS][4] = {"dsl", "dsf", "dsr"};
  DistanceSensor *sensors[NUM_SENSORS];
  for (int i = LEFT_SENSOR; i < NUM_SENSORS; i++) {
    sensors[i] = robot->getDistanceSensor(dsNames[i]);
    sensors[i]->enable(TIME_STEP);
  }
  
  // Open CSV file to output motion execution
  ofstream csvfile;
  csvfile.open(MOTION_EXECUTION_PATH);
  csvfile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall,\n";


  // Read Motion Plan line from file
  string mpline = readMotionPlan(MOTION_PLAN_PATH);
  cout << PREFIX << "Executing motion plan...\n";
  

  // Initialise required variables
  int step = 0;
  int row = mpline[0] - '0';
  int col = mpline[1] - '0';
  char heading = mpline[2];
  string mp = mpline.erase(0, 3);
  int numMoves = mp.length();


  // Main loop
  bool active = true;
  double leftDist = 0.0;
  double rightDist = 0.0;
  while (robot->step(TIME_STEP) != -1 and step <= numMoves) {
  
    // Check if robot is in motion
    if (active) {
      // Check if 2 seconds have passed since last motion command
      if (robot->getTime() >= step * 2.5) {
        active = false;
        detectWalls(walls, sensors);
        
        // Print state to console
        cout << PREFIX;
        printf("Step: %03d, Row: %d, Column: %d, Heading: %c, Left Wall: %c, Front Wall: %c, Right Wall: %c\n", 
                step, row, col, heading, walls[LEFT_SENSOR], walls[FRONT_SENSOR], walls[RIGHT_SENSOR]);
        csvfile <<step<<","<<row<<","<<col<<","<<heading<<","<<walls[LEFT_SENSOR]<<","<<walls[FRONT_SENSOR]<<","<<walls[RIGHT_SENSOR]<<",\n";
        step++;
       
      // Otherwise continue looping until 2.5 seconds have passed
      } else continue;
     
    // If robot is not moving (i.e. 2.5 seconds have passed)
    // Execute next motion command
    } else {
      if (step != 0) {
        char move = mp[step-1];
        
        // Move the robot depending on current move in motion plan
        if (move == 'F') {
          // Move robot forward, then adjust row and column indices
          moveForward(robot, &leftDist, &rightDist);
          adjustRowCol(&row, &col, heading);
            
        } else {
          // Turn robot left or right, then adjust heading
          turnRobot(move, robot, &leftDist, &rightDist);
          heading = getHeading(heading, move);
        }
        active = true;
      }
    }  
  };

  
  cout << PREFIX << "Motion plan executed!\n";
  csvfile.close();
  
  delete robot;
  return 0;
}






/*
 * Helper Functions
 */


// Reads the Motion Plan from the given text file
string readMotionPlan(string path) {
  string line;
  ifstream file(path);
  cout << PREFIX << "Reading in motion plan from " << path << "...\n";
  if (file.is_open()) {
    getline(file, line);
    cout << PREFIX << "Motion Plan: " << line << '\n';
    cout << PREFIX << "Motion plan read in!\n";
    file.close();
  }

  else cout << "Unable to open file";
  
  return line;  
}


// Moves the robot forward one square
void moveForward(Robot *robot, double *ld, double *rd) {
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  *ld = *ld + FWD_DISTANCE/WHEEL_RADIUS;
  *rd = *rd + FWD_DISTANCE/WHEEL_RADIUS;
  
  // Set motors
  leftMotor->setVelocity(MAX_SPEED);
  rightMotor->setVelocity(MAX_SPEED);
  leftMotor->setPosition(*ld);
  rightMotor->setPosition(*rd);
 
   
}


// Turns the robot left or right 90 degrees
void turnRobot(char turn, Robot *robot, double *ld, double *rd) {
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  // Calculate velocity
  // float velo = MAX_SPEED*0.70;
  float velo = WHEEL_RADIUS * MAX_SPEED / AXLE_LENGTH;
  
  if (turn == 'L') {
    *ld = *ld - AXLE_LENGTH * PI / 4 / WHEEL_RADIUS;
    *rd = *rd + AXLE_LENGTH * PI / 4 / WHEEL_RADIUS;
    // turn left
    leftMotor->setVelocity(velo);
    rightMotor->setVelocity(velo);
    leftMotor->setPosition(*ld);
    rightMotor->setPosition(*rd);
    
  } else {
    *ld = *ld + AXLE_LENGTH * PI / 4 / WHEEL_RADIUS;
    *rd = *rd - AXLE_LENGTH * PI / 4 / WHEEL_RADIUS;
    // turn right
    leftMotor->setVelocity(velo);
    rightMotor->setVelocity(velo);
    leftMotor->setPosition(*ld);
    rightMotor->setPosition(*rd);
    
  }
  
}


// Updates Row and Column numbers if robot moved forward
void adjustRowCol(int *row, int *col, char heading) {
  switch (heading) {
    case 'N':
      *row = *row - 1;
      break;
    case 'E':
      *col = *col + 1;
      break;
    case 'S':
      *row = *row + 1;
      break;
    case 'W':
      *col = *col - 1;
      break;  
  }
}


// Return current heading of robot
char getHeading(char h, char turn) {
  char newHeading = h;
  switch (h) {
    case 'N': 
      newHeading = (turn == 'L') ? 'W' : 'E';
      break;
    case 'E':
      newHeading = (turn == 'L') ? 'N' : 'S';
      break;
    case 'S':
      newHeading = (turn == 'L') ? 'E' : 'W';
      break;
    case 'W':
      newHeading = (turn == 'L') ? 'S' : 'N';
      break;
  }
  return newHeading;
}


// Detect walls on left, right and front of robot
void detectWalls(char walls[], DistanceSensor *sensors[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    walls[i] = (sensors[i]->getValue() < 995) ? 'Y' : 'N';
  }
}

