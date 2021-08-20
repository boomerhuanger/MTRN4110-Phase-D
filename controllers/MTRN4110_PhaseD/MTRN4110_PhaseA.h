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

constexpr double MAX_MOTOR_SPEED {6.28}; // Max speed of the robot motor in rad/s

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
#define TIME_BETWEEN_STEPS 1.4
#define SENSOR_VALUE 995

const auto PREFIX = "[z5206979_MTRN4110_PhaseA] ";
const auto MOTION_PLAN_PATH = "../../MotionPlan.txt";
const auto MOTION_EXECUTION_PATH = "../../MotionExecution.csv";
const auto LEFT_WHEEL_MOTOR = "left wheel motor";
const auto RIGHT_WHEEL_MOTOR = "right wheel motor";
const auto OUTPUT_COLUMNS = "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall,\n";
const auto OUTPUT_INFO = "Step: %03d, Row: %d, Column: %d, Heading: %c, Left Wall: %c, Front Wall: %c, Right Wall: %c\n";
const auto EXECUTING_MOTION_PLAN_MESSAGE = "Executing motion plan...\n";
const auto MOTION_PLAN_EXECUTED_MESSAGE = "Motion plan executed!\n";
const auto UNABLE_TO_OPEN_FILE_MESSAGE = "Unable to open file";
const auto MOTION_PLAN_READ_IN_MESSAGE = "Motion plan read in!\n";
const auto READING_IN_MOTION_PLAN_MESSAGE = "Reading in motion plan from ";
const auto START_MESSAGE = "Phase A is now running...";
const auto NORTH_DIRECTION = 'N';
const auto SOUTH_DIRECTION = 'S';
const auto EAST_DIRECTION = 'E';
const auto WEST_DIRECTION = 'W';
const auto YES = 'Y';
const auto NO = 'N';
const auto FORWARD_DIRECTION = 'F';
const auto LEFT_DIRECTION = 'L';
const auto RIGHT_DIRECTION = 'R';
const auto epuckRobotName = "e-puck";

// Helper function prototypes
string readMotionPlan(string path, bool wallFollowing);
void moveForward(Robot *robot, double *ld, double *rd);
void turnRobot(char turn, Robot *robot, double *ld, double *rd);
void adjustRowCol(int *row, int *col, char heading);
char getHeading(char h, char turn);
void detectWalls(char walls[], DistanceSensor *sensors[]);


// This is the main program of your controller.
void runPhaseA(Robot *robot, bool wallFollowing, std::string newWallPath) {
    if (robot->getName() == epuckRobotName) {
        auto startingTime = robot->getTime();
        std::cout << START_MESSAGE << std::endl;
    
        // Enable left, front and right distance sensors
        // and create array to store detected walls
        char walls[NUM_SENSORS];
        char dsNames[NUM_SENSORS][4] = {"dsl", "dsf", "dsr"};
        DistanceSensor *sensors[NUM_SENSORS];
        for (auto i = LEFT_SENSOR; i < NUM_SENSORS; i++) {
            sensors[i] = robot->getDistanceSensor(dsNames[i]);
            sensors[i]->enable(TIME_STEP);
        }
        
        // Open CSV file to output motion execution
        ofstream csvfile;
        csvfile.open(MOTION_EXECUTION_PATH);
        csvfile << OUTPUT_COLUMNS;
      
        // Read Motion Plan line from file
        string mpline = readMotionPlan(MOTION_PLAN_PATH, wallFollowing);
        cout << PREFIX << EXECUTING_MOTION_PLAN_MESSAGE;
        auto step = 0;
        auto row = mpline[0] - '0';
        auto col = mpline[1] - '0';
        auto heading = mpline[2];
        auto mp = mpline.erase(0,3);
        
        // Initialise required variables
        if (wallFollowing == true) {
            mp = newWallPath;
        }
        auto numMoves = (int)mp.length();
        
        // Main loop
        auto active = true;
        auto leftDist = 0.0;
        auto rightDist = 0.0;
        while (robot->step(TIME_STEP) != -1 and step <= numMoves) {
            auto time = robot->getTime() - startingTime;
    
          // Check if robot is in motion
            if (active) {
                // Check if 2 seconds have passed since last motion command
                if (time >= step * TIME_BETWEEN_STEPS) {
                    active = false;
                    detectWalls(walls, sensors);
                    
                    // Print state to console
                    cout << PREFIX;
                    printf(OUTPUT_INFO, 
                            step, row, col, heading, walls[LEFT_SENSOR], walls[FRONT_SENSOR], walls[RIGHT_SENSOR]);
                    csvfile <<step<<","<<row<<","<<col<<","<<heading<<","<<walls[LEFT_SENSOR]<<","<<walls[FRONT_SENSOR]<<","<<walls[RIGHT_SENSOR]<<",\n";
                    step++;
                
                // Otherwise continue looping until 2.5 seconds have passed
                } else continue;
            
            // If robot is not moving (i.e. 2.5 seconds have passed)
            // Execute next motion command
            } else {
                if (step != 0) {
                    auto move = mp[step-1];
                
                // Move the robot depending on current move in motion plan
                if (move == FORWARD_DIRECTION) {
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
      
        
        cout << PREFIX << MOTION_PLAN_EXECUTED_MESSAGE;
        csvfile.close();
    }
}






/*
 * Helper Functions
 */


// Reads the Motion Plan from the given text file
string readMotionPlan(string path, bool wallFollowing) {
    string plan = " ";
    if (wallFollowing == false) {
        string line;
        ifstream file(path);
        cout << PREFIX << READING_IN_MOTION_PLAN_MESSAGE << path << "...\n";
        if (file.is_open()) {
            getline(file, line);
            plan = line;
            cout << PREFIX << "Motion Plan: " << line << '\n';
            cout << PREFIX << MOTION_PLAN_READ_IN_MESSAGE;
            file.close();
        }
        else cout << UNABLE_TO_OPEN_FILE_MESSAGE;
    }
    return plan;  
}


// Moves the robot forward one square
void moveForward(Robot *robot, double *ld, double *rd) {
    Motor *leftMotor = robot->getMotor(LEFT_WHEEL_MOTOR);
    Motor *rightMotor = robot->getMotor(RIGHT_WHEEL_MOTOR);
    
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
    Motor *leftMotor = robot->getMotor(LEFT_WHEEL_MOTOR);
    Motor *rightMotor = robot->getMotor(RIGHT_WHEEL_MOTOR);
    
    // Calculate velocity
    // float velo = MAX_SPEED*0.70;
    float velo = WHEEL_RADIUS * MAX_SPEED / AXLE_LENGTH;
    
    if (turn == LEFT_DIRECTION) {
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
        case NORTH_DIRECTION:
            *row = *row - 1;
            break;
        case EAST_DIRECTION:
            *col = *col + 1;
            break;
        case SOUTH_DIRECTION:
            *row = *row + 1;
            break;
        case WEST_DIRECTION:
            *col = *col - 1;
            break;  
    }
}


// Return current heading of robot
char getHeading(char h, char turn) {
    char newHeading = h;
    if (turn == FORWARD_DIRECTION) {
        return newHeading;
    }
    switch (h) {
        case NORTH_DIRECTION: 
            newHeading = (turn == LEFT_DIRECTION) ? WEST_DIRECTION : EAST_DIRECTION;
            break;
        case EAST_DIRECTION:
            newHeading = (turn == LEFT_DIRECTION) ? NORTH_DIRECTION : SOUTH_DIRECTION;
            break;
        case SOUTH_DIRECTION:
            newHeading = (turn == LEFT_DIRECTION) ? EAST_DIRECTION : WEST_DIRECTION;
            break;
        case WEST_DIRECTION:
            newHeading = (turn == LEFT_DIRECTION) ? SOUTH_DIRECTION : NORTH_DIRECTION;
            break;
    }
    return newHeading;
}


// Detect walls on left, right and front of robot
void detectWalls(char walls[], DistanceSensor *sensors[]) {
    for (auto i = 0; i < NUM_SENSORS; i++) {
        walls[i] = (sensors[i]->getValue() < SENSOR_VALUE) ? YES : NO;
    }
}