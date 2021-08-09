// File:          EpuckController.cpp
// Date:          20/06/2021
// Description:   Controller of E-puck for Phase A- Driving and Perception
// Author:        William Huang
// Modifications: 
// Platform: Windows

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

constexpr auto DISTANCE_SENSOR_COUNT {3}; //total number of distance sensors
constexpr double MAX_MOTOR_SPEED {6.28}; // Max speed of the robot motor in rad/s
constexpr double OBSTACLE_THRESHOLD {600.0}; // threshold for detecting front obstacle
constexpr int TIME_BETWEEN_CELLS{1250}; //time used to travel from one cell to another adjacent cell
constexpr int TOTAL_COLUMNS{7}; //total amount of columns in the CSV file
constexpr double PI{3.14159};
constexpr char LEFT{'L'}; //left relative to the robot
constexpr char RIGHT{'R'}; //right relative to the robot
constexpr char FORWARD{'F'}; //forward relative to the robot
constexpr char NORTH{'N'}; //north relative to the world
constexpr char EAST{'E'}; //east relative to the world
constexpr char SOUTH{'S'}; //south relative to the world
constexpr char WEST{'W'}; //west relative to the world
constexpr char ZERO{'0'};
constexpr char NO{'N'};
constexpr char YES{'Y'};

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//CsvProcessor class which is used to open and write to a CSV file

class CsvProcessor {
public:
    CsvProcessor(std::string fileName, char delimeter = ',')   // constructor
        : fileName{fileName}, delimeter{delimeter} {};               
    void writeLineToCsv (const std::vector<std::string> &dataLine) const;  // write line to csv
private:
    std::string fileName;   // csv file name
    char delimeter;  //delimeter    
    std::string errorMessage = "Writing new line to failed! If the CSV file is already open, close the file and try again";                        
};

// Write line to CSV file

void CsvProcessor::writeLineToCsv (const std::vector<std::string> &dataLine) const {
    std::ofstream fout {fileName, std::ios::out|std::ios::app};  // open and truncate file on open
    if (fout.is_open()) {  // check if open successfully
        // write elements (except the last) of dataLine to file, delimited by ','
        for (auto iter = dataLine.begin(); iter != dataLine.end() - 1; ++iter) { 
            fout <<*iter << delimeter;      
        }
        // write the last element of dataLine to file, followed by '\n' and flush
        fout << dataLine.back() << std::endl; 
    } else {
        // throw an error message
        throw std::runtime_error (errorMessage);
    }
}

void turnRobot(double &currentYaw, double &targetYaw, double &oldVelocity, Motor *leftMotor, Motor *rightMotor, Robot *robot, int timeStep, InertialUnit *inertialUnit) {                
    
    robot->step(timeStep);

    //get the current yaw value from the inertial unit
    auto rollPitchYaw = inertialUnit->getRollPitchYaw();
    currentYaw = rollPitchYaw[2];

    //logic based on a proportional controller used to reduce position error while turning the robot by adjusting velocity
    //logic found on: https://cyberbotics.com/doc/reference/motor
    auto error = targetYaw - currentYaw;
    auto desiredVelocity = MAX_MOTOR_SPEED;
    auto currentAcceleration = leftMotor->getAcceleration();
    auto desiredAcceleration = 0.40;
    auto currentLeftMotorVelocity = 12 * error;
    auto signCurrentVelocity = 1;
    auto signCurrentAcceleration = 1;

    if (currentLeftMotorVelocity < 0) {
        signCurrentVelocity = -1;
    }

    if (currentAcceleration < 0) {
        signCurrentAcceleration = -1;
    }

    if (abs(currentLeftMotorVelocity) > desiredVelocity) 
        currentLeftMotorVelocity = signCurrentVelocity * desiredVelocity;
    
    if (desiredAcceleration != -1) {
        currentAcceleration = (currentLeftMotorVelocity - oldVelocity) / timeStep;

        if (abs(currentAcceleration) > desiredAcceleration) {          
            currentAcceleration = signCurrentAcceleration * desiredAcceleration;
        }

    currentLeftMotorVelocity = oldVelocity + currentAcceleration * timeStep; 

    //update the old velocity to be the current velocity for the next iteration 
    oldVelocity = currentLeftMotorVelocity;
    }

    //when turning, the left and right motor velocities should be identical in magnitude but opposite in sign
    leftMotor->setVelocity(-currentLeftMotorVelocity);
    rightMotor->setVelocity(currentLeftMotorVelocity);
}

int main(int argc, char **argv) {

    const auto leftMotorDevice = "left wheel motor";
    const auto rightMotorDevice = "right wheel motor";
    const auto motionPlanFile = "../../MotionPlan.txt";
    const auto motionExecutionFile = "../../MotionExecution.csv";
    const auto inertialUnitDevice = "inertial unit";
    const auto leftDistanceSensorDevice = "left distance sensor";
    const auto rightDistanceSensorDevice = "right distance sensor";
    const auto frontDistanceSensorDevice = "front distance sensor";
    const auto stepMessage = "Step: ";
    const auto stepName = "Step";
    const auto rowMessage = ", Row: ";
    const auto rowName = "Row";
    const auto columnMessage = ", Column: ";
    const auto columnName = "Column";
    const auto headingMessage = ", Heading: ";
    const auto headingName = "Heading";
    const auto leftWallMessage = ", Left Wall: ";
    const auto leftWallName = "Left Wall";
    const auto frontWallMessage = ", Front Wall: ";
    const auto frontWallName = "Front Wall";
    const auto rightWallMessage = ", Right Wall: ";
    const auto rightWallName = "Right Wall";
    const auto zIDMessage = "[z5205986_MTRN4110_PhaseA] ";
    const auto readingInMotionPlanMessage = std::string(zIDMessage) + "Reading in motion plan from ";
    const auto printingMotionPlanMessage = std::string(zIDMessage) + "Motion Plan: ";
    const auto motionPlanReadInMessage = std::string(zIDMessage) + "Motion plan read in!";
    const auto executingMotionPlanMessage = std::string(zIDMessage) + "Executing motion plan...";
    const auto motionPlanExecutedMessage = "Motion plan executed!";
    const auto readingFileFailedMessage = "Reading file failed";
    const auto ellipses = "...";
    
    //create a new Robot instance and get associated sensors 
    auto robot = new Robot();
    auto leftMotor = robot->getMotor(leftMotorDevice);
    auto rightMotor = robot->getMotor(rightMotorDevice);
    auto inertialUnit = robot->getInertialUnit(inertialUnitDevice);

    // get the time step of the current world and enable all sensors
    auto timeStep = 64;
    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);
    inertialUnit->enable(timeStep);
 

    std::array<double, DISTANCE_SENSOR_COUNT> mDistanceSensors;
    std::array<webots::DistanceSensor*, DISTANCE_SENSOR_COUNT> ds;
    std::array<std::string, DISTANCE_SENSOR_COUNT> dsNames {
        frontDistanceSensorDevice, leftDistanceSensorDevice, rightDistanceSensorDevice
    }; 
   
    /*///PHASE D STUFF
    
    auto keyboard = robot->getKeyboard(); 
    keyboard->enable(timeStep);
    
    //auto leftMotor = robot->getMotor(leftMotorDevice);
    //auto rightMotor = robot->getMotor(rightMotorDevice);
     std::cout << "I am here1" << std::endl;
    auto leftSpeed = 0.0;
    auto rightSpeed = 0.0;
    while (robot->step(timeStep) != -1) {
        std::cout << "I am here" << std::endl;
        auto secondKey{0};
        secondKey = keyboard->getKey();
        if (secondKey == webots::Keyboard::UP) {
            leftSpeed = MAX_MOTOR_SPEED;
            rightSpeed = MAX_MOTOR_SPEED;
            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);
        } else if (secondKey == webots::Keyboard::DOWN) {
            leftSpeed = -MAX_MOTOR_SPEED;
            rightSpeed = -MAX_MOTOR_SPEED;
            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);
        } else if (secondKey == webots::Keyboard::LEFT) {
            leftSpeed = -0.1 * MAX_MOTOR_SPEED;
            rightSpeed = 0.8 * MAX_MOTOR_SPEED;
            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);
        } else if (secondKey == webots::Keyboard::RIGHT) {
            leftSpeed = 0.1 * MAX_MOTOR_SPEED;
            rightSpeed = -0.8 * MAX_MOTOR_SPEED;
            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);
        } else {
            leftMotor->setVelocity(leftSpeed);
            rightMotor->setVelocity(rightSpeed);
        }    
    }    */
    
    
    for (auto i = 0; i < DISTANCE_SENSOR_COUNT; ++i) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(timeStep);
    } 
  
    //create a CSV processor and push the column names into the file
    std::array<std::string, TOTAL_COLUMNS> columnNames {
        stepName, rowName, columnName, headingName, leftWallName, frontWallName, rightWallName
    }; 

    CsvProcessor csvProcessor{motionExecutionFile};
    std::vector<std::string> data;

    for (auto i = 0; i < TOTAL_COLUMNS; ++i) {
        data.push_back(columnNames[i]);
    }

    csvProcessor.writeLineToCsv(data);
    
    //prepare to open the motionPlan text file 

    std::ifstream fin {motionPlanFile, std::ios::in};
    std::string motionPlanPath;

    //check whether the file is open
    if (fin.is_open()) {

        //use std::getline to read in the motion plan path
        std::getline(fin, motionPlanPath);

        //print motoin plan path and corresponding messages to the console
        std::cout << readingInMotionPlanMessage << motionPlanFile << ellipses << std::endl;
        std::cout << printingMotionPlanMessage << motionPlanPath << std::endl;
        std::cout << motionPlanReadInMessage << std::endl;
        std::cout << executingMotionPlanMessage << std::endl;

        //get starting information of the robot
        
        auto row = motionPlanPath[0];
        auto column = motionPlanPath[1];
        auto heading = motionPlanPath[2];
        auto step = 0;
        auto overallDirection = heading;
        
        //remove starting information of the robot as no longer needed
        motionPlanPath.erase(0,2);
        
        // Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (robot->step(timeStep) != -1) {
             
            //iterating through the entire motion path
            for(auto &ch : motionPlanPath) {

                //get direction which the robot plans on heading 
                auto heading = ch;

                //if the robot is heading forward relative to itself
                if (heading == FORWARD) {

                    //set the motor velocities to be equal to head forward
                    rightMotor->setVelocity(MAX_MOTOR_SPEED);
                    leftMotor->setVelocity(MAX_MOTOR_SPEED);
                    robot->step(TIME_BETWEEN_CELLS);

                    //update the row and column values after moving forward
                    if (overallDirection == SOUTH) {
                        row += 1;
                    } else if (overallDirection == EAST) {
                        column += 1;
                    } else if (overallDirection == WEST) {
                        column -= 1;
                    } else if (overallDirection == NORTH) {
                        row -= 1;
                    }
                }
                    
                //obtain the current yaw value
                auto targetYaw = 0.0;
                auto oldVelocity = 0.0;
                auto rollPitchYaw = inertialUnit->getRollPitchYaw();
                auto currentYaw = rollPitchYaw[2];
                
                //determine target yaw value based on the overall direction after turning left from its current direction
                if (heading == LEFT) {
                    if (overallDirection == SOUTH) {
                        targetYaw = 0.0;
                    } else if (overallDirection == EAST) {
                        targetYaw = PI/2;
                    } else if (overallDirection == NORTH) {
                        targetYaw = PI;
                    } else if (overallDirection == WEST) {
                        targetYaw = -PI/2;
                        currentYaw = -PI;
                    }

                    //turn the robot left
                    
                    if (currentYaw < targetYaw) {
                        while (currentYaw < targetYaw) {
                            turnRobot(currentYaw, targetYaw, oldVelocity, leftMotor, rightMotor, robot, timeStep, inertialUnit);
                        } 
                    } else if (currentYaw > targetYaw) {
                        while (currentYaw > targetYaw) {
                            turnRobot(currentYaw, targetYaw, oldVelocity, leftMotor, rightMotor, robot, timeStep, inertialUnit);
                        }
                    }

                 //determine target yaw value based on the overall direction after turning right from its current direction
                } else if (heading == RIGHT) {
                    if (overallDirection == SOUTH) {
                        targetYaw = -PI;    
                    } else if (overallDirection == EAST) {
                        targetYaw = -PI/2;
                    } else if (overallDirection == NORTH) {
                        targetYaw = 0.0;
                    } else if (overallDirection == WEST) {
                        targetYaw = PI/2;
                        currentYaw = PI;
                    }

                    //turn the robot right

                    if (currentYaw < targetYaw) {
                        while (currentYaw < targetYaw) {
                            turnRobot(currentYaw, targetYaw, oldVelocity, leftMotor, rightMotor, robot, timeStep, inertialUnit);
                        } 
                    } else if (currentYaw > targetYaw) {
                        while (currentYaw > targetYaw) {
                            turnRobot(currentYaw, targetYaw, oldVelocity, leftMotor, rightMotor, robot, timeStep, inertialUnit);
                        }
                    }
                }

                //updating the overall direction of the robot after turning for the next iteration
                if (overallDirection == SOUTH) {
                    if (heading == LEFT) {
                        overallDirection = EAST;
                    } else if (heading == RIGHT) {
                        overallDirection = WEST;
                    }
                } else if (overallDirection == EAST) {
                    if (heading == LEFT) {
                        overallDirection = NORTH;
                    } else if (heading == RIGHT) {
                        overallDirection = SOUTH;
                    }
                } else if (overallDirection == NORTH) {
                    if (heading == LEFT) {
                        overallDirection = WEST;
                    } else if (heading == RIGHT) {
                        overallDirection = EAST;
                    }
                } else if (overallDirection == WEST) {
                    if (heading == LEFT) {
                        overallDirection = SOUTH;
                    } else if (heading == RIGHT) {
                        overallDirection = NORTH;
                    }
                }
                
                //determine whether there are walls in front, to the left and to the right of the robot
                for (auto i = 0; i < DISTANCE_SENSOR_COUNT ; ++i) {
                    mDistanceSensors[i] = ds[i]->getValue();
                }

                //check if there is a wall in front
                auto isFrontObstacle =
                    mDistanceSensors[0] < OBSTACLE_THRESHOLD;
                
                //check if there is a wall on the left
                auto isLeftObstacle =
                    mDistanceSensors[1] < OBSTACLE_THRESHOLD;
            
                //check if there is a wall on the right
                auto isRightObstacle =
                    mDistanceSensors[2] < OBSTACLE_THRESHOLD;

                auto isFrontWall = NO;
                auto isRightWall = NO;
                auto isLeftWall = NO;

                //if there is an obstacle detected, there is a wall
                if (isFrontObstacle) {
                    isFrontWall = YES;
                }
        
                if (isRightObstacle) {
                    isRightWall = YES;
                }
        
                if (isLeftObstacle) {
                    isLeftWall = YES;
                }
                
                std::vector<std::string> data;
                std::stringstream ss;  

                //ensure that the step is printed as a 3 digit number with zeroes filling the front of the number
                ss << std::setfill(ZERO) << std::setw(3)<< step;

                //convert variables into strings
                auto adjustedStep = ss.str();
                std::string rowAsString(1,row);
                std::string columnAsString(1,column);
                std::string overallDirectionAsString(1,overallDirection);
                std::string frontWallAsString(1,isFrontWall);
                std::string leftWallAsString(1,isLeftWall);
                std::string rightWallAsString(1,isRightWall);

                //push the values for each step and write to CSV
                data.push_back(adjustedStep);
                data.push_back(rowAsString);
                data.push_back(columnAsString);
                data.push_back(overallDirectionAsString);
                data.push_back(leftWallAsString);
                data.push_back(frontWallAsString);
                data.push_back(rightWallAsString);
                csvProcessor.writeLineToCsv(data);
                
                //print out all the relevant information onto the console
                std::cout << zIDMessage << stepMessage << adjustedStep << rowMessage << row << columnMessage << column << headingMessage << overallDirection << leftWallMessage << isLeftWall << frontWallMessage << isFrontWall << rightWallMessage << isRightWall << std::endl;
                step += 1;
            }

            //print how the motion plan has been executed before breaking out of the Webots simulation loop
            std::cout << motionPlanExecutedMessage << std::endl;
            break;
        }
    } else {

        // throw an error message if cannot open file
        throw std::runtime_error (readingFileFailedMessage);
    }

    //stop the motors before deleting the robot
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    delete robot;
    return 0;
}

