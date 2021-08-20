#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/DistanceSensor.hpp>
#include <array>

#define NUM_SENSOR 4

double getActualDistance(double sensorReading) {
    double distance {0.0};
    if (1000 <= sensorReading) {
        distance = 0.01;
    } else if (500 <= sensorReading && sensorReading < 1000) {
        distance = 0.09/500.0*(1000.0-sensorReading) + 0.01;
    } else if (50 <= sensorReading && sensorReading < 500) {
        distance = 0.8/450*(500.0-sensorReading) + 0.1;
    } else if (10 <= sensorReading && sensorReading < 50) {
        distance = 0.1/40.0*(50-sensorReading) + 0.9;
    } else { 
        distance = 1.0;
    }
    return distance;
}

void runEpuckManualControlMode(Robot *robot, int timeStep) {
    const auto epuckRobotName = "e-puck";
    if (robot->getName() == epuckRobotName) {
        const auto START_MESSAGE = "Manual Control Mode of epuck is now running...";
        std::cout << START_MESSAGE << std::endl;
        const auto leftMotorDevice = "left wheel motor";
        const auto rightMotorDevice = "right wheel motor";
    
        auto keyboard = robot->getKeyboard(); 
        keyboard->enable(timeStep);
        
        auto leftMotor = robot->getMotor(leftMotorDevice);
        auto rightMotor = robot->getMotor(rightMotorDevice);
        rightMotor->setPosition(INFINITY);
        leftMotor->setPosition(INFINITY);
        auto leftSpeed = 0.0;
        auto rightSpeed = 0.0;
        
        std::array<std::string, NUM_SENSOR> dsNames {
            "dss", "dsl", "dsr", "dsf"
        };
        
        std::array<std::string, NUM_SENSOR> dsActualNames {
            "Back Distance Sensor", "Left Distance Sensor", "Right Distance Sensor", "Front Distance Sensor"
        };
        
        DistanceSensor *sensors[NUM_SENSOR];
        for (auto i = 0; i < NUM_SENSOR; i++) {
            sensors[i] = robot->getDistanceSensor(dsNames[i]);
            sensors[i]->enable(timeStep);
        }
        
        auto prevKey{-1};
        while (robot->step(timeStep) != -1) {
            auto secondKey = keyboard->getKey();
            if (secondKey != prevKey && secondKey >= 0) {
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
                } else if (secondKey == webots::Keyboard::RIGHT) {
                    leftSpeed = 0.1 * MAX_MOTOR_SPEED;
                    rightSpeed = -0.8 * MAX_MOTOR_SPEED;
                    leftMotor->setVelocity(leftSpeed);
                    rightMotor->setVelocity(rightSpeed);
                } else if (secondKey == webots::Keyboard::LEFT) {
                    leftSpeed = -0.8 * MAX_MOTOR_SPEED;
                    rightSpeed = 0.1 * MAX_MOTOR_SPEED;
                    leftMotor->setVelocity(leftSpeed);
                    rightMotor->setVelocity(rightSpeed);
                } else {
                    leftMotor->setVelocity(leftSpeed);
                    rightMotor->setVelocity(rightSpeed);
                }         
            }
            
            prevKey = secondKey;  
            
            std::array<double, NUM_SENSOR> dsValues;
            for (auto i = 0; i < NUM_SENSOR; ++i) {
                dsValues[i] = sensors[i]->getValue();  
                auto distance = getActualDistance(dsValues[i]);
                std::cout << "Distance from E-puck to " << dsActualNames[i] << " is: " << distance << "m" << std::endl;
                robot->step(timeStep);
            }
        }  
        
        
    } 
}