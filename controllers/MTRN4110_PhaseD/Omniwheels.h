/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Demo of a three-omni-wheels robot
 * Thanks to Mehdi Ghanavati, Shahid Chamran University
 */

#include <stdio.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <cstdlib>

using namespace webots;

#define MOTOR_SPEED 0.1

void runOmniWheelRobotManualControlMode(Robot *robot, int timeStep) {
    const auto omniWheeledRobotName = "omniWheeledRobot";
    if (robot->getName() == omniWheeledRobotName) {
        const auto START_MESSAGE = "Manual Control Mode of omni-wheeled robot is now running...";
        std::cout << START_MESSAGE << std::endl;
        const auto wheel1 = "wheel1";
        const auto wheel2 = "wheel2";
        const auto wheel3 = "wheel3";
        auto keyboard = robot->getKeyboard(); 
        keyboard->enable(timeStep);
        auto wheel1Motor = robot->getMotor(wheel1);
        auto wheel2Motor = robot->getMotor(wheel2);
        auto wheel3Motor = robot->getMotor(wheel3);
        wheel1Motor->setPosition(INFINITY);
        wheel2Motor->setPosition(INFINITY);
        wheel3Motor->setPosition(INFINITY);
        auto wheel1MotorSpeed = 0.0;
        auto wheel2MotorSpeed = 0.0;
        auto wheel3MotorSpeed = 0.0;
        wheel1Motor->setVelocity(wheel1MotorSpeed);
        wheel2Motor->setVelocity(wheel2MotorSpeed);
        wheel3Motor->setVelocity(wheel3MotorSpeed);
        auto prevKey{-1};
        while(true) {
            if (robot->step(timeStep) == -1) {
                break;
            } 
            auto secondKey = keyboard->getKey();
            if (secondKey != prevKey && secondKey >= 0) {
                switch (secondKey) {
                    case webots::Keyboard::UP:
                        wheel1MotorSpeed = 0.0;
                        wheel2MotorSpeed = -2 * MOTOR_SPEED;
                        wheel3MotorSpeed = 2 * MOTOR_SPEED;
                        break;
                    case webots::Keyboard::DOWN:
                        wheel1MotorSpeed = 0.0;
                        wheel2MotorSpeed = 2 * MOTOR_SPEED;
                        wheel3MotorSpeed = -2 * MOTOR_SPEED;
                        break;
                    case webots::Keyboard::LEFT:
                        wheel1MotorSpeed = 2 * MOTOR_SPEED;
                        wheel2MotorSpeed = -MOTOR_SPEED;
                        wheel3MotorSpeed = -MOTOR_SPEED;
                        break;
                    case webots::Keyboard::RIGHT:
                        wheel1MotorSpeed = -2 * MOTOR_SPEED;
                        wheel2MotorSpeed = MOTOR_SPEED;
                        wheel3MotorSpeed = MOTOR_SPEED;
                        break;
                 }
                 prevKey = secondKey;
            }
            wheel1Motor->setVelocity(wheel1MotorSpeed);
            wheel2Motor->setVelocity(wheel2MotorSpeed);
            wheel3Motor->setVelocity(wheel3MotorSpeed);
        }
    }
}