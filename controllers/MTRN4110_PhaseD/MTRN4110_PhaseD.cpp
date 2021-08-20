// File:          MTRN4110_PhaseD.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <webots/Robot.hpp>
#include <webots/Keyboard.hpp>
#include "MTRN4110_PhaseA.h"
#include "MTRN4110_PhaseB.h"
#include "MTRN4110_Manual_Control_Mode.h"
#include "MTRN4110_Wall_Following_Mode.h"
#include "Omniwheels.h"
#include "MTRN4110_BangBang.h"

constexpr int TIME_STEP_VALUE{64};
constexpr char ONE_KEY{'1'};
constexpr char TWO_KEY{'2'};
constexpr char THREE_KEY{'3'};
constexpr char FOUR_KEY{'4'};
constexpr char FIVE_KEY{'5'};
constexpr char SIX_KEY{'6'};
constexpr int PROGRAM_STOP{-1};

const auto PATH_PLAN_FILE_NAME = "../../PathPlan.txt"; 

using namespace webots;

int main(int argc, char **argv) {
    auto startCommands(
        "Please select which mode you would like to run\n"
        "[1] run Phase A\n"
        "[2] run Phase B\n"
        "[3] run Manual Control Mode with epuck\n"
        "[4] run Wall Following Mode with epuck\n"
        "[5] run Manual Control Mode with omni-wheeled robot\n"
        "[6] run Bang Bang Mode\n"
    );
    std::cout << startCommands << std::endl;
    auto robot = new Robot();
    //std::cout << robot->scale() << std::endl;
    auto keyboard = robot->getKeyboard(); 
    keyboard->enable(TIME_STEP_VALUE);
    auto prevKey{-1};
    auto flag1 = false;
    auto flag = ' ';
    
    while (robot->step(TIME_STEP_VALUE) != PROGRAM_STOP) {
        //initialise key value
        
        //get the key which is pressed
        auto key = keyboard->getKey();
        
        if (key != prevKey) {
            //run chosen mode based on user input
            if (key == ONE_KEY) {
                runPhaseA(robot, false, " ");
            } else if (key == TWO_KEY) {
                runPhaseB(robot, 0,0, true, false, PATH_PLAN_FILE_NAME, flag, flag1);
            } else if (key == THREE_KEY) {
                runEpuckManualControlMode(robot, TIME_STEP_VALUE);
            } else if (key == FOUR_KEY) {
                runWallFollowingMode(robot, TIME_STEP_VALUE);
            } else if (key == FIVE_KEY) {
                runOmniWheelRobotManualControlMode(robot, 4);
            } else if (key == SIX_KEY) {
                runBangBangMode(robot);
            }
        }
        prevKey = key;
    }
    
    delete robot;
    return 0;
}
