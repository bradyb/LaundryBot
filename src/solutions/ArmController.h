#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"


#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cassert>
#include <queue>
#include <vector>
#include <utility>
#include <limits>

#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "math/angle_functions.hpp"

#define NUM_SERVOS 6
#define NUM_BALLS 4
#define NUM_CELLS 9
#define STATUS_CHANNEL "ARM_STATUS"
#define COMMAND_CHANNEL "ARM_COMMAND"
#define ARM_CONTROLLER_COMMAND_CHANNEL "ARM_CONTROLLER_COMMAND"

#define MOVING_PICKUP_Z .16f
#define MOVING_Z .16f
#define DROPPING_Z 0.14f
#define PICKUP_Z 0.11f

#define OPEN_GRIPPER 0
#define CLOSE_GRIPPER 2.05
#define EXTEND_JOINT3_DELTA 0.15f
#define WRIST_TILT -(M_PI / 2)
#define BASE_SERVO 0
#define JOINT1_SERVO 1
#define JOINT2_SERVO 2
#define JOINT3_SERVO 3
#define WRIST_TILT_SERVO 4
#define GRIPPER_SERVO 5

#define STOP_SPEED 0
#define SLOW_SPEED 0.02f
#define MOVE_SPEED 0.06f
#define WRIST_SPEED 0.1f
#define LOW_TORQUE 0.45f
#define HIGH_TORQUE 0.45f

#define ERROR .1
#define GRIP_ERROR .05
#define SPEED_ERROR .01

//in meters
#define D1 .11f
#define D2 .105f 
#define D3 .10f
#define D4 .078f
#define D5 .105f
#define MAX_DISTANCE 0.16f

struct Ball
{
	double x;
	double y;
};

class ArmController
{

	public:
	ArmController();
		
	void handleStatusMsg(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const dynamixel_status_list_t* msg);
                             
                              
        void goHome();
                              
        void moveTo(double x, double y, double z, double wristTilt, double gripTheta);
        void run();
        
        void setGripper(double theta);
        void setJoint(int joint, double theta);
        void goWhereNoBallHasGoneBefore();
        void wait(dynamixel_command_list_t cmd);
                              
        lcm::LCM controllerLCM;
		
	std::mutex statusMutex;
	dynamixel_status_list_t statusList;
	private:

};


#endif
