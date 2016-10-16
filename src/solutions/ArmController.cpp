#include "ArmController.h"

using namespace std;

ArmController::ArmController()
{
	statusList.statuses.resize(NUM_SERVOS);

	std::thread statusThread([this] () {
		lcm::LCM lcm;
		lcm.subscribe(STATUS_CHANNEL, &ArmController::handleStatusMsg, this);
		while(lcm.handle() == 0);
	});
	statusThread.detach();
	
}

void ArmController::run()
{
	//moveTo(.13,.1,.12, -1.171, OPEN_GRIPPER);
	moveTo(.18,0,.12, -1.171, OPEN_GRIPPER);

	statusMutex.lock();
	double joint1Position = statusList.statuses[JOINT1_SERVO].position_radians;	
	cout << "Joint 1 position: " << joint1Position << std::endl;
	statusMutex.unlock();
	
	setJoint(JOINT3_SERVO, .9);

	setJoint(JOINT1_SERVO, joint1Position + 0.17417); // .87);
	setJoint(JOINT3_SERVO, 1.6);
	setGripper(CLOSE_GRIPPER);
	goHome();
	
	cout << "finished"<<endl;
	
}

void ArmController::handleStatusMsg(const lcm::ReceiveBuffer* rbuf,
                              const std::string& chan,
                              const dynamixel_status_list_t* msg)
{
	statusMutex.lock();
	statusList.len = msg->len;
	statusList.statuses.assign(msg->statuses.begin(), msg->statuses.end());
	statusMutex.unlock();
}

void ArmController::goHome()
{
	dynamixel_command_list_t cmd;
	
	cmd.len = NUM_SERVOS;
	cmd.commands.resize(NUM_SERVOS);
	
	for(unsigned int i = 0; i < NUM_SERVOS; ++i)
	{
		cmd.commands[i].utime = utime_now();
		cmd.commands[i].position_radians = 0;
		cmd.commands[i].speed = MOVE_SPEED;
		cmd.commands[i].max_torque = HIGH_TORQUE;
	}
	
	cmd.commands[GRIPPER_SERVO].position_radians = CLOSE_GRIPPER;
	controllerLCM.publish(COMMAND_CHANNEL, &cmd);
	wait(cmd);
}

void ArmController::setJoint(int joint, double theta)
{
	dynamixel_command_list_t cmd;
	cmd.len = NUM_SERVOS;
	cmd.commands.resize(NUM_SERVOS);

	for(unsigned int i = 0; i < NUM_SERVOS-1; ++i)
	{
		lock_guard<mutex> statusMutexLock(statusMutex);

		cmd.commands[i].utime = utime_now();
		cmd.commands[i].position_radians = statusList.statuses[i].position_radians;
		cmd.commands[i].speed = STOP_SPEED;
		cmd.commands[i].max_torque = HIGH_TORQUE;
	}
	
	cmd.commands[joint].utime = utime_now();
	cmd.commands[joint].position_radians = theta;
	cmd.commands[joint].speed = MOVE_SPEED;
	cmd.commands[joint].max_torque = HIGH_TORQUE;
	
	controllerLCM.publish(COMMAND_CHANNEL, &cmd);
	
	bool complete = false;
	while(!complete)
	{
		statusMutex.lock();
		complete = fabs(eecs467::angle_diff(statusList.statuses[joint].position_radians, theta )) < ERROR;
		statusMutex.unlock();
	}
	cout << "successful move\n";

}

void ArmController::setGripper(double theta)
{
	dynamixel_command_list_t cmd;
	cmd.len = NUM_SERVOS;
	cmd.commands.resize(NUM_SERVOS);

	for(unsigned int i = 0; i < NUM_SERVOS-1; ++i)
	{
		lock_guard<mutex> statusMutexLock(statusMutex);
		cmd.commands[i].utime = utime_now();
		cmd.commands[i].position_radians = statusList.statuses[i].position_radians;
		cmd.commands[i].speed = STOP_SPEED;
		cmd.commands[i].max_torque = HIGH_TORQUE;
	}
	
	cmd.commands[GRIPPER_SERVO].utime = utime_now();
	cmd.commands[GRIPPER_SERVO].position_radians = theta;
	cmd.commands[GRIPPER_SERVO].speed = MOVE_SPEED;
	cmd.commands[GRIPPER_SERVO].max_torque = HIGH_TORQUE;
	
	controllerLCM.publish(COMMAND_CHANNEL, &cmd);

	bool complete = false;
	while(!complete)
	{
		statusMutex.lock();
		complete = fabs(eecs467::angle_diff(statusList.statuses[GRIPPER_SERVO].position_radians, theta )) < GRIP_ERROR;
		statusMutex.unlock();
	}
	cout << "successful move\n";
}

void ArmController::goWhereNoBallHasGoneBefore()
{
	dynamixel_command_list_t cmd;
	dynamixel_status_list_t curStatuses;
	cmd.len = curStatuses.len = NUM_SERVOS;
	cmd.commands.resize(NUM_SERVOS);
	curStatuses.statuses.resize(NUM_SERVOS);	
	statusMutex.lock();
	curStatuses.statuses = statusList.statuses;
	statusMutex.unlock();
	double theta = curStatuses.statuses[JOINT3_SERVO].position_radians;
	std::cout << "Going where no man has gone before\n";
	if (theta < 0)
	{
		theta += EXTEND_JOINT3_DELTA;
	}
	else
	{
		theta -= EXTEND_JOINT3_DELTA;
	}
	
	for(unsigned int i = 0; i < NUM_SERVOS; ++i)
	{
		cmd.commands[i].utime = utime_now();
		cmd.commands[i].speed = STOP_SPEED;
		cmd.commands[i].position_radians = curStatuses.statuses[i].position_radians;
		cmd.commands[i].max_torque = HIGH_TORQUE;
	}
	
	cmd.commands[JOINT3_SERVO].utime = utime_now();
	cmd.commands[JOINT3_SERVO].position_radians = theta;
	cmd.commands[JOINT3_SERVO].speed = MOVE_SPEED;
	cmd.commands[JOINT3_SERVO].max_torque = HIGH_TORQUE;
	
	controllerLCM.publish(COMMAND_CHANNEL, &cmd);
	wait(cmd);
}

void ArmController::moveTo(double x, double y, double z, double wristTilt, double gripTheta)
{
	dynamixel_command_list_t cmd;
	cmd.len = NUM_SERVOS;
	cmd.commands.resize(NUM_SERVOS);
	
	double h = z;
	//assert(z > D5);
	double R = sqrt(x*x + y*y);
	double alpha = atan2(D4+h-D1, R);
	double M =sqrt(R*R + sq(D4+h-D1));
	double cosBeta = (-(D3*D3) + (D2*D2) + (M*M))/(2*D2*M);
	double cosGamma = (-(M*M)+(D2*D2)+(D3*D3))/(2*D2*D3);
	double beta = acos(cosBeta);
	double gamma = acos(cosGamma);
	assert(-1 <= cosBeta and cosBeta <= 1);
	assert(-1 <= cosGamma and cosGamma <= 1);
	
	double thetas[NUM_SERVOS];
	thetas[BASE_SERVO] = atan2(y, x);
	thetas[JOINT1_SERVO] = M_PI/2 - beta - alpha;
	thetas[JOINT2_SERVO] = (M_PI - gamma);
	thetas[JOINT3_SERVO] = M_PI - thetas[JOINT1_SERVO] - thetas[JOINT2_SERVO];
	thetas[WRIST_TILT_SERVO] = wristTilt;
	thetas[GRIPPER_SERVO] = gripTheta;
	
	cmd.commands[BASE_SERVO].utime = utime_now();
	cmd.commands[BASE_SERVO].position_radians = thetas[BASE_SERVO];
	cmd.commands[BASE_SERVO].speed = MOVE_SPEED;
	cmd.commands[BASE_SERVO].max_torque = HIGH_TORQUE;
	
	cmd.commands[JOINT1_SERVO].utime = utime_now();
	cmd.commands[JOINT1_SERVO].position_radians = thetas[JOINT1_SERVO];
	cmd.commands[JOINT1_SERVO].speed = MOVE_SPEED;
	cmd.commands[JOINT1_SERVO].max_torque = HIGH_TORQUE;
	
	cmd.commands[JOINT2_SERVO].utime = utime_now();
	cmd.commands[JOINT2_SERVO].position_radians = thetas[JOINT2_SERVO];
	cmd.commands[JOINT2_SERVO].speed = MOVE_SPEED;
	cmd.commands[JOINT2_SERVO].max_torque = HIGH_TORQUE;
	
	cmd.commands[JOINT3_SERVO].utime = utime_now();
	cmd.commands[JOINT3_SERVO].position_radians = thetas[JOINT3_SERVO];
	cmd.commands[JOINT3_SERVO].speed = MOVE_SPEED;
	cmd.commands[JOINT3_SERVO].max_torque = HIGH_TORQUE;
	
	cmd.commands[WRIST_TILT_SERVO].utime = utime_now();
	cmd.commands[WRIST_TILT_SERVO].position_radians = wristTilt;
	cmd.commands[WRIST_TILT_SERVO].speed = WRIST_SPEED;
	cmd.commands[WRIST_TILT_SERVO].max_torque = HIGH_TORQUE;
	
	cmd.commands[GRIPPER_SERVO].utime = utime_now();
	cmd.commands[GRIPPER_SERVO].position_radians = gripTheta;
	cmd.commands[GRIPPER_SERVO].speed = MOVE_SPEED;
	cmd.commands[GRIPPER_SERVO].max_torque = HIGH_TORQUE;
	
	controllerLCM.publish(COMMAND_CHANNEL, &cmd);
	
	wait(cmd);
}


void ArmController::wait(dynamixel_command_list_t cmd)
{
	bool complete = false;
	while(!complete)
	{
		statusMutex.lock();
		complete = true;
		for(unsigned int i = 0; i < statusList.statuses.size()-1; ++i)
		{
			complete = complete and 
						(fabs(eecs467::angle_diff(statusList.statuses[i].position_radians, 
						cmd.commands[i].position_radians)) < ERROR);
		}
		complete = complete and 
					(statusList.statuses[5].speed < SPEED_ERROR);
					
		statusMutex.unlock();
	}
	cout <<"successful move\n";
}

int main(int argc, char **argv)
{
	ArmController controllerObject;
 	controllerObject.run();
 	return 0;
}
