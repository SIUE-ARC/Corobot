#include <ros/ros.h>

#include <phidgets_api/encoder.h>
#include <phidgets_api/motor.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_ENCODER 0
#define RIGHT_ENCODER 1
#define RAD_PER_TICK 0.0019634954 //2 * PI / (64 * 50)
#define K_P 0.0
#define K_I 1.0
#define K_D 0.5

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

class CorobotBase : public hardware_interface::RobotHW {
public:
	CorobotBase() {
		ROS_INFO("Waiting for encoders...");
		encoders.open(-1);
		encoders.waitForAttachment(0);

		encoders.setEnabled(LEFT_ENCODER, true);
		encoders.setEnabled(RIGHT_ENCODER, true);

		ROS_INFO("Waiting for motors...");
		motors.open(-1);
		motors.waitForAttachment(0);

		motors.setAcceleration(LEFT_MOTOR, 50.0);
		motors.setAcceleration(RIGHT_MOTOR, 50.0);
		motors.setVelocity(LEFT_MOTOR, 0.0);
		motors.setVelocity(RIGHT_MOTOR, 0.0);

		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle_left("front_left_wheel_joint", &pos[0], &vel[0], &eff[0]);
		hardware_interface::JointStateHandle state_handle_back_left("back_left_wheel_joint", &pos[0], &vel[0], &eff[0]);
		stateInterface.registerHandle(state_handle_left);
		stateInterface.registerHandle(state_handle_back_left);

		hardware_interface::JointStateHandle state_handle_right("front_right_wheel_joint", &pos[1], &vel[1], &eff[1]);
		hardware_interface::JointStateHandle state_handle_back_right("back_right_wheel_joint", &pos[1], &vel[1], &eff[1]);
		stateInterface.registerHandle(state_handle_right);
		stateInterface.registerHandle(state_handle_back_right);

		registerInterface(&stateInterface);


		// Register the command interface
		hardware_interface::JointHandle cmd_handle_left(stateInterface.getHandle("front_left_wheel_joint"), &cmd[0]);
		hardware_interface::JointHandle cmd_handle_back_left(stateInterface.getHandle("back_left_wheel_joint"), &cmd[0]);
		cmdInterface.registerHandle(cmd_handle_left);
		cmdInterface.registerHandle(cmd_handle_back_left);

		hardware_interface::JointHandle cmd_handle_right(stateInterface.getHandle("front_right_wheel_joint"), &cmd[1]);
		hardware_interface::JointHandle cmd_handle_back_right(stateInterface.getHandle("back_right_wheel_joint"), &cmd[1]);
		cmdInterface.registerHandle(cmd_handle_right);
		cmdInterface.registerHandle(cmd_handle_back_right);

		registerInterface(&cmdInterface);
	}

	~CorobotBase() {
		encoders.close();

		motors.setVelocity(LEFT_MOTOR, 0.0);
		motors.setVelocity(RIGHT_MOTOR, 0.0);
		motors.close();
	}

	double getBatteryVoltage() {
		return motors.getSupplyVoltage();
	}

	void read(ros::Duration period) {
		double newPos[2];
		newPos[0] = -1 * encoders.getPosition(LEFT_ENCODER) * RAD_PER_TICK;
		newPos[1] = encoders.getPosition(RIGHT_ENCODER) * RAD_PER_TICK;

		// Velocity calcs
		vel[0] = (newPos[0] - pos[0]) / period.toSec();
		vel[1] = (newPos[1] - pos[1]) / period.toSec();

		pos[0] = newPos[0];
		pos[1] = newPos[1];
	}

	void write(ros::Duration period) {
		double err[2];
		err[0] = cmd[0] - vel[0];
		err[1] = cmd[1] - vel[1];

		errI[0] = errI[0] + err[0];
		errI[1] = errI[1] + err[1];

		errD[0] = err[0] - errD[0];
		errD[1] = err[1] - errD[1];

		thrt[0] = err[0] * K_P + errI[0] * K_I + errD[0] * K_D;
		thrt[1] = err[1] * K_P + errI[1] * K_I + errD[1] * K_D;

		ROS_INFO("dt:%f    p:(%f, %f) v:(%f, %f) t:(%f, %f)    e:(%f, %f) ei:(%f, %f) ed(%f, %f)"
                        , period.toSec(), pos[0], pos[1], vel[0], vel[1], thrt[0], thrt[1], err[0], err[1], errI[0], errI[1], errD[0], errD[1]);

		if(abs(cmd[0]) < 0.1) {
			thrt[0] = 0;
			errI[0] = 0;
		}

		if(abs(cmd[1]) < 0.1) {
			thrt[1] = 0;
			errI[1] = 0;
		}

		thrt[0] = min(thrt[0], 99.0);
		thrt[1] = min(thrt[1], 99.0);

		thrt[0] = max(thrt[0], -99.0);
		thrt[1] = max(thrt[1], -99.0);

		// Run pid using cmd as setpoint and vel as value
		motors.setVelocity(LEFT_MOTOR, thrt[0]);
		motors.setVelocity(RIGHT_MOTOR, thrt[1]);

		errD[0] = err[0];
		errD[1] = err[1];
	}

private:
	phidgets::Encoder encoders;
	phidgets::MotorController motors;

	hardware_interface::JointStateInterface stateInterface;
	hardware_interface::VelocityJointInterface cmdInterface;
	
	double cmd[2] = {0, 0};
	double pos[2] = {0, 0};
	double vel[2];
	double eff[2];

	double thrt[2] = {0, 0};
	double errI[2];
	double errD[2];
};

int main(int argc, char **argv) {
	// Initialize ROS
	ros::init(argc, argv, "corobot_driver");
	ros::NodeHandle n;

	CorobotBase corobot;
	controller_manager::ControllerManager controller(&corobot, n);
	
	ros::Rate rate(25);
	ros::Time then = ros::Time::now();
        ros::AsyncSpinner spinner(1);

	spinner.start();
	while(ros::ok()) {
		ros::Time now = ros::Time::now();

		// Robot control
		corobot.read(now - then);
		controller.update(now, now - then);
		corobot.write(now - then);

		// hand back to ROS
		then = now;
		rate.sleep();
	}
	spinner.stop();

	return 0;
}

