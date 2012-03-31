#include "sub.h"
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "SubMotorController/MotorMessage.h"
#include "motor.h"
#include <stdlib.h>

Submarine sub;

struct MotorMessage {
	MotorMessage(int data) {
		speed = data & 0xFF;
		if(data & 0x100) 
			speed = -speed;
		rightMotor = data & 0x200;
		leftMotor = data & 0x400;
		speed /= 256.0;
	}
	float speed;
	bool rightMotor;
	bool leftMotor;
};

float setPower(int level) {
	level = -level;
	if(abs(level) < 55)
		return 0.0;
	bool reversed = false;
	if(level < 0)
		return (level + 55)/200.0;
	else
		return (level - 55)/200.0;
}

void mMotorCallback(const SubMotorController::MotorMessage::ConstPtr& msg) {
	if(RIGHT_DRIVE_MOTOR_BIT & msg->mask) 
		sub.motor.mainR = setPower(msg->Right);
	if(LEFT_DRIVE_MOTOR_BIT & msg->mask) 
		sub.motor.mainL = setPower(msg->Left);
	if(FRONT_DEPTH_MOTOR_BIT & msg->mask) 
		sub.motor.depthF = setPower(msg->FrontDepth);
	if(REAR_DEPTH_MOTOR_BIT & msg->mask) 
		sub.motor.depthR = setPower(msg->RearDepth);
	if(FRONT_TURN_MOTOR_BIT & msg->mask) 
		sub.motor.turnF = setPower(msg->FrontTurn);
	if(REAR_TURN_MOTOR_BIT & msg->mask) 
		sub.motor.turnR = setPower(msg->RearTurn);
}

void mDriveCallback(const std_msgs::Int16::ConstPtr& msg) {
	MotorMessage message(msg->data);
	if(message.rightMotor)
		sub.motor.mainR = message.speed;
	if(message.leftMotor)
		sub.motor.mainL = message.speed;
}

void mDepthCallback(const std_msgs::Int16::ConstPtr& msg) {
	MotorMessage message(msg->data);
	if(message.rightMotor)
		sub.motor.depthF = message.speed;
	if(message.leftMotor)
		sub.motor.depthR = message.speed;
}

void mTurnCallback(const std_msgs::Int16::ConstPtr& msg) {
	MotorMessage message(msg->data);
	if(message.rightMotor)
		sub.motor.turnF = message.speed;
	if(message.leftMotor)
		sub.motor.turnR = message.speed;
}

void UpdateDepth(float delta) {
	sub.velocity.depth -= (double)(sub.motor.depthR + sub.motor.depthF) * sub.motorPowerConst*delta;
	if(sub.position.depth < -1)
		sub.velocity.depth += bouyancy * sub.motorPowerConst*delta;
	else if(sub.position.depth < 0)
		sub.velocity.depth += -sub.position.depth * weight * sub.motorPowerConst*delta;
	else
		sub.velocity.depth += weight * sub.motorPowerConst*delta;

	sub.velocity.depth += sub.velocity.depth * sub.frictionConst * delta;
	sub.position.depth += sub.velocity.depth * delta;
}

void UpdatePosition(float delta) {
	sub.velocity.x += sin(sub.rotation.yaw) * (sub.motor.mainR + sub.motor.mainL) * sub.motorPowerConst * delta;
	sub.velocity.x += sin(sub.rotation.yaw + M_PI/2) * (sub.motor.turnR + -sub.motor.turnF) * sub.motorPowerConst * delta;
	sub.velocity.x += sub.velocity.x * sub.frictionConst * delta;
	sub.position.x += sub.velocity.x * delta;
	
	sub.velocity.y += cos(sub.rotation.yaw) * (sub.motor.mainR + sub.motor.mainL) * sub.motorPowerConst * delta;
	sub.velocity.x += cos(sub.rotation.yaw + M_PI/2) * (sub.motor.turnR + -sub.motor.turnF) * sub.motorPowerConst * delta;
	sub.velocity.y += sub.velocity.y * sub.frictionConst * delta;
	sub.position.y += sub.velocity.y * delta;
}

void UpdateRotation(float delta) {
	sub.rotationalVelocity.yaw += (sub.motor.mainL - sub.motor.mainR) * sub.mainRotationalPowerConst * delta;
	sub.rotationalVelocity.yaw -= (sub.motor.turnF + sub.motor.turnR) * sub.turnRotationalPowerConst * delta;
	sub.rotationalVelocity.yaw += sub.rotationalVelocity.yaw * sub.rotationalFrictionConst * delta;
	printf("rotation = %f\n", sub.rotation.yaw);
	sub.rotation.yaw += sub.rotationalVelocity.yaw * delta;
	if(sub.rotation.yaw > 2*M_PI)
		sub.rotation.yaw -= 2*M_PI;
	if(sub.rotation.yaw < 0)
		sub.rotation.yaw += 2*M_PI;
}

void Update(float delta) {
	UpdateDepth(delta);
	UpdatePosition(delta);
	UpdateRotation(delta);
}

void PublishDepth(ros::Publisher pub) {
	std_msgs::Float32 msg;
	msg.data = sub.position.depth;
	pub.publish(msg);
}

void PublishPosition() {
	static ros::Publisher pubX;
	static ros::Publisher pubY;
	static ros::Publisher pubZ;
	static ros::Publisher pubYaw;
	static ros::NodeHandle n;
	static bool firstTime = true;

	if(firstTime) {
		pubX = n.advertise<std_msgs::Float32>("/sim/pos/x", 10);
		pubY = n.advertise<std_msgs::Float32>("/sim/pos/y", 10);
		pubZ = n.advertise<std_msgs::Float32>("/sim/pos/z", 10);
		pubYaw = n.advertise<std_msgs::Float32>("/sim/rotation/yaw", 10);
	}

	std_msgs::Float32 msgX;
	msgX.data = sub.position.x;
	pubX.publish(msgX);

	std_msgs::Float32 msgY;
	msgY.data = sub.position.y;
	pubY.publish(msgY);

	std_msgs::Float32 msgZ;
	msgZ.data = sub.position.depth;
	pubZ.publish(msgZ);

	std_msgs::Float32 msgYaw;
	msgYaw.data = sub.rotation.yaw;
	pubYaw.publish(msgYaw);
}

void init() {
	sub.position.x = 0;
	sub.position.y = 0;
	sub.position.depth = 0;
	sub.velocity.x = 0;
	sub.velocity.y = 0;
	sub.velocity.depth = 0;
	sub.rotation.roll = 0;
	sub.rotation.pitch = 0;
	sub.rotation.yaw = 0;
	sub.rotationalVelocity.roll = 0;
	sub.rotationalVelocity.pitch = 0;
	sub.rotationalVelocity.yaw = 0;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "subSim");
	ros::NodeHandle n;
	ros::Publisher depthPub = n.advertise<std_msgs::Float32>("/sim/sensor/depth", 1);

	ros::Subscriber mDriveSub = n.subscribe("/Motor_Driver_Drive", 1, mDriveCallback);
	ros::Subscriber mDepthSub = n.subscribe("/Motor_Driver_Depth", 1, mDepthCallback);
	ros::Subscriber mTurnSub = n.subscribe("/Motor_Driver_Turn", 1, mTurnCallback);
	ros::Subscriber mMotorSub = n.subscribe("/motorControl", 1, mMotorCallback);
	
	ros::Rate loop_rate(30);

	while(ros::ok()) {
		Update(1/30.0);
		PublishDepth(depthPub);
		PublishPosition();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
