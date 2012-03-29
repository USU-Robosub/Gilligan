#include "motorController.h"
#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

using namespace std;


void printMessageMismatchError() {
}

ros::NodeHandle* n;

void MotorControllerHandler::print(string error) {
	bool init = false;
	if(!init) {
		errorOut = n->advertise<std_msgs::String>("/Error_Stream", 100);
		init = true;
	}
	std_msgs::String msg;
	msg.data = error;
	errorOut.publish(msg);
}

MotorControllerHandler::MotorControllerHandler(ros::NodeHandle* nh, const char* Port) 
		: serialPort(Port) {
	n = nh;

	awaitingResponce = false;
	currentMessage.type = NO_MESSAGE;
	gettimeofday(&lastQRCurTime, NULL);
	gettimeofday(&lastQLCurTime, NULL);
	gettimeofday(&lastQVoltTime, NULL);
	gettimeofday(&lastMotorTime, NULL);
	rightSpeed = leftSpeed = rightTargetSpeed = leftTargetSpeed = 0;
	MaxStep = 20;
	name = Port;
	try {
		serialPort.Open(BAUD, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
	} catch (...) {
		char temp[1000];
		sprintf(temp, "%s error: Failed during initialization\n", name.c_str());
		print(string(temp));
	}
	bufIndex = 0;
}

void MotorControllerHandler::sendMessage(Message m) {
	currentMessage = m;
	transmit();
}

int filter(int speed) {
	if(speed >= 256)
		speed = 255;
	if(speed <= -256)
		speed = -255;
	return speed;
}

void MotorControllerHandler::setMotorSpeed(int right, int left) {
	rightTargetSpeed = filter(right);
	leftTargetSpeed = filter(left);
	printf("setting target speeds to %d %d\n", rightTargetSpeed, leftTargetSpeed);
}

Message createMessageFromSpeed(int rightSpeed, int leftSpeed) {
	printf("creating message for %d %d", rightSpeed, leftSpeed);
	Message msg;
	msg.type = MOTOR_TYPE;

	if(leftSpeed > 0) {
		msg.DataC[0] = leftSpeed;
		msg.DataC[1] = 0;
	} else {
		int rev = -leftSpeed;
		msg.DataC[0] = 0;
		msg.DataC[1] = rev;
	}
	if(rightSpeed > 0) {
		msg.DataC[2] = rightSpeed;
		msg.DataC[3] = 0;
	} else {
		int rev = -rightSpeed;
		msg.DataC[2] = 0;
		msg.DataC[3] = rev;
	}

	return msg;
}

void MotorControllerHandler::transmit() {
	if(currentMessage.type == NO_MESSAGE) 
		return;

	gettimeofday(&lastSendTime, NULL);
	printf("sending message %c %d %d %d %d\n", currentMessage.type,
			currentMessage.DataC[0],
			currentMessage.DataC[1],
			currentMessage.DataC[2],
			currentMessage.DataC[3]);
	awaitingResponce = true;

	if(!serialPort.IsOpen()) {
		printf("%s: port not open attempting to re-open port\n", name.c_str());
		try {
			serialPort.Open(BAUD, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
		} catch (...) {
			char temp[1000];
			sprintf(temp, "%s error: Unable to open port\n", name.c_str());
			print(string(temp));
			return;
		}
	}
	try {
		serialPort.WriteByte('S');
		serialPort.WriteByte(currentMessage.type);
		for(int i = 0; i < 4; i++) {
			serialPort.WriteByte(currentMessage.DataC[i]);
		}
		serialPort.WriteByte('E');
	} catch (...) {
			char temp[1000];
			sprintf(temp, "%s error: Unable to send message\n", name.c_str());
			print(string(temp));
			return;
	}
	awaitingResponce = true;
}

void MotorControllerHandler::processResponce() {
	if(buffer[0] != 'S' || buffer[6] != 'E') {
		//Misaligned data? throw out bytes until you get it to align correctly
		for(int i = 0; i < 6; i++) {
			buffer[i] = buffer[i+1];
			bufIndex--;
			return;
		}
	}
	bufIndex = 0;
	Message responce;
	responce.type = buffer[1];
	printf("got responce of type %c\n", responce.type);
	for(int i = 0; i < 4; i++) {
		responce.DataC[i] = buffer[i+2];
	}
	
	//printf("got responce %c %c %x %x %x %x %c\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
	switch (responce.type) {
		case ERROR_TYPE:
			char temp[1000];
			sprintf(temp, "%s error from controller: %c%c%c%c\n", name.c_str(), buffer[2], buffer[3], buffer[4], buffer[5]);
			print(string(temp));
			awaitingResponce = false;
			break;
		case MOTOR_RESPONCE_TYPE:
			if(currentMessage.type != MOTOR_TYPE) {
				printMessageMismatchError();
				break;
			}
			if(responce.DataC[0])
				leftSpeed = responce.DataC[0];
			else
				leftSpeed = -responce.DataC[1];

			if(responce.DataC[2])
				rightSpeed = responce.DataC[2];
			else
				rightSpeed = -responce.DataC[3];
			printf("new speeds right=%d(%d) left=%d(%d)\n", rightSpeed, rightTargetSpeed, leftSpeed, leftTargetSpeed);
			currentMessage.type = NO_MESSAGE;
			awaitingResponce = false;
			break;
		case CURRENT_RESPONCE_TYPE:
			if(currentMessage.type != CURRENT_TYPE) {
				printMessageMismatchError();
				break;
			}

			if(currentMessage.DataC[0] == 'L')
				LeftCurrent = responce.DataF;
			else
				RightCurrent = responce.DataF;

			currentMessage.type = NO_MESSAGE;
			awaitingResponce = false;
			break;
		case VOLTAGE_RESPONCE_TYPE:
			if(currentMessage.type != VOLTAGE_TYPE) {
				printMessageMismatchError();
				break;
			}
			Voltage = responce.DataF;
			currentMessage.type = NO_MESSAGE;
			awaitingResponce = false;
			break;
		default:
			printf("Unrecognized responce type: %c\n", responce.type);
	}
}

void MotorControllerHandler::recieve() {
	if(serialPort.IsOpen() && awaitingResponce) {
		try {
			while(serialPort.IsDataAvailable()) {
				unsigned char data = serialPort.ReadByte();
				//printf("recieved byte \'%c\'\n", data);
				while(bufIndex == 7) {
					printf("You are not clearing bufIndex\n");
					processResponce();
				}
				buffer[bufIndex++] = data;
				if(bufIndex == 7) {
					processResponce();
				}
			}
		} catch (...) {
			char temp[1000];
			sprintf(temp, "%s error: While attempting to read data\n", name.c_str());
			print(string(temp));
			return;
		}
	}
}

int getMilliSecsBetween(timeval& start, timeval& end) {
	int millis = (end.tv_sec - start.tv_sec) * 1000;
	millis += (end.tv_usec - start.tv_usec) / 1000;
	return millis;
}

bool MotorControllerHandler::TransmitTimeout() {
	timeval curTime;
	gettimeofday(&curTime, NULL);
	int elsaped = getMilliSecsBetween(lastSendTime, curTime);
	if(elsaped > RESEND_TIMEOUT)
		return true;
	return false;
}

void MotorControllerHandler::CheckQuery() {
	if(awaitingResponce)
		return;
	timeval curtime;
	gettimeofday(&curtime, NULL);
	int elsaped = getMilliSecsBetween(lastQRCurTime, curtime);
	if(elsaped > QUERY_PERIOD) {
		Message query;
		query.type = CURRENT_TYPE;
		query.DataC[0] = 'R';
		sendMessage(query);
		gettimeofday(&lastQRCurTime, NULL);
		return;
	}
	elsaped = getMilliSecsBetween(lastQLCurTime, curtime);
	if(elsaped > QUERY_PERIOD) {
		Message query;
		query.type = CURRENT_TYPE;
		query.DataC[0] = 'L';
		sendMessage(query);
		gettimeofday(&lastQLCurTime, NULL);
		return;
	}
	elsaped = getMilliSecsBetween(lastQVoltTime, curtime);
	if(elsaped > QUERY_PERIOD) {
		Message query;
		query.type = VOLTAGE_TYPE;
		sendMessage(query);
		gettimeofday(&lastQVoltTime, NULL);
		return;
	}
}

void MotorControllerHandler::CheckMotor() {
	if(awaitingResponce)
		return;
	timeval curTime;
	gettimeofday(&curTime, NULL);
	int elsaped = getMilliSecsBetween(lastMotorTime, curTime);
	if(elsaped < MOTOR_PERIOD)
		return;

	bool needsSent = false;
	int leftSetSpeed = leftSpeed;
	int rightSetSpeed = rightSpeed;
	if(leftSpeed != leftTargetSpeed) {
		int diff = leftTargetSpeed - leftSpeed;
		if(diff > MaxStep)
			diff = MaxStep;
		if(diff < -MaxStep)
			diff = -MaxStep;
		leftSetSpeed += diff;
		needsSent=true;
	}
	if(rightSpeed != rightTargetSpeed) {
		int diff = rightTargetSpeed - rightSpeed;
		if(diff > MaxStep)
			diff = MaxStep;
		if(diff < -MaxStep)
			diff = -MaxStep;
		rightSetSpeed += diff;
		needsSent=true;
	}
	if(needsSent) {
		sendMessage(createMessageFromSpeed(rightSetSpeed, leftSetSpeed));
		gettimeofday(&lastMotorTime, NULL);
	}
}

void MotorControllerHandler::spinOnce() {
	recieve();
	CheckMotor();
	CheckQuery();
	if(awaitingResponce && TransmitTimeout()) {
		transmit();
	}
} 
