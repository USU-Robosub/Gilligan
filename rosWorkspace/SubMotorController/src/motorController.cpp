#include "motorController.h"
#include "time.h"

void printMessageMismatchError() {
}

MotorControllerHandler::MotorControllerHandler(const char* Port) 
		: serialPort(Port) {

	awaitingResponce = false;
	currentMessage.type = NO_MESSAGE;
	nextMessage.type = NO_MESSAGE;
	gettimeofday(&lastQueryTime, NULL);
	name = Port;
	try {
		serialPort.Open(BAUD, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
	} catch (...) {
		printf("%s error: Failed during initialization\n", name.c_str());
	}
	bufIndex = 0;
}

void MotorControllerHandler::sendMessage(Message m) {
	if(awaitingResponce) {
		nextMessage = m;
		return;
	}

	currentMessage = m;
	transmit();
}

void MotorControllerHandler::transmit() {
	if(currentMessage.type == NO_MESSAGE) 
		return;

	gettimeofday(&lastSendTime, NULL);

	if(!serialPort.IsOpen()) {
		printf("%s: port not open attempting to re-open port\n", name.c_str());
		try {
			serialPort.Open(BAUD, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
		} catch (...) {
			printf("%s: Unable to open port\n", name.c_str());
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
		printf("%s error: Unable to send message\n", name.c_str());
	}
	//printf("sending message  %c %c %x %x %x %x %c\n", 'S', currentMessage.type, currentMessage.DataC[0],
	//										currentMessage.DataC[1],
	//										currentMessage.DataC[2],
	//										currentMessage.DataC[3],
	//										'E');
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
	for(int i = 0; i < 4; i++) {
		responce.DataC[i] = buffer[i+2];
	}
	
	//printf("got responce %c %c %x %x %x %x %c\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
	switch (responce.type) {
		case ERROR_TYPE:
			printf("%s error: %c%c%c%c\n", name.c_str(), buffer[2], buffer[3], buffer[4], buffer[5]);
			currentMessage = nextMessage;
			nextMessage.type = NO_MESSAGE;
			awaitingResponce = false;
			break;
		case MOTOR_RESPONCE_TYPE:
			if(currentMessage.type != MOTOR_TYPE) {
				printMessageMismatchError();
				break;
			}
			currentMessage = nextMessage;
			nextMessage.type = NO_MESSAGE;
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

			currentMessage = nextMessage;
			nextMessage.type = NO_MESSAGE;
			awaitingResponce = false;
			break;
		case VOLTAGE_RESPONCE_TYPE:
			if(currentMessage.type != VOLTAGE_TYPE) {
				printMessageMismatchError();
				break;
			}
			Voltage = responce.DataF;
			currentMessage = nextMessage;
			nextMessage.type = NO_MESSAGE;
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
			printf("%s error: while attempting to read data\n", name.c_str());
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

void MotorControllerHandler::CheckPullTimes() {
	timeval curTime;
	gettimeofday(&curTime, NULL);
	int elsaped = getMilliSecsBetween(lastQueryTime, curTime);
	if(elsaped > QUERY_PERIOD) {
		Message query;
		query.type = VOLTAGE_TYPE;
		sendMessage(query);
		gettimeofday(&lastQueryTime, NULL);
	}
}

void MotorControllerHandler::spinOnce() {
	recieve();
	CheckPullTimes();
	if(awaitingResponce && TransmitTimeout()) {
		//maybe we are out of sync?
		transmit();
	}
} 
