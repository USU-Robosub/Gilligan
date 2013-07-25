#include <sys/time.h>
#include <string>
#include <ros/ros.h>
#include <SerialPort.h>

using namespace std;

const char NO_MESSAGE = ' ';
const char MOTOR_TYPE = 'D';
const char MOTOR_RESPONSE_TYPE = 'd';
const char CURRENT_TYPE = 'C';
const char CURRENT_RESPONSE_TYPE = 'c';
const char VOLTAGE_TYPE = 'V';
const char VOLTAGE_RESPONSE_TYPE = 'v';
const char ERROR_TYPE = 'e';

const SerialPort::BaudRate BAUD = SerialPort::BAUD_115200;
//const SerialPort::BaudRate BAUD = SerialPort::BAUD_19200;

const int RESEND_TIMEOUT = 1000;
const int QUERY_PERIOD = 1000;
const int CURRENT_PERIOD = 100;
const int MOTOR_PERIOD = 70;

struct Message {
	char type;
	union {
		unsigned char DataC[4];
		int DataI;
		float DataF;
	};
};

const int Timeout = 100; //in msec
class MotorControllerHandler {
	public:
		MotorControllerHandler(ros::NodeHandle* nh, const char* Port);
		void sendMessage(Message);
		void transmit();
		void receive();
		void spinOnce();
		void processResponse();
		bool TransmitTimeout();
		void CheckQuery();
		void CheckMotor();
		void setMotorSpeed(int right, int left);
		void print(std::string error);
	private:
		int MaxStep;
		bool awaitingResponse;
		ros::Publisher errorOut;
		ros::Publisher currentMotorSetting;
		ros::Publisher motorStatus;
		ros::Publisher motorCurrent;
		Message currentMessage;
		string name;
		timeval lastSendTime;
		timeval lastQRCurTime;
		timeval lastQLCurTime;
		timeval lastQVoltTime;
		timeval lastMotorTime;;
		SerialPort serialPort;
		int rightSpeed;
		int leftSpeed;
		int rightTargetSpeed;
		int leftTargetSpeed;
		float RightCurrent;
		float LeftCurrent;
		float Voltage;
		char* portName;
		char buffer[8];
		int bufIndex;
};
