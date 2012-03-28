#include <sys/time.h>
#include <string>
#include <SerialStream.h>

using namespace std;

const char NO_MESSAGE = ' ';
const char MOTOR_TYPE = 'D';
const char MOTOR_RESPONCE_TYPE = 'd';
const char CURRENT_TYPE = 'C';
const char CURRENT_RESPONCE_TYPE = 'c';
const char VOLTAGE_TYPE = 'V';
const char VOLTAGE_RESPONCE_TYPE = 'v';
const char ERROR_TYPE = 'e';

const SerialPort::BaudRate BAUD = SerialPort::BAUD_115200;
//const SerialPort::BaudRate BAUD = SerialPort::BAUD_19200;

const int RESEND_TIMEOUT = 1000;
const int QUERY_PERIOD = 500;

struct Message {
	char type;
	union {
		char DataC[4];
		int DataI;
		float DataF;
	};
};

const int Timeout = 100; //in msec
class MotorControllerHandler {
	public:
		MotorControllerHandler(const char* Port);
		void sendMessage(Message);
		void transmit();
		void recieve();
		void spinOnce();
		void processResponce();
		bool TransmitTimeout();
		void CheckPullTimes();
	private:
		bool awaitingResponce;
		Message currentMessage;
		Message nextMessage;
		string name;
		timeval lastSendTime;
		timeval lastQueryTime;
		SerialPort serialPort;
		float RightCurrent;
		float LeftCurrent;
		float Voltage;
		char* portName;
		char buffer[8];
		int bufIndex;
};
