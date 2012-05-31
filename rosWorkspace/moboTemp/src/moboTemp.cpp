#include <stdlib.h>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "moboTemp");
	string line;
	ifstream temperatureFile;
	ros::NodeHandle rosHandle;

	ros::Publisher temperaturePublisher = rosHandle.advertise<std_msgs::Float32MultiArray>("Mobo_Temp", 1);

	ros::Rate loopRate(1);

	temperatureFile.open("temp.out");

	while (ros::ok())
 	{
		std_msgs::Float32MultiArray msg;

		system("sensors | grep -o '+.*C\ ' > temp.out");
		temperatureFile.open("temp.out");

		while (temperatureFile.good())
		{
			getline(temperatureFile, line);
			if (line != "")
			{
				float value = strtof(line.c_str(), NULL);
				value = (9.0/5.0) * value + 32.0;
				msg.data.push_back(value);
			}
		}

		temperaturePublisher.publish(msg);
		temperatureFile.close();

		ros::spinOnce();
		loopRate.sleep();
 	}

	temperatureFile.close();

 	return 0;
 }
