#ifndef _PATH_TASK_HPP
#define _PATH_TASK_HPP

#include <ros/ros.h>
//#include "std_msgs/UInt8.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

struct PathData
{
	ros::Time lastSampleTime;
	short centerX;
	short centerY;
	float confidence;
	float rotation;
};

class PathTask
{
public:
	PathTask();
	~PathTask();

	void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
	void pathCallback(const SubImageRecognition::ImgRecObject& msg);

private:
	void performTask(void);

	PathData m_path;
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_pathSubscriber;
	ros::Subscriber m_taskStateSubscriber;
	ros::Publisher m_highLevelMotorPublisher;
	bool m_isEnabled;

};

#endif // _PATH_TASK_HPP
