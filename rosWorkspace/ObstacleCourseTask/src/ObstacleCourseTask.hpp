#ifndef _OBSTACLE_COURSE_TASK_HPP
#define _OBSTACLE_COURSE_TASK_HPP

#include "ros/ros.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/ImgRecObject.h"

struct ObstacleCourseData
{
	ros::Time lastSampleTime;
	short centerX;
	short centerY;
	float confidence
	short height;
	short width;
};

class ObstacleCourseTask
{
public:
	ObstacleCourseTask();
	~ObstacleCourseTask();

	void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
	void obstacleCourseCallback(const SubImageRecognition::ImgRecObject& msg);

private:
	void performTask(void);

	ObstacleCourseData m_obstacleCourse;
	ros::NodeHandle m_nodeHandle;
	ros::Subscriber m_obstacleCourseSubscriber;
	ros::Subscriber m_taskStateSubscriber;
	ros::Publisher m_highLevelMotorPublisher;
	bool m_isEnabled;
};

#endif // _OBSTACLE_COURSE_TASK_HPP
