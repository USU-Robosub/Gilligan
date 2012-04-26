#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>

#include "SubImageRecognition/ImgRecObject.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/SwitchAlgorithm.h"

using namespace std;

// CONSTANTS

const int SAMPLE_SIZE = 6;
const int MIN_POINTS = 30;

const int MAX_ALGORITHMS = 16;
const int MAX_TOPIC_LENGTH = 64;

const float MAX_LENGTH_THRESHOLD = 0.8;

const char TOPIC_ROOT[] = "image_recognition/";
const char TOPIC_FORWARD[] = "forward/";
const char TOPIC_DOWNWARD[] = "downward/";

const int CAMERA_FORWARD = 0;
const int CAMERA_DOWNWARD = 1;

const int ANALYSIS_RECTANGLE = 0;
const int ANALYSIS_GATE = 1;

const int CONFIDENCE_RECTANGLE = 0;
const int CONFIDENCE_CIRCLE = 1;

// CLASS DEFINITIONS

class Algorithm {
public:
	bool enabled;
	const char *name;
	int camera;
	cv::Scalar minThreshold;
	cv::Scalar maxThreshold;
	int analysis;
	int maxPointSets;
	int confidenceType;
	ros::Publisher publisher;
	
	Algorithm(
			bool enabled,
			const char *name,
			int camera,
			cv::Scalar minThreshold,
			cv::Scalar maxThreshold,
			int analysis,
			int maxPointSets,
			int confidenceType) {
		this->enabled = enabled;
		this->name = name;
		this->camera = camera;
		this->minThreshold = minThreshold;
		this->maxThreshold = maxThreshold;
		this->analysis = analysis;
		this->maxPointSets = maxPointSets;
		this->confidenceType = confidenceType;
		
		// Prepare the publisher for use later on
		// XXX: I guess this should have logic to make sure we don't go
		//      out of bounds in the character array. Or we could use the
		//      string class or something...
		ros::NodeHandle nodeHandle;
		char topic[MAX_TOPIC_LENGTH];
		strcpy(topic, TOPIC_ROOT);
		switch (camera) {
		case CAMERA_FORWARD:
			strcat(topic, TOPIC_FORWARD);
			strcat(topic, name);
			this->publisher = nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
			break;
		case CAMERA_DOWNWARD:
			strcat(topic, TOPIC_DOWNWARD);
			strcat(topic, name);
			this->publisher = nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
			break;
		}
	}
};

// GLOBALS  :/  HA HA AH WELL

Algorithm *algorithms[MAX_ALGORITHMS];

int forwardCounter = 0;
int downwardCounter = 0;
image_transport::Publisher forwardPub;
image_transport::Publisher downwardPub;

// FUNCTIONS

void initAlgorithms() {
	int i = 0;
	algorithms[i++] = new Algorithm(
		false,
		"gate",
		CAMERA_FORWARD,
		cv::Scalar(0, 0, 0),
		cv::Scalar(250, 180, 60),
		ANALYSIS_GATE,
		1,
		CONFIDENCE_RECTANGLE
	);
	algorithms[i++] = new Algorithm(
		false,
		"buoys/red",
		CAMERA_FORWARD,
		cv::Scalar(135, 0, 30),
		cv::Scalar(200, 210, 120),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	);
	algorithms[i++] = new Algorithm(
		false,
		"buoys/green",
		CAMERA_FORWARD,
		cv::Scalar(110, 200, 110),
		cv::Scalar(130, 240, 200),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	);
	algorithms[i++] = new Algorithm(
		false,
		"buoys/yellow",
		CAMERA_FORWARD,
		cv::Scalar(95, 185, 160),
		cv::Scalar(115, 240, 220),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	);
	algorithms[i++] = new Algorithm(
		false,
		"obstacle_course",
		CAMERA_FORWARD,
		cv::Scalar(0, 0, 0),
		cv::Scalar(255, 255, 255),
		ANALYSIS_RECTANGLE,
		3,
		CONFIDENCE_RECTANGLE
	);
	algorithms[i] = new Algorithm(
		false,
		"paths",
		CAMERA_DOWNWARD,
		cv::Scalar(5, 50, 50),
		cv::Scalar(15, 255, 255),
		ANALYSIS_RECTANGLE,
		2,
		CONFIDENCE_RECTANGLE
	);
}

void genericCallback(const int camera, const sensor_msgs::ImageConstPtr &rosImg) {
	// Copy image from ROS format to OpenCV format
	cv_bridge::CvImagePtr cvImg;
	try {
		cvImg = cv_bridge::toCvCopy(rosImg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("ImageRecognition: CvBridge Exception in genericCallback(): %s", e.what());
		return;
	}

	// Allocate image memory if needed
	//TODO

	// Rotate image counter-clockwise
	//TODO

	// Segment image into HSV
	//TODO

	// Normalize value channel
	//TODO

	// Loop through algorithms in settings
	//TODO
}

void forwardCallback(const sensor_msgs::ImageConstPtr &rosImg) {
	genericCallback(CAMERA_FORWARD, rosImg);
}

void downwardCallback(const sensor_msgs::ImageConstPtr &rosImg) {
	genericCallback(CAMERA_DOWNWARD, rosImg);
}

bool listAlgorithmsCallback(
		SubImageRecognition::ListAlgorithms::Request &req,
		SubImageRecognition::ListAlgorithms::Response &res) {
	// TODO
	return true;
}

bool switchAlgorithmCallback(
		SubImageRecognition::SwitchAlgorithm::Request &req,
		SubImageRecognition::SwitchAlgorithm::Response &res) {
	// TODO
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ImageRecognition");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport imageTransport(nodeHandle);
	imageTransport.subscribe("left/image_raw", 1, forwardCallback);
	imageTransport.subscribe("right/image_raw", 1, forwardCallback);
	forwardPub = imageTransport.advertise("forward_camera/image_raw", 1);
	downwardPub = imageTransport.advertise("downward_camera/image_raw", 1);
	char topic[MAX_TOPIC_LENGTH];
	strcpy(topic, TOPIC_ROOT);
	strcat(topic, "list_algorithms");
	nodeHandle.advertiseService(topic, listAlgorithmsCallback);
	strcpy(topic, TOPIC_ROOT);
	strcat(topic, "switch_algorithm");
	nodeHandle.advertiseService(topic, switchAlgorithmCallback);
	initAlgorithms();
	ros::spin();
	return 0;
}

