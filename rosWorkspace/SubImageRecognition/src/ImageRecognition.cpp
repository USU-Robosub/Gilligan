#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>
#include <vector>

#include "SubImageRecognition/ImgRecObject.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/SwitchAlgorithm.h"

using namespace std;

// CONSTANTS

const int SAMPLE_SIZE = 6;
const int MIN_POINTS = 30;

const float MAX_LENGTH_THRESHOLD = 0.8;

const char TOPIC_ROOT[] = "image_recognition/";

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
	string name;
	int camera;
	cv::Scalar minThreshold;
	cv::Scalar maxThreshold;
	int analysis;
	int maxPointSets;
	int confidenceType;
	ros::Publisher publisher;
	
	Algorithm(
			bool enabled,
			string name,
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
		ros::NodeHandle nodeHandle;
		string topic(TOPIC_ROOT);
		topic += this->name;
		this->publisher =
				nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
	}
};

// GLOBALS  :/  HA HA AH WELL

vector<Algorithm *> algorithms;

int forwardCounter = 0;
int downwardCounter = 0;
image_transport::Publisher forwardPub;
image_transport::Publisher downwardPub;

cv_bridge::CvImage forwardRotated, downwardRotated;
cv_bridge::CvImage forwardSegmented, downwardSegmented;
cv_bridge::CvImage forwardThreshold, downwardThreshold;

// FUNCTIONS

void initAlgorithms() {
	algorithms.push_back(new Algorithm(
		false,
		"gate",
		CAMERA_FORWARD,
		cv::Scalar(0, 0, 0),
		cv::Scalar(250, 180, 60),
		ANALYSIS_GATE,
		1,
		CONFIDENCE_RECTANGLE
	));
	algorithms.push_back(new Algorithm(
		false,
		"buoys/red",
		CAMERA_FORWARD,
		cv::Scalar(135, 0, 30),
		cv::Scalar(200, 210, 120),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	));
	algorithms.push_back(new Algorithm(
		false,
		"buoys/green",
		CAMERA_FORWARD,
		cv::Scalar(110, 200, 110),
		cv::Scalar(130, 240, 200),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	));
	algorithms.push_back(new Algorithm(
		false,
		"buoys/yellow",
		CAMERA_FORWARD,
		cv::Scalar(95, 185, 160),
		cv::Scalar(115, 240, 220),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE
	));
	algorithms.push_back(new Algorithm(
		false,
		"obstacle_course",
		CAMERA_FORWARD,
		cv::Scalar(0, 0, 0),
		cv::Scalar(255, 255, 255),
		ANALYSIS_RECTANGLE,
		3,
		CONFIDENCE_RECTANGLE
	));
	algorithms.push_back(new Algorithm(
		false,
		"paths",
		CAMERA_DOWNWARD,
		cv::Scalar(5, 50, 50),
		cv::Scalar(15, 255, 255),
		ANALYSIS_RECTANGLE,
		2,
		CONFIDENCE_RECTANGLE
	));
}

void forwardCallback(const sensor_msgs::ImageConstPtr &rosImg) {
	const static int valueOut[] = {2, 0};
	const static int valueIn[] = {0, 2};

	// Copy image from ROS format to OpenCV format
	cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(rosImg);

	// Rotate image upright
	cv::transpose(cvImg->image, forwardRotated.image);
	cv::flip(forwardRotated.image, forwardRotated.image, 0); // 0=ccw, 1=cw

	// Segment into HSV
	cv::cvtColor(forwardRotated.image, forwardSegmented.image, CV_BGR2HSV);

	// Normalize brightness (the c versions for split and merge are easier/faster to call than the the c++ versions)
	forwardThreshold.image.create(forwardSegmented.image.rows, forwardSegmented.image.cols, CV_8UC1);
	cv::mixChannels(&(forwardSegmented.image), 1, &(forwardThreshold.image), 1, valueOut, 1);
	cv::normalize(forwardThreshold.image, forwardThreshold.image, 0, 255, CV_MINMAX);
	cv::mixChannels(&(forwardThreshold.image), 1, &(forwardSegmented.image), 1, valueIn, 1);
	cv::cvtColor(forwardSegmented.image, forwardRotated.image, CV_HSV2BGR);

	// Run applicable algorithms
	// TODO

	// Publish annotated image
	forwardPub.publish(forwardRotated.toImageMsg());
}

void downwardCallback(const sensor_msgs::ImageConstPtr &rosImg) {
	const static int valueOut[] = {2, 0};
	const static int valueIn[] = {0, 2};

	// Copy image from ROS format to OpenCV format
	cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(rosImg);

	// Rotate image upright
	cv::transpose(cvImg->image, downwardRotated.image);
	cv::flip(downwardRotated.image, downwardRotated.image, 0); // 0=ccw, 1=cw

	// Segment into HSV
	cv::cvtColor(downwardRotated.image, downwardSegmented.image, CV_BGR2HSV);

	// Normalize brightness
	downwardThreshold.image.create(downwardSegmented.image.rows, downwardSegmented.image.cols, CV_8UC1);
	cv::mixChannels(&(downwardSegmented.image), 1, &(downwardThreshold.image), 1, valueOut, 1);
	cv::normalize(downwardThreshold.image, downwardThreshold.image, 0, 255, CV_MINMAX);
	cv::mixChannels(&(downwardThreshold.image), 1, &(downwardSegmented.image), 1, valueIn, 1);
	cv::cvtColor(downwardSegmented.image, downwardRotated.image, CV_HSV2BGR);

	// Run applicable algorithms
	// TODO

	// Publish annotated image
	downwardPub.publish(downwardRotated.toImageMsg());
}

bool listAlgorithmsCallback(
		SubImageRecognition::ListAlgorithms::Request &req,
		SubImageRecognition::ListAlgorithms::Response &res) {
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		res.algorithms.push_back(algorithms.at(i)->name);
	}
	return true;
}

bool switchAlgorithmCallback(
		SubImageRecognition::SwitchAlgorithm::Request &req,
		SubImageRecognition::SwitchAlgorithm::Response &res) {
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		if (req.algorithm.compare(algorithms.at(i)->name) == 0) {
			algorithms.at(i)->enabled = (req.enabled != 0);
			res.result = 1;
			break;
		}
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ImageRecognition");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport imageTransport(nodeHandle);

	imageTransport.subscribe("left/image_raw", 1, forwardCallback);
//	imageTransport.subscribe("right/image_raw", 1, downwardCallback);

	forwardPub = imageTransport.advertise("forward_camera/image_raw", 1);
	downwardPub = imageTransport.advertise("downward_camera/image_raw", 1);

	string listAlgorithmsTopic(TOPIC_ROOT);
	listAlgorithmsTopic += "list_algorithms";
	ros::ServiceServer listAlgorithmsService =
			nodeHandle.advertiseService(listAlgorithmsTopic, listAlgorithmsCallback);

	string switchAlgorithmTopic(TOPIC_ROOT);
	switchAlgorithmTopic += "switch_algorithm";
	ros::ServiceServer switchAlgorithmService =
			nodeHandle.advertiseService(switchAlgorithmTopic, switchAlgorithmCallback);

	initAlgorithms();
	ros::spin();
	return 0;
}

