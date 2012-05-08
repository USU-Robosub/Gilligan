#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
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
const char IMGRECOBJECT_NAME[] = "deprecated";

const int CAMERA_FORWARD = 0;
const int CAMERA_DOWNWARD = 1;

const int ANALYSIS_RECTANGLE = 0;
const int ANALYSIS_GATE = 1;

const int CONFIDENCE_RECTANGLE = 0;
const int CONFIDENCE_CIRCLE = 1;

// DEFINITIONS

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
		// Correct hue values - the 0.5 is to allow us to round up
		minThreshold[0] = (int) ((minThreshold[0] * 179.0 / 255.0) + 0.5);
		maxThreshold[0] = (int) ((maxThreshold[0] * 179.0 / 255.0) + 0.5);

		// Save class properties
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

int forwardOffset = 0, downwardOffset = 0;
image_transport::Publisher forwardPublisher, downwardPublisher;
cv_bridge::CvImage forwardRotated, downwardRotated;
cv::Mat forwardSegmented, downwardSegmented;
cv::Mat forwardThreshold, downwardThreshold;

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

void normalizeValue(cv::Mat &image, cv::Mat &temp) {
	const static int valueOut[] = {2, 0};
	const static int valueIn[] = {0, 2};
	temp.create(image.rows, image.cols, CV_8UC1);
	cv::mixChannels(&image, 1, &temp, 1, valueOut, 1);
	cv::normalize(temp, temp, 0, 255, CV_MINMAX);
	cv::mixChannels(&temp, 1, &image, 1, valueIn, 1);
}

void reduceNoise(cv::Mat &image) {
	const static cv::Size size = cv::Size(3, 3);
	const static cv::Point point = cv::Point(1, 1);
	const static cv::Mat elementEllipse = cv::getStructuringElement(
			cv::MORPH_ELLIPSE, size, point);
	const static cv::Mat elementRect = cv::getStructuringElement(
			cv::MORPH_RECT, size, point);
	cv::erode(image, image, elementEllipse, point, 2);
	cv::dilate(image, image, elementEllipse, point, 4);
	cv::erode(image, image, elementRect, point, 2);
	cv::dilate(image, image, elementRect, point, 4);
}

vector<int> findBlob(cv::Mat &image, int i, int j) {
	uint8_t *pixelPtr = (uint8_t *) image.data;
	unsigned int index = 0;
	vector<int> blob = vector<int>();
	blob.push_back(i);
	blob.push_back(j);
	pixelPtr[i * image.cols + j] = 0;
	while (index < blob.size()) {
		i = blob[index];
		j = blob[index + 1];
		if (i + SAMPLE_SIZE < image.rows &&
				pixelPtr[(i + SAMPLE_SIZE) * image.cols + j] == 255) {
			blob.push_back(i + SAMPLE_SIZE);
			blob.push_back(j);
			pixelPtr[(i + SAMPLE_SIZE) * image.cols + j] = 127;
		}
		if (i - SAMPLE_SIZE >= 0 &&
				pixelPtr[(i - SAMPLE_SIZE) * image.cols + j] == 255) {
			blob.push_back(i - SAMPLE_SIZE);
			blob.push_back(j);
			pixelPtr[(i - SAMPLE_SIZE) * image.cols + j] = 127;
		}
		if (j + SAMPLE_SIZE < image.cols &&
				pixelPtr[i * image.cols + (j + SAMPLE_SIZE)] == 255) {
			blob.push_back(i);
			blob.push_back(j + SAMPLE_SIZE);
			pixelPtr[i * image.cols + (j + SAMPLE_SIZE)] = 127;
		}
		if (j - SAMPLE_SIZE >= 0 &&
				pixelPtr[i * image.cols + (j - SAMPLE_SIZE)] == 255) {
			blob.push_back(i);
			blob.push_back(j - SAMPLE_SIZE);
			pixelPtr[i * image.cols + (j - SAMPLE_SIZE)] = 127;
		}
		index += 2;
	}
	return blob;
}

vector<vector<int> > findBlobs(cv::Mat &image, const int offset) {
	uint8_t *pixelPtr = (uint8_t *) image.data;
	vector<vector<int> > blobs = vector<vector<int> >();
	for (int i = offset; i < image.rows; i += SAMPLE_SIZE) {
		for (int j = offset; j < image.cols; j += SAMPLE_SIZE) {
			if (pixelPtr[i * image.cols + j] == 255) {
				vector<int> blob = findBlob(image, i, j);
				if (blob.size() >= MIN_POINTS * 2) {
					blobs.push_back(blob);
				}
			}
		}
	}
	return blobs;
}

void genericCallback(
		const int camera,
		const sensor_msgs::ImageConstPtr &rosImage,
		cv_bridge::CvImage &rotated,
		cv::Mat &segmented,
		cv::Mat &threshold,
		const int offset,
		const image_transport::Publisher &publisher) {
	// Copy image from ROS format to OpenCV format
	cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(rosImage, "bgr8");

	// Rotate image upright
	cv::transpose(cvImage->image, rotated.image);
	cv::flip(rotated.image, rotated.image, 0); // 0=ccw, 1=cw
	rotated.encoding = cvImage->encoding;

	// Segment into HSV
	cv::cvtColor(rotated.image, segmented, CV_BGR2HSV);

	// Normalize brightness and copy back to BGR
	normalizeValue(segmented, threshold);
	cv::cvtColor(segmented, rotated.image, CV_HSV2BGR);

	// Run applicable algorithms
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		Algorithm *algorithm = algorithms.at(i);
		if (algorithm->enabled && algorithm->camera == camera) {
			cv::inRange(segmented, algorithm->minThreshold,
					algorithm->maxThreshold, threshold);
			reduceNoise(threshold);
			vector<vector<int> > blobs = findBlobs(segmented, offset);
			// TODO: Publish blobs based on algorithm settings
		}
	}

	// Publish annotated image
	publisher.publish(rotated.toImageMsg());
}

void forwardCallback(const sensor_msgs::ImageConstPtr &rosImage) {
	genericCallback(CAMERA_FORWARD, rosImage, forwardRotated, forwardSegmented,
			forwardThreshold, forwardOffset, forwardPublisher);
	forwardOffset = (forwardOffset + 1) % SAMPLE_SIZE;
}

void downwardCallback(const sensor_msgs::ImageConstPtr &rosImage) {
	genericCallback(CAMERA_DOWNWARD, rosImage, downwardRotated, downwardSegmented,
			downwardThreshold, downwardOffset, downwardPublisher);
	downwardOffset = (downwardOffset + 1) % SAMPLE_SIZE;
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

	forwardPublisher = imageTransport.advertise("forward_camera/image_raw", 1);
	downwardPublisher = imageTransport.advertise("downward_camera/image_raw", 1);

	string listAlgorithmsTopic(TOPIC_ROOT);
	listAlgorithmsTopic += "list_algorithms";
	ros::ServiceServer listAlgorithmsService = nodeHandle.advertiseService(
			listAlgorithmsTopic, listAlgorithmsCallback);

	string switchAlgorithmTopic(TOPIC_ROOT);
	switchAlgorithmTopic += "switch_algorithm";
	ros::ServiceServer switchAlgorithmService = nodeHandle.advertiseService(
			switchAlgorithmTopic, switchAlgorithmCallback);

	image_transport::Subscriber forwardSubscriber =
			imageTransport.subscribe("left/image_raw", 1, forwardCallback);
	image_transport::Subscriber downwardSubscriber =
			imageTransport.subscribe("right/image_raw", 1, downwardCallback);

	initAlgorithms();
	ros::spin();
	return 0;
}

