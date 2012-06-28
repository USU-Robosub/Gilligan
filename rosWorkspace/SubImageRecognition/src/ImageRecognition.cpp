#define _USE_MATH_DEFINES0

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "SubImageRecognition/ImgRecAlgorithm.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/UpdateAlgorithm.h"

using namespace cv;
using namespace std;

// CONSTANTS

const int SAMPLE_SIZE = 6;
const unsigned int MIN_POINTS = 10;

const char NAMESPACE_ROOT[] = "img_rec/";

const char PARAM_FLAGS[] = "/flags"; // int (0+) see flag constants below
const char PARAM_H_MAX[] = "/h_max"; // double (0-255)
const char PARAM_H_MIN[] = "/h_min"; // double (0-255)
const char PARAM_S_MAX[] = "/s_max"; // double (0-255)
const char PARAM_S_MIN[] = "/s_min"; // double (0-255)
const char PARAM_V_MAX[] = "/v_max"; // double (0-255)
const char PARAM_V_MIN[] = "/v_min"; // double (0-255)

const int FLAG_ENABLED = 1;
const int FLAG_PUBLISH_THRESHOLD = 2;

const int CAMERA_FORWARD = 0;
const int CAMERA_DOWNWARD = 1;

const int ANALYSIS_RECTANGLE = 0;

const int CONFIDENCE_RECTANGLE = 0;
const int CONFIDENCE_CIRCLE = 1;

const int ANNOTATION_ROTATION = 0;
const int ANNOTATION_RADIUS = 1;

// DEFINITIONS

typedef vector<Point> Points;

class Algorithm {
private:
	string buildParamName(const char* param) {
		string paramName(NAMESPACE_ROOT);
		paramName += name;
		paramName += param;
		return paramName;
	}

public:
	string name;
	int flags;
	int camera;
	Scalar minThreshold;
	Scalar maxThreshold;
	int analysisType;
	int maxBlobs;
	int confidenceType;
	Scalar annotationColor;
	int annotationType;
	ros::Publisher publisher;

	Algorithm(
			string _name,
			int _camera,
			int _analysisType,
			int _maxBlobs,
			int _confidenceType,
			Scalar _annotationColor,
			int _annotationType) {
		// Save class properties
		name = _name;
		camera = _camera;
		analysisType = _analysisType;
		maxBlobs = _maxBlobs;
		confidenceType = _confidenceType;
		annotationColor = _annotationColor;
		annotationType = _annotationType;

		// Retrieve persisted settings from parameter server
		ros::NodeHandle nodeHandle;
		nodeHandle.param<int>(buildParamName(PARAM_FLAGS), flags, 0);
		nodeHandle.param<double>(buildParamName(PARAM_H_MAX), maxThreshold[0], 255);
		nodeHandle.param<double>(buildParamName(PARAM_H_MIN), minThreshold[0], 0);
		nodeHandle.param<double>(buildParamName(PARAM_S_MAX), maxThreshold[1], 255);
		nodeHandle.param<double>(buildParamName(PARAM_S_MIN), minThreshold[1], 0);
		nodeHandle.param<double>(buildParamName(PARAM_V_MAX), maxThreshold[2], 255);
		nodeHandle.param<double>(buildParamName(PARAM_V_MIN), minThreshold[2], 0);

		// Fix hue thresholds
		maxThreshold[0] *= 179.0 / 255.0;
		minThreshold[0] *= 179.0 / 255.0;

		// Prepare the publisher for use later on
		string topic(NAMESPACE_ROOT);
		topic += name;
		publisher =
				nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
	}

	~Algorithm() {
		// Save persisted settings on parameter server
		//TODO
	}
};

class BlobAnalysis {
public:
	int center_x;
	int center_y;
	float rotation;
	unsigned int width;
	unsigned int height;
	unsigned int size;

	BlobAnalysis() {}

	// Constructor for use with ANALYSIS_RECTANGLE
	BlobAnalysis(Points& blob, RotatedRect rectangle) {
		center_x = (int) rectangle.center.x;
		center_y = (int) rectangle.center.y;
		rotation = rectangle.angle;
		width = (unsigned int) rectangle.size.width;
		height = (unsigned int) rectangle.size.height;
		size = blob.size();

		// Convert rotation from degrees to radians
		rotation *= M_PI / 180.0;

		// Correct dimensions and rotation so that height is always larger
		if (height < width) {
			unsigned int temp = height;
			height = width;
			width = temp;
		} else {
			rotation -= M_PI / 2.0;
		}
	}
};

// GLOBALS  :/  HA HA AH WELL

vector<Algorithm> algorithms;

int forwardOffset = 0, downwardOffset = 0;
image_transport::Publisher forwardPublisher, downwardPublisher;
cv_bridge::CvImage forwardRotated, downwardRotated;
Mat forwardSegmented, downwardSegmented;
Mat forwardThreshold, downwardThreshold;

// FUNCTIONS

void initAlgorithms() {
	algorithms.push_back(Algorithm(
		"gate",
		CAMERA_FORWARD,
		ANALYSIS_RECTANGLE,
		2,
		CONFIDENCE_RECTANGLE,
		Scalar(0, 128, 255), // Orange
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
		"buoys/red",
		CAMERA_FORWARD,
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 0, 255), // Red
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"buoys/green",
		CAMERA_FORWARD,
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 0), // Green
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"buoys/yellow",
		CAMERA_FORWARD,
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 255), // Yellow
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"obstacle_course",
		CAMERA_FORWARD,
		ANALYSIS_RECTANGLE,
		3,
		CONFIDENCE_RECTANGLE,
		Scalar(255, 0, 0), // Blue
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
		"paths",
		CAMERA_DOWNWARD,
		ANALYSIS_RECTANGLE,
		2,
		CONFIDENCE_RECTANGLE,
		Scalar(0, 128, 255), // Orange
		ANNOTATION_ROTATION
	));
}

void normalizeValue(Mat& image, Mat& temp) {
	const static int valueOut[] = {2, 0};
	const static int valueIn[] = {0, 2};
	temp.create(image.rows, image.cols, CV_8UC1);
	mixChannels(&image, 1, &temp, 1, valueOut, 1);
	normalize(temp, temp, 0, 255, CV_MINMAX);
	mixChannels(&temp, 1, &image, 1, valueIn, 1);
}

void reduceNoise(Mat& image) {
	const static Size size(3, 3);
	const static Point point(1, 1);
	const static Mat elementRect = getStructuringElement(
			MORPH_RECT, size, point);
	erode(image, image, elementRect, point, 4);
	dilate(image, image, elementRect, point, 2);
}

Points findBlob(Mat& image, int i, int j) {
	unsigned int index = 0;
	Points blob;
	Point point(j, i);
	blob.push_back(point);
	image.at<uint8_t>(i, j, 0) = 127;
	while (index < blob.size()) {
		point = blob[index];
		i = point.y;
		j = point.x;
		if (i+SAMPLE_SIZE < image.rows
				&& image.at<uint8_t>(i+SAMPLE_SIZE, j, 0) == 255) {
			blob.push_back(Point(j, i+SAMPLE_SIZE));
			image.at<uint8_t>(i+SAMPLE_SIZE, j, 0) = 127;
		}
		if (i-SAMPLE_SIZE >= 0
				&& image.at<uint8_t>(i-SAMPLE_SIZE, j, 0) == 255) {
			blob.push_back(Point(j, i-SAMPLE_SIZE));
			image.at<uint8_t>(i-SAMPLE_SIZE, j, 0) = 127;
		}
		if (j+SAMPLE_SIZE < image.cols
				&& image.at<uint8_t>(i, j+SAMPLE_SIZE, 0) == 255) {
			blob.push_back(Point(j+SAMPLE_SIZE, i));
			image.at<uint8_t>(i, j+SAMPLE_SIZE, 0) = 127;
		}
		if (j-SAMPLE_SIZE >= 0 &&
				image.at<uint8_t>(i, j-SAMPLE_SIZE, 0) == 255) {
			blob.push_back(Point(j-SAMPLE_SIZE, i));
			image.at<uint8_t>(i, j-SAMPLE_SIZE, 0) = 127;
		}
		index++;
	}
	return blob;
}

bool compareBlobs(Points& blob0, Points& blob1) {
	return blob0.size() > blob1.size();
}

vector<Points> findBlobs(Mat& image,
		const int offset, const unsigned int maxBlobs) {
	// First get all blobs that are at least the minimum size
	vector<Points> allBlobs;
	for (int i = offset; i < image.rows; i += SAMPLE_SIZE) {
		for (int j = offset; j < image.cols; j += SAMPLE_SIZE) {
			if (image.at<uint8_t>(i, j, 0) == 255) {
				Points blob = findBlob(image, i, j);
				if (blob.size() >= MIN_POINTS) {
					allBlobs.push_back(blob);
				}
			}
		}
	}
	// Stop now if there are 'maxBlobs' or fewer blobs
	if (allBlobs.size() <= maxBlobs) {
		return allBlobs;
	}
	// Otherwise limit to the biggest 'maxBlobs' blobs
	make_heap(allBlobs.begin(), allBlobs.end(), compareBlobs);
	vector<Points> blobs = vector<Points>();
	blobs.push_back(allBlobs.front());
	for (unsigned int i = 1; i < maxBlobs && allBlobs.size() > 1; i++) {
		pop_heap(allBlobs.begin(), allBlobs.end(), compareBlobs);
		blobs.push_back(allBlobs.front());
	}
	return blobs;
}

vector<BlobAnalysis> analyzeBlob(Algorithm& algorithm,
		Points& blob, Mat& image) {
	vector<BlobAnalysis> analysisList;
	RotatedRect rectangle;
	switch (algorithm.analysisType) {
	case ANALYSIS_RECTANGLE:
		analysisList.push_back(BlobAnalysis(blob, minAreaRect(Mat(blob))));
		break;
	}
	return analysisList;
}

float computeConfidence(Algorithm& algorithm, BlobAnalysis& a) {
	// A return value of -1 indicates 'divide by zero' error
	// A return value of -2 indicates 'unknown confidence type' error
	int expectedPoints;
	switch (algorithm.confidenceType) {
	case CONFIDENCE_RECTANGLE:
		expectedPoints = (a.width * a.height) / (SAMPLE_SIZE * SAMPLE_SIZE);
		break;
	case CONFIDENCE_CIRCLE:
		expectedPoints =
				(M_PI * a.width * a.height) / (4 * SAMPLE_SIZE * SAMPLE_SIZE);
		break;
	default:
		return -2; // Unknown confidence type
	}
	if (expectedPoints <= 0) {
		return -1; // Divide by zero
	} else {
		float confidence = ((float) a.size) / ((float) expectedPoints);
		if (confidence > 1) {
			return 1;
		} else {
			return confidence;
		}
	}
}

void annotateImage(Mat& image, Algorithm& algorithm, BlobAnalysis& a) {
	int r, x, y;
	switch (algorithm.annotationType) {
	case ANNOTATION_ROTATION:
		x = (int) (a.height / 2.0 * cos(a.rotation));
		y = (int) (a.height / 2.0 * sin(a.rotation));
		circle(image, Point(a.center_x, a.center_y), 1,
				algorithm.annotationColor, 5, CV_AA);
		line(image, Point(a.center_x, a.center_y),
				Point(a.center_x + x, a.center_y + y),
				algorithm.annotationColor, 1, CV_AA);
		break;
	case ANNOTATION_RADIUS:
		r = (int) ((a.width + a.height) / 2.0);
		circle(image, Point(a.center_x, a.center_y), r,
				algorithm.annotationColor, 2, CV_AA);
		break;
	}
}

void genericCallback(
		const int camera,
		const sensor_msgs::ImageConstPtr& rosImage,
		cv_bridge::CvImage& rotated,
		Mat& segmented,
		Mat& threshold,
		const int offset,
		const image_transport::Publisher& publisher) {
	// Copy image from ROS format to OpenCV format
	cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(rosImage, "bgr8");

	// Rotate image upright
	transpose(cvImage->image, rotated.image);
	flip(rotated.image, rotated.image, 0); // 0=ccw, 1=cw
	rotated.encoding = cvImage->encoding;

	// Segment into HSV
	cvtColor(rotated.image, segmented, CV_BGR2HSV);

	// Normalize brightness and copy back to BGR
	//normalizeValue(segmented, threshold);
	//cvtColor(segmented, rotated.image, CV_HSV2BGR);

	// Iterate through all algorithms
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		Algorithm algorithm = algorithms.at(i);
		// Run applicable algorithms
		if ((algorithm.flags & FLAG_ENABLED) && algorithm.camera == camera) {
			inRange(segmented, algorithm.minThreshold,
					algorithm.maxThreshold, threshold);
			reduceNoise(threshold);
			vector<Points> blobs = findBlobs(
					threshold, offset, algorithm.maxBlobs);
			if (algorithm.flags & FLAG_PUBLISH_THRESHOLD) {
				cv_bridge::CvImage temp;
				temp.encoding = "mono8";
				temp.image = threshold;
				publisher.publish(temp.toImageMsg());
			}
			// Iterate through all blobs
			for (unsigned int j = 0; j < blobs.size(); j++) {
				Points blob = blobs.at(j);
				vector<BlobAnalysis> analysisList =
						analyzeBlob(algorithm, blob, rotated.image);
				// Iterate through all blob analysis objects
				for (unsigned int k = 0; k < analysisList.size(); k++) {
					BlobAnalysis analysis = analysisList[k];
					// Publish information
					SubImageRecognition::ImgRecObject msg;
					msg.center_x = analysis.center_x - rotated.image.cols / 2;
					msg.center_y = rotated.image.rows / 2 - analysis.center_y;
					msg.rotation = analysis.rotation + M_PI / 2.0;
					msg.width = analysis.width;
					msg.height = analysis.height;
					msg.confidence = computeConfidence(algorithm, analysis);
					algorithm.publisher.publish(msg);
					// Annotate image
					annotateImage(rotated.image, algorithm, analysis);
				}
			}
		}
	}

	// Publish annotated image
	publisher.publish(rotated.toImageMsg());
}

void forwardCallback(const sensor_msgs::ImageConstPtr& rosImage) {
	genericCallback(CAMERA_FORWARD, rosImage, forwardRotated, forwardSegmented,
			forwardThreshold, forwardOffset, forwardPublisher);
	forwardOffset = (forwardOffset + 1) % SAMPLE_SIZE;
}

void downwardCallback(const sensor_msgs::ImageConstPtr& rosImage) {
	genericCallback(CAMERA_DOWNWARD, rosImage, downwardRotated, downwardSegmented,
			downwardThreshold, downwardOffset, downwardPublisher);
	downwardOffset = (downwardOffset + 1) % SAMPLE_SIZE;
}

bool listAlgorithmsCallback(
		SubImageRecognition::ListAlgorithms::Request& req,
		SubImageRecognition::ListAlgorithms::Response& res) {
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		// TODO
		//res.algorithms.push_back(algorithms.at(i).name);
	}
	return true;
}

bool updateAlgorithmCallback(
		SubImageRecognition::UpdateAlgorithm::Request& req,
		SubImageRecognition::UpdateAlgorithm::Response& res) {
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		// TODO
		/*
		if (req.algorithm.compare(algorithms.at(i).name) == 0) {
			algorithms.at(i).enabled = (req.enabled != 0);
			res.result = 1;
			break;
		}
		*/
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ImageRecognition");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport imageTransport(nodeHandle);

	forwardPublisher = imageTransport.advertise("forward_camera/image_raw", 1);
	downwardPublisher = imageTransport.advertise("downward_camera/image_raw", 1);

	string listAlgorithmsTopic(NAMESPACE_ROOT);
	listAlgorithmsTopic += "list_algorithms";
	ros::ServiceServer listAlgorithmsService = nodeHandle.advertiseService(
			listAlgorithmsTopic, listAlgorithmsCallback);

	string updateAlgorithmTopic(NAMESPACE_ROOT);
	updateAlgorithmTopic += "update_algorithm";
	ros::ServiceServer updateAlgorithmService = nodeHandle.advertiseService(
			updateAlgorithmTopic, updateAlgorithmCallback);

	image_transport::Subscriber forwardSubscriber =
			imageTransport.subscribe("left/image_raw", 1, forwardCallback);
	image_transport::Subscriber downwardSubscriber =
			imageTransport.subscribe("right/image_raw", 1, downwardCallback);

	initAlgorithms();
	ros::spin();
	return 0;
}

