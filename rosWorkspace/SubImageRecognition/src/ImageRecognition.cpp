#define _USE_MATH_DEFINES

#include <algorithm>
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

using namespace cv;
using namespace std;

// CONSTANTS

const int SAMPLE_SIZE = 6;
const int MIN_POINTS = 30;

// XXX: Used by ANALYSIS_GATE
//const float MAX_LENGTH_THRESHOLD = 0.8;

const char TOPIC_ROOT[] = "image_recognition/";

const int CAMERA_FORWARD = 0;
const int CAMERA_DOWNWARD = 1;

const int ANALYSIS_RECTANGLE = 0;
const int ANALYSIS_GATE = 1;

const int CONFIDENCE_RECTANGLE = 0;
const int CONFIDENCE_CIRCLE = 1;

const int ANNOTATION_ROTATION = 0;
const int ANNOTATION_RADIUS = 1;

// DEFINITIONS

typedef vector<Point> Points;

class Algorithm {
public:
	bool enabled;
	string name;
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
			bool _enabled,
			string _name,
			int _camera,
			Scalar _minThreshold,
			Scalar _maxThreshold,
			int _analysisType,
			int _maxBlobs,
			int _confidenceType,
			Scalar _annotationColor,
			int _annotationType) {
		// Correct hue values - the 0.5 is to allow us to round up
		_minThreshold[0] = (int) ((_minThreshold[0] * 179.0 / 255.0) + 0.5);
		_maxThreshold[0] = (int) ((_maxThreshold[0] * 179.0 / 255.0) + 0.5);

		// Save class properties
		enabled = _enabled;
		name = _name;
		camera = _camera;
		minThreshold = _minThreshold;
		maxThreshold = _maxThreshold;
		analysisType = _analysisType;
		maxBlobs = _maxBlobs;
		confidenceType = _confidenceType;
		annotationColor = _annotationColor;
		annotationType = _annotationType;

		// Prepare the publisher for use later on
		ros::NodeHandle nodeHandle;
		string topic(TOPIC_ROOT);
		topic += this->name;
		this->publisher =
				nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
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
			rotation += M_PI / 2.0;
		}

		// Normalize rotation for drawing
		if (sin(rotation) > 0) {
			rotation += M_PI;
		}
		while (rotation >= M_PI * 2.0) {
			rotation -= M_PI * 2.0;
		}

		// XXX: We used to do the image annotation here but now it's elsewhere
		//      so this can be simplified and the rotation needs to be fixed
		//      before being drawn later on

		// Normalize rotation for publishing
		if (rotation == 0) {
			rotation = M_PI * 2;
		}
		rotation -= M_PI * 1.5;
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
		false,
		"gate",
		CAMERA_FORWARD,
		Scalar(0, 0, 0),
		Scalar(250, 180, 60),
		ANALYSIS_RECTANGLE, // XXX: Was ANALYSIS_GATE
		1,
		CONFIDENCE_RECTANGLE,
		Scalar(0, 128, 255), // Orange
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
		false,
		"buoys/red",
		CAMERA_FORWARD,
		Scalar(135, 0, 30),
		Scalar(200, 210, 120),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 0, 255), // Red
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		false,
		"buoys/green",
		CAMERA_FORWARD,
		Scalar(110, 200, 110),
		Scalar(130, 240, 200),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 0), // Green
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		false,
		"buoys/yellow",
		CAMERA_FORWARD,
		Scalar(95, 185, 160),
		Scalar(115, 240, 220),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 255), // Yellow 
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		false,
		"obstacle_course",
		CAMERA_FORWARD,
		Scalar(0, 0, 0),
		Scalar(255, 255, 255),
		ANALYSIS_RECTANGLE,
		3,
		CONFIDENCE_RECTANGLE,
		Scalar(255, 0, 0), // Blue
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
		false,
		"paths",
		CAMERA_DOWNWARD,
		Scalar(5, 50, 50),
		Scalar(15, 255, 255),
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
	const static Mat elementEllipse = getStructuringElement(
			MORPH_ELLIPSE, size, point);
	const static Mat elementRect = getStructuringElement(
			MORPH_RECT, size, point);
	erode(image, image, elementEllipse, point, 2);
	dilate(image, image, elementEllipse, point, 4);
	erode(image, image, elementRect, point, 2);
	dilate(image, image, elementRect, point, 4);
}

Points findBlob(Mat& image, int i, int j) {
	uint8_t *pixelPtr = (uint8_t *) image.data;
	unsigned int index = 0;
	Points blob;
	Point point;
	blob.push_back(Point(i, j));
	pixelPtr[i * image.cols + j] = 0;
	while (index < blob.size()) {
		point = blob[index];
		i = point.x;
		j = point.y;
		if (i + SAMPLE_SIZE < image.rows &&
				pixelPtr[(i + SAMPLE_SIZE) * image.cols + j] == 255) {
			blob.push_back(Point(i + SAMPLE_SIZE, j));
			pixelPtr[(i + SAMPLE_SIZE) * image.cols + j] = 127;
		}
		if (i - SAMPLE_SIZE >= 0 &&
				pixelPtr[(i - SAMPLE_SIZE) * image.cols + j] == 255) {
			blob.push_back(Point(i - SAMPLE_SIZE, j));
			pixelPtr[(i - SAMPLE_SIZE) * image.cols + j] = 127;
		}
		if (j + SAMPLE_SIZE < image.cols &&
				pixelPtr[i * image.cols + (j + SAMPLE_SIZE)] == 255) {
			blob.push_back(Point(i, j + SAMPLE_SIZE));
			pixelPtr[i * image.cols + (j + SAMPLE_SIZE)] = 127;
		}
		if (j - SAMPLE_SIZE >= 0 &&
				pixelPtr[i * image.cols + (j - SAMPLE_SIZE)] == 255) {
			blob.push_back(Point(i, j - SAMPLE_SIZE));
			pixelPtr[i * image.cols + (j - SAMPLE_SIZE)] = 127;
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
	uint8_t *pixelPtr = (uint8_t *) image.data;
	vector<Points> allBlobs;
	for (int i = offset; i < image.rows; i += SAMPLE_SIZE) {
		for (int j = offset; j < image.cols; j += SAMPLE_SIZE) {
			if (pixelPtr[i * image.cols + j] == 255) {
				Points blob = findBlob(image, i, j);
				if (blob.size() >= MIN_POINTS * 2) {
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
	case ANALYSIS_GATE:
		// TODO?
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
	normalizeValue(segmented, threshold);
	cvtColor(segmented, rotated.image, CV_HSV2BGR);

	// Run applicable algorithms
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		Algorithm algorithm = algorithms.at(i);
		if (algorithm.enabled && algorithm.camera == camera) {
			inRange(segmented, algorithm.minThreshold,
					algorithm.maxThreshold, threshold);
			reduceNoise(threshold);
			vector<Points> blobs = findBlobs(
					segmented, offset, algorithm.maxBlobs);
			for (unsigned int j = 0; j < blobs.size(); j++) {
				Points blob = blobs.at(j);
				vector<BlobAnalysis> analysisList =
						analyzeBlob(algorithm, blob, rotated.image);
				for (unsigned int k = 0; k < analysisList.size(); k++) {
					float confidence =
							computeConfidence(algorithm, analysisList[k]);
					// TODO: Publish analysis based on algorithm settings
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
		res.algorithms.push_back(algorithms.at(i).name);
	}
	return true;
}

bool switchAlgorithmCallback(
		SubImageRecognition::SwitchAlgorithm::Request& req,
		SubImageRecognition::SwitchAlgorithm::Response& res) {
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		if (req.algorithm.compare(algorithms.at(i).name) == 0) {
			algorithms.at(i).enabled = (req.enabled != 0);
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

