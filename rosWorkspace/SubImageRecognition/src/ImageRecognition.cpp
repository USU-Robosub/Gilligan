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

#include "SubImageRecognition/ImgRecAlgorithm.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "SubImageRecognition/ImgRecThreshold.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/UpdateAlgorithm.h"
#include "SubImageRecognition/SwitchAlgorithm.h"

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

	void saveSettings() {
		ros::NodeHandle nodeHandle;
		nodeHandle.setParam(buildParamName(PARAM_FLAGS), flags);
		nodeHandle.setParam(buildParamName(PARAM_H_MAX), maxThreshold[0] * 255.0 / 179.0);
		nodeHandle.setParam(buildParamName(PARAM_H_MIN), minThreshold[0] * 255.0 / 179.0);
		nodeHandle.setParam(buildParamName(PARAM_S_MAX), maxThreshold[1]);
		nodeHandle.setParam(buildParamName(PARAM_S_MIN), minThreshold[1]);
		nodeHandle.setParam(buildParamName(PARAM_V_MAX), maxThreshold[2]);
		nodeHandle.setParam(buildParamName(PARAM_V_MIN), minThreshold[2]);
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
			Scalar _minThreshold,
			Scalar _maxThreshold,
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
		nodeHandle.param<int>(buildParamName(PARAM_FLAGS), flags, 1);
		nodeHandle.param<double>(buildParamName(PARAM_H_MAX), maxThreshold[0], _maxThreshold[0]);
		nodeHandle.param<double>(buildParamName(PARAM_H_MIN), minThreshold[0], _minThreshold[0]);
		nodeHandle.param<double>(buildParamName(PARAM_S_MAX), maxThreshold[1], _maxThreshold[1]);
		nodeHandle.param<double>(buildParamName(PARAM_S_MIN), minThreshold[1], _minThreshold[1]);
		nodeHandle.param<double>(buildParamName(PARAM_V_MAX), maxThreshold[2], _maxThreshold[2]);
		nodeHandle.param<double>(buildParamName(PARAM_V_MIN), minThreshold[2], _minThreshold[2]);

		// Fix hue thresholds
		maxThreshold[0] *= 179.0 / 255.0;
		minThreshold[0] *= 179.0 / 255.0;

		saveSettings();

		// Prepare the publisher for use later on
		string topic(NAMESPACE_ROOT);
		topic += name;
		publisher =
				nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
	}

	void updateSettings(SubImageRecognition::ImgRecAlgorithm& a) {
		flags = (int) a.flags;
		maxThreshold[0] = ((double) a.h_max) * 179.0 / 255.0;
		minThreshold[0] = ((double) a.h_min) * 179.0 / 255.0;
		maxThreshold[1] = (double) a.s_max;
		minThreshold[1] = (double) a.s_min;
		maxThreshold[2] = (double) a.v_max;
		minThreshold[2] = (double) a.v_min;

		saveSettings();
	}

	SubImageRecognition::ImgRecAlgorithm toImgRecAlgorithm() {
		SubImageRecognition::ImgRecAlgorithm alg;
		alg.name = name;
		alg.flags = flags;
		alg.h_max = (int) ((maxThreshold[0] * 255.0 / 179.0) + 0.5);
		alg.h_min = (int) ((minThreshold[0] * 255.0 / 179.0) + 0.5);
		alg.s_max = maxThreshold[1];
		alg.s_min = minThreshold[1];
		alg.v_max = maxThreshold[2];
		alg.v_min = minThreshold[2];
		return alg;
	}

	void print() {
		printf("algorithm:{name: %s, flags: %i, hue: %f-%f, sat: %f-%f, val: %f-%f}\n",
			name.c_str(), flags, minThreshold[0], maxThreshold[0],
			minThreshold[1], maxThreshold[1], minThreshold[2], maxThreshold[2]);
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

class HsvHistogram {
private:
	int histSize[3];
	float hRanges[2];
	float svRanges[2];
	const float* ranges[3];
	int channels[3];

public:
	HsvHistogram() {
		histSize[0] = histSize[1] = histSize[2] = 256;
		hRanges[0] = svRanges[0] = 0.0;
		hRanges[1] = 179.0;
		svRanges[1] = 255.0;
		ranges[0] = hRanges;
		ranges[1] = ranges[2] = svRanges;
		channels[0] = 0;
		channels[1] = 1;
		channels[2] = 2;
	}

	SparseMat getSparseHistogram(const Mat& image) {
		SparseMat hist(3, histSize, CV_8UC3); // resulting histogram
		// TODO: These arguments aren't quite right yet
		calcHist(&image,
				1,           // histogram of only 1 image
				channels,    // the channel used
				cv::Mat(),   // no mask is used
				hist,        // resulting histogram
				histSize,    // number of bins
				ranges);     // pixel value range
		return hist;
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
		Scalar(0, 0, 0),
		Scalar(250, 180, 60),
		ANALYSIS_RECTANGLE,
		2,
		CONFIDENCE_RECTANGLE,
		Scalar(0, 128, 255), // Orange
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
		"buoys/red",
		CAMERA_FORWARD,
		Scalar(135, 0, 75),
		Scalar(255, 255, 110),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 0, 255), // Red
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"buoys/green",
		CAMERA_FORWARD,
		Scalar(0, 0, 0),
		Scalar(130, 245, 125),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 0), // Green
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"buoys/yellow",
		CAMERA_FORWARD,
		Scalar(0, 185, 110),
		Scalar(130, 240, 140),
		ANALYSIS_RECTANGLE,
		1,
		CONFIDENCE_CIRCLE,
		Scalar(0, 255, 255), // Yellow
		ANNOTATION_RADIUS
	));
	algorithms.push_back(Algorithm(
		"obstacle_course",
		CAMERA_FORWARD,
		Scalar(0, 0, 0),
		Scalar(120, 255, 210),
		ANALYSIS_RECTANGLE,
		3,
		CONFIDENCE_RECTANGLE,
		Scalar(255, 0, 0), // Blue
		ANNOTATION_ROTATION
	));
	algorithms.push_back(Algorithm(
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

/*void normalizeValue(Mat& image, Mat& temp) {
	const static int valueOut[] = {2, 0};
	const static int valueIn[] = {0, 2};
	temp.create(image.rows, image.cols, CV_8UC1);
	mixChannels(&image, 1, &temp, 1, valueOut, 1);
	normalize(temp, temp, 0, 255, CV_MINMAX);
	mixChannels(&temp, 1, &image, 1, valueIn, 1);
}*/

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
	return blob0.size() < blob1.size();
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
		Algorithm algorithm = algorithms[i];
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
				vector<BlobAnalysis> analysisList =
						analyzeBlob(algorithm, blobs[j], rotated.image);
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

void thresholdBoxCallback(const SubImageRecognition::ImgRecThreshold& t) {
	printf("[SubImageRecognition] Threshold published to Threshold_Box\n");
	printf("\tname: %s, x1: %i, y1: %i, x2: %i, y2: %i\n",
			t.name.c_str(), t.x1, t.y1, t.x2, t.y2);
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		if (t.name.compare(algorithms[i].name) == 0) {
			// Access correct image
			Mat& segmented = (algorithms[i].camera == CAMERA_FORWARD) ?
				forwardSegmented : downwardSegmented;
			// Ensure points are upper-left and lower-right
			int x1 = min(t.x1, t.x2);
			int y1 = min(t.y1, t.y2);
			int x2 = max(t.x1, t.x2);
			int y2 = max(t.y1, t.y2);
			// Convert format to distance from top-left corner and dimensions
			x1 = t.x1 + (segmented.cols / 2);
			y1 = t.y1 + (segmented.rows / 2);
			x2 = t.x2 - t.x1;
			y2 = t.y2 - t.y1;
			printf("\t[fixed] x1: %i, y1: %i, x2: %i, y2: %i\n", x1, y1, x2, y2);
			// Get region of interest and histogram
			Mat segmentedROI = segmented(Rect(x1, y1, x2, y2));
			HsvHistogram hsvHistogram();
			SparseMat hist = hsvHistogram.getSparseHistogram(segmentedROI);
			// Compute new thresholds from histogram
			//TODO
			break;
		}
	}
}

bool listAlgorithmsCallback(
		SubImageRecognition::ListAlgorithms::Request& req,
		SubImageRecognition::ListAlgorithms::Response& res) {
	printf("[SubImageRecognition] Received call to listAlgorithms()\n");
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		res.algorithms.push_back(algorithms[i].toImgRecAlgorithm());
	}
	return true;
}

bool updateAlgorithmCallback(
		SubImageRecognition::UpdateAlgorithm::Request& req,
		SubImageRecognition::UpdateAlgorithm::Response& res) {
	SubImageRecognition::ImgRecAlgorithm a = req.algorithm;
	printf("[SubImageRecognition] Received call to updateAlgorithm()\n");
	printf("\tname: %s, flags: %u, hue: %u-%u, sat: %u-%u, val: %u-%u\n",
			a.name.c_str(), a.flags, a.h_min, a.h_max,
			a.s_min, a.s_max, a.v_min, a.v_max);
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		if (a.name.compare(algorithms[i].name) == 0) {
			algorithms[i].updateSettings(a);
			res.result = 1;
			break;
		}
	}
	return true;
}

bool switchAlgorithmCallback(
		SubImageRecognition::SwitchAlgorithm::Request& req,
		SubImageRecognition::SwitchAlgorithm::Response& res) {
	printf("[SubImageRecognition] Received call to switchAlgorithm()\n");
	printf("\tname: %s, enabled: %u, publish_threshold: %u\n",
			req.name.c_str(), req.enabled, req.publish_threshold);
	for (unsigned int i = 0; i < algorithms.size(); i++) {
		if (req.name.compare(algorithms[i].name) == 0) {
			if (req.enabled) {
				algorithms[1].flags |= FLAG_ENABLED;
			} else if (algorithms[i].flags & FLAG_ENABLED) {
				algorithms[i].flags -= FLAG_ENABLED;
			}
			if (req.publish_threshold) {
				algorithms[i].flags |= FLAG_PUBLISH_THRESHOLD;
			} else if (algorithms[i].flags & FLAG_PUBLISH_THRESHOLD) {
				algorithms[i].flags -= FLAG_PUBLISH_THRESHOLD;
			}
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

	image_transport::Subscriber forwardSubscriber =
			imageTransport.subscribe("left/image_raw", 1, forwardCallback);

	image_transport::Subscriber downwardSubscriber =
			imageTransport.subscribe("right/image_raw", 1, downwardCallback);

	ros::Subscriber thresholdBoxSubscriber =
			nodeHandle.subscribe("Threshold_Box", 1, thresholdBoxCallback);

	string listAlgorithmsTopic(NAMESPACE_ROOT);
	listAlgorithmsTopic += "list_algorithms";
	ros::ServiceServer listAlgorithmsService = nodeHandle.advertiseService(
			listAlgorithmsTopic, listAlgorithmsCallback);

	string updateAlgorithmTopic(NAMESPACE_ROOT);
	updateAlgorithmTopic += "update_algorithm";
	ros::ServiceServer updateAlgorithmService = nodeHandle.advertiseService(
			updateAlgorithmTopic, updateAlgorithmCallback);

	string switchAlgorithmTopic(NAMESPACE_ROOT);
	switchAlgorithmTopic += "switch_algorithm";
	ros::ServiceServer switchAlgorithmService = nodeHandle.advertiseService(
			switchAlgorithmTopic, switchAlgorithmCallback);

	initAlgorithms();
	ros::spin();
	return 0;
}

