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
#include <fstream>
#include <math.h>

#include "SubImageRecognition/ImgRecAlgorithm.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "SubImageRecognition/ImgRecThreshold.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/UpdateAlgorithm.h"
#include "SubImageRecognition/SwitchAlgorithm.h"
#include "DLT.h"

using namespace cv;
using namespace std;

// CONSTANTS

const int SAMPLE_SIZE = 4;
const unsigned int MIN_POINTS = 10;
const float MIN_CONFIDENCE = 0.5;

const char NAMESPACE_ROOT[] = "img_rec/";

const int FLAG_ENABLED = 1;
const int FLAG_PUBLISH_THRESHOLD = 2;

const int CAMERA_FORWARD = 0;
const int CAMERA_DOWNWARD = 1;

const int ANALYSIS_RECTANGLE = 0;

const int CONFIDENCE_RECTANGLE = 0;
const int CONFIDENCE_CIRCLE = 1;

const int ANNOTATION_ROTATION = 0;
const int ANNOTATION_RADIUS = 1;

const int FRAME_MARGIN_OF_ERROR=2;
const int TRACKING_MOVEMENT_TOLERANCE=50;

// DEFINITIONS

typedef vector<Point> Points;

class Object {
public:
	string name;
	int flags;
	int camera;
	int analysisType;
	int maxBlobs;
	int confidenceType;
	Scalar annotationColor;
	int annotationType;
	int enumType;
	ros::Publisher publisher;
	Object(string name, int flags, int camera, int analysisType, int maxBlobs, int confidenceType, Scalar annotationColor, int annotationType, int enumType):
	name(name)
	,flags(flags)
	,camera(camera)
	,analysisType(analysisType)
	,maxBlobs(maxBlobs)
	,confidenceType(confidenceType)
	,annotationColor(annotationColor)
	,annotationType(annotationType)
	,enumType(enumType)
	{
		ros::NodeHandle nodeHandle;
		string topic(NAMESPACE_ROOT);
		topic += name;
		publisher = nodeHandle.advertise<SubImageRecognition::ImgRecObject>(topic, 1);
	};
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

class BlobTrack
{
public:
	int x;
	int y;
	int lifetime;
	int lastSeen;
	int objType;
	BlobTrack(int x, int y, int lastSeen, int objType):
	x(x)
	,y(y)
	,lastSeen(lastSeen)
	,objType(objType)
	,lifetime(0)
	{
	}
};

// GLOBALS  :/  HA HA AH WELL

vector<Object> objects;
vector<BlobTrack> trackBlobs;

int curFrame=0;

int forwardOffset = 0, downwardOffset = 0;
image_transport::Publisher forwardPublisher, downwardPublisher;
cv_bridge::CvImage forwardRotated, downwardRotated;
Mat forwardSegmented, downwardSegmented;
Mat forwardThreshold, downwardThreshold;
DLT* pTree;

// FUNCTIONS

//Need to add enumTypes to Objects
void initObjects() {
		objects.push_back(Object(
				"gate",
				1,
				CAMERA_FORWARD,
				ANALYSIS_RECTANGLE,
				2,
				CONFIDENCE_RECTANGLE,
				Scalar(0, 128, 255), // Orange
				ANNOTATION_ROTATION,
				1
		));
		objects.push_back(Object(
				"buoys/red",
                1,
				CAMERA_FORWARD,
				ANALYSIS_RECTANGLE,
				1,
				CONFIDENCE_CIRCLE,
				Scalar(0, 0, 255), // Red
				ANNOTATION_RADIUS,
				2
		));
		objects.push_back(Object(
				"buoys/green",
                1,
				CAMERA_FORWARD,
				ANALYSIS_RECTANGLE,
				1,
				CONFIDENCE_CIRCLE,
				Scalar(0, 255, 0), // Green
				ANNOTATION_RADIUS,
				3
		));
		objects.push_back(Object(
				"buoys/yellow",
                1,
				CAMERA_FORWARD,
				ANALYSIS_RECTANGLE,
				1,
				CONFIDENCE_CIRCLE,
				Scalar(0, 255, 255), // Yellow
				ANNOTATION_RADIUS,
				4
		));
		objects.push_back(Object(
				"obstacle_course",
                1,
				CAMERA_FORWARD,
				ANALYSIS_RECTANGLE,
				3,
				CONFIDENCE_RECTANGLE,
				Scalar(255, 0, 0), // Blue
				ANNOTATION_ROTATION,
				6
		));
		objects.push_back(Object(
				"paths",
                1,
				CAMERA_DOWNWARD,
				ANALYSIS_RECTANGLE,
				2,
				CONFIDENCE_RECTANGLE,
				Scalar(0, 128, 255), // Orange
				ANNOTATION_ROTATION,
				5
		));
		objects.push_back(Object(
				"obstacle_course_downward",
                1,
				CAMERA_DOWNWARD,
				ANALYSIS_RECTANGLE,
				1,
				CONFIDENCE_RECTANGLE,
				Scalar(255, 0, 0), // Blue
				ANNOTATION_ROTATION,
				7
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
		erode(image, image, elementRect, point, 5);
		dilate(image, image, elementRect, point, 2);
}

Points findBlob(Mat& image, int i, int j, int obj) {
		unsigned int index = 0;
		Points blob;
		Point point(j, i);
		blob.push_back(point);
		image.at<uint8_t>(i, j, 0) = 0;
		while (index < blob.size()) {
				point = blob[index];
				i = point.y;
				j = point.x;
				if (i+SAMPLE_SIZE < image.rows
								&& image.at<uint8_t>(i+SAMPLE_SIZE, j, 0) == obj) {
						blob.push_back(Point(j, i+SAMPLE_SIZE));
						image.at<uint8_t>(i+SAMPLE_SIZE, j, 0) = 0;
				}
				if (i-SAMPLE_SIZE >= 0
								&& image.at<uint8_t>(i-SAMPLE_SIZE, j, 0) == obj) {
						blob.push_back(Point(j, i-SAMPLE_SIZE));
						image.at<uint8_t>(i-SAMPLE_SIZE, j, 0) = 0;
				}
				if (j+SAMPLE_SIZE < image.cols
								&& image.at<uint8_t>(i, j+SAMPLE_SIZE, 0) == obj) {
						blob.push_back(Point(j+SAMPLE_SIZE, i));
						image.at<uint8_t>(i, j+SAMPLE_SIZE, 0) = 0;
				}
				if (j-SAMPLE_SIZE >= 0 &&
								image.at<uint8_t>(i, j-SAMPLE_SIZE, 0) == obj) {
						blob.push_back(Point(j-SAMPLE_SIZE, i));
						image.at<uint8_t>(i, j-SAMPLE_SIZE, 0) = 0;
				}
				index++;
		}
		return blob;
}

bool compareBlobs(Points& blob0, Points& blob1) {
		return blob0.size() < blob1.size();
}

vector<Points> findBlobs(Mat& image,
				const int offset, const unsigned int maxBlobs, int obj) {
		// First get all blobs that are at least the minimum size
		vector<Points> allBlobs;
		for (int i = offset; i < image.rows; i += SAMPLE_SIZE) {
				for (int j = offset; j < image.cols; j += SAMPLE_SIZE) {
						if (image.at<uint8_t>(i, j, 0) == obj) {
								Points blob = findBlob(image, i, j, obj);
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

vector<BlobAnalysis> analyzeBlob(Object& object,
				Points& blob, Mat& image) {
		vector<BlobAnalysis> analysisList;
		RotatedRect rectangle;
		switch (object.analysisType) {
		case ANALYSIS_RECTANGLE:
				analysisList.push_back(BlobAnalysis(blob, minAreaRect(Mat(blob))));
				break;
		}
		return analysisList;
}

float computeConfidence(Object& object, BlobAnalysis& a) {
		// A return value of -1 indicates 'divide by zero' error
		// A return value of -2 indicates 'unknown confidence type' error
		int expectedPoints;
		switch (object.confidenceType) {
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

void annotateImage(Mat& image, Object& object, BlobAnalysis& a) {
		int r, x, y;
		switch (object.annotationType) {
		case ANNOTATION_ROTATION:
				x = (int) (a.height / 2.0 * cos(a.rotation));
				y = (int) (a.height / 2.0 * sin(a.rotation));
				circle(image, Point(a.center_x, a.center_y), 1,
								object.annotationColor, 5, CV_AA);
				line(image, Point(a.center_x, a.center_y),
								Point(a.center_x + x, a.center_y + y),
								object.annotationColor, 1, CV_AA);
				break;
		case ANNOTATION_RADIUS:
				r = (int) ((a.width + a.height) / 4.0);
				circle(image, Point(a.center_x, a.center_y), r,
								object.annotationColor, 2, CV_AA);
				break;
		}
}


//assuming i=y and j=x
void objInRange(const Mat& segmented, Mat& threshold, const int offset)
{	if (threshold.total() == 0) {
		threshold.create(segmented.rows, segmented.cols, CV_8U);
	}
	for (unsigned int i=0;i<objects.size();++i){
		Object object=objects[i];
		for (int i = offset; i < threshold.rows; i += SAMPLE_SIZE) {
			for (int j = offset; j < threshold.cols; j += SAMPLE_SIZE) {
				Sample sample;
				Vec3b hsv = segmented.at<cv::Vec3b>(i, j);
				sample.type=0;
				sample.iAttr[0]=i;
				sample.iAttr[1]=j;
				sample.iAttr[2]=hsv[0];
				sample.iAttr[3]=hsv[1];
				sample.iAttr[4]=hsv[2];
				if(pTree->Classify(sample)==object.enumType)
				{
					threshold.at<uint8_t>(i,j,0)=object.enumType;
				}
			}
		}
	}
}

bool inCircle(BlobAnalysis analysis, BlobTrack track)
{
	int deltaX=analysis.center_x-track.x;
	int deltaY=analysis.center_y-track.y;
	return (sqrt((float)deltaX * deltaX + deltaY * deltaY)) <= TRACKING_MOVEMENT_TOLERANCE;
}

bool trackBlob(BlobAnalysis analysis, int type)
{
	for(int i=0;i<trackBlobs.size();++i)
	{
		if(type==trackBlobs[i].objType)
		{
			if(inCircle(analysis, trackBlobs[i]))
			{
				trackBlobs[i].x=analysis.center_x;
				trackBlobs[i].y=analysis.center_y;
				trackBlobs[i].lastSeen=curFrame;
				++trackBlobs[i].lifetime;
				return FRAME_MARGIN_OF_ERROR>=trackBlobs[i].lifetime;
			}
		}
	}
	trackBlobs.push_back(BlobTrack(analysis.center_x, analysis.center_y, curFrame, type));
	return false;
}

void updateBlobTracking()
{
	for(int i=0;i<trackBlobs.size();++i)
	{
		if(trackBlobs[i].lastSeen < (curFrame - FRAME_MARGIN_OF_ERROR))
		{
			trackBlobs.erase(trackBlobs.begin()+i);
			--i;
		}
	}
	++curFrame;
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

		// Iterate through all objects
		objInRange(segmented, threshold, offset);
		for (unsigned int i = 0; i < objects.size(); i++) {
				Object object = objects[i];
				// Run applicable algorithms
				if ((object.flags & FLAG_ENABLED) && object.camera == camera) {
						reduceNoise(threshold);
						vector<Points> blobs = findBlobs(
										threshold, offset, object.maxBlobs, object.enumType);
						if (object.flags & FLAG_PUBLISH_THRESHOLD) {
								cv_bridge::CvImage temp;
								temp.encoding = "mono8";
								temp.image = threshold;
								publisher.publish(temp.toImageMsg());
						}
						// Iterate through all blobs
						for (unsigned int j = 0; j < blobs.size(); j++) {
								vector<BlobAnalysis> analysisList =
												analyzeBlob(object, blobs[j], rotated.image);
								// Iterate through all blob analysis objects
								ros::Time time = ros::Time::now();
								for (unsigned int k = 0; k < analysisList.size(); k++) {
										BlobAnalysis analysis = analysisList[k];

										//if (trackBlob(analysis, object.enumType)) {
                                        //FIXIT: just testing this out 7/17/13
                                        if(true) {
												// Publish information
												SubImageRecognition::ImgRecObject msg;
												msg.stamp = time;
												msg.id = k;
												msg.center_x = analysis.center_x - rotated.image.cols / 2;
												msg.center_y = rotated.image.rows / 2 - analysis.center_y;
												msg.rotation = (analysis.rotation + M_PI / 2.0) * 180.0 / M_PI;
												msg.width = analysis.width;
												msg.height = analysis.height;
												msg.confidence = computeConfidence(object, analysis);
												object.publisher.publish(msg);
												// Annotate image
												annotateImage(rotated.image, object, analysis);
										}

								}
						}
				}
		}

		// Publish annotated image
		publisher.publish(rotated.toImageMsg());
		updateBlobTracking();
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

int main(int argc, char **argv) {
	fstream file("/opt/robosub/rosWorkspace/SubImageRecognition/tree.tree");
	pTree = new DLT(file);

	ros::init(argc, argv, "ImageRecognition");
	ros::NodeHandle nodeHandle;
	image_transport::ImageTransport imageTransport(nodeHandle);

	forwardPublisher = imageTransport.advertise("forward_camera/image_raw", 1);
	downwardPublisher = imageTransport.advertise("downward_camera/image_raw", 1);

	image_transport::Subscriber forwardSubscriber = imageTransport.subscribe("left/image_raw", 1, forwardCallback);
	image_transport::Subscriber downwardSubscriber = imageTransport.subscribe("image_raw", 1, downwardCallback);

	initObjects();
	ros::spin();
	return 0;
}
