#include <ros/ros.h>
#include "Robosub/ModuleEnableMsg.h"
#include "SubImageRecognition/ImgRecObject.h"
#include "Robosub/HighLevelControl.h"

class GateTask
{
  public:
    GateTask();
    ~GateTask();

    void run();

    void gateCallback(const SubImageRecognition::ImgRecObject& msg);
    void moduleEnableCallback(const Robosub::ModuleEnableMsg& msg);
    void publishMotor(std::string direction, std::string motion, float value);
  private:
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_gateSubscriber;
    ros::Subscriber m_taskStateSubscriber;
    ros::Publisher m_centerPointPublisher;
    ros::Publisher m_taskCompletePublisher;
    ros::Publisher m_highLevelMotorPublisher;
    bool m_isEnabled;
    bool m_identifiedCenter;
    bool m_haveBothLegs;
    SubImageRecognition::ImgRecObject m_center;
    SubImageRecognition::ImgRecObject m_zeroth;
    int m_lastId;


    float calculateDistanceFromCenter(float centerDir, float width);
    float getPixelsPerInch(float curWidthPixels, float expectedWidthInches);
    float getDistance(float curObjSize, float actualObjSize);
    void reportSuccess(bool success);

};
