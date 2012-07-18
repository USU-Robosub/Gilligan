#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "Robosub/ModuleEnableMsg.h"
#include "Robosub/Point.h"
#include "Robosub/Line.h"

#define OFF 0
#define ON 1
#define MAX_THRESHOLD 0.05 //5%
#define MIN_THRESHOLD 0.01 //1%

class NavigationControl
{
  public:
    NavigationControl();
    ~NavigationControl();

    void run(); //leave empty unless needed
    void EnabledCallback(const Robosub::ModuleEnableMsg& msg);
    void PointCallback(const Robosub::Point& msg);

  private:
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_highLevelMotorPublisher;
    //bool OMODE = OFF;
    bool POINTMODE;
    bool LINEMODE;
    //bool Centered = false;
    bool Begun;
    bool OnLine;
    bool Rotating;

    float start_x;
    float start_y;
    float start_rot;

    float makeVoltage(float percent);
    void setDive(float val);
    void setStraf(float val);
    void setDrive(float val);
    void setTurn(float val);
    bool moveToLine(int x, int y);
    float sanitize(float rot);
    void LineCallback(const Robosub::Line msg);
    void publishMotor(std::string direction, std::string motion, float value);
};
