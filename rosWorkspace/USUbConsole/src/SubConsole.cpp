#include <QPoint>
#include <QDesktopWidget>
#include <QImage>
#include <QImageReader>
#include <QPixmap>
#include <QPainter>
#include <QDebug>
#include <QDateTime>
#include <QPalette>
#include <iostream>
#include <math.h>
#include "qwt/qwt_dial_needle.h"
#include <sys/time.h>

#include "SubConsole.hpp"
//#include "USUbConsole/MotorMessage.h"
#include "Robosub/HighLevelControl.h"
#include "SubImageRecognition/UpdateAlgorithm.h"
#include "SubImageRecognition/ListAlgorithms.h"
#include "SubImageRecognition/ImgRecThreshold.h"
#include "ui_SubConsole.h"


#define LEFT_DRIVE_BIT  0x01
#define RIGHT_DRIVE_BIT 0x02
#define FRONT_DEPTH_BIT 0x04
#define REAR_DEPTH_BIT  0x08
#define FRONT_TURN_BIT  0x10
#define REAR_TURN_BIT   0x20


#define X_LINE 0
#define X_DOT 1
#define Y_LINE 2
#define Y_DOT 3
#define Z_LINE 4
#define Z_DOT 5

//Depth Controller Stuff
#define P_LINE 6
#define I_LINE 7
#define E_LINE 8
#define S_LINE 9
#define D_LINE 10
/*
void ActiveLineEdit::focusInEvent(QFocusEvent* e)
{
    if (e->reason() == Qt::MouseFocusReason)
    {
      // The mouse trigerred the event
    }

    // You might also call the parent method.
    QLineEdit::focusInEvent(e);
}*/

/*
 * @brief SubConsole ctor which sets up timers and connect signals/slots
 *
 * @param pParent Poitner to parent widget
 **/
SubConsole::SubConsole(QWidget* pParent)
   : QMainWindow(pParent),
     m_pUi(new Ui::SubConsole),
     m_pJoystickTimer(new QTimer(this)),
     m_pCallbackTimer(new QTimer(this)),
     m_pJoystick(new Joystick()),
     m_nodeHandle(),
     m_motorDriverPublisher(),
     //m_depthPublisher(),
     m_thresholdBoxPublisher(),
     m_torpedoPublisher(),
     m_imageRecService(),
     m_listAlgorithmService(),
     m_imuSubscriber(),
     m_motorControllerTempSubscriber(),
     m_moboTempSubscriber(),
     m_pressureSubscriber(),
     m_depthSubscriber(),
//     m_forwardCameraSubscriber(),
     m_leftCameraSubscriber(),
     m_rightCameraSubscriber(),
     m_downwardCameraSubscriber(),
     m_voltageCurrentSubscriber(),
     m_errorLogSubscriber(),
     m_rawAccelSubscriber(),
     m_lastXAxisValue(0),
     m_lastYAxisValue(0),
     m_lastThrottleValue(0),
     m_lastTwistValue(0),
     m_turnForwardPercentage(1.0),
     m_rightThrustPercentage(1.0),
     m_leftThrustPercentage(1.0),
     m_leftFwdMotorVal(0),
     m_rightFwdMotorVal(0),
     m_frontTurnMotorVal(0),
     m_rearTurnMotorVal(0),
     m_frontDepthMotorVal(0),
     m_rearDepthMotorVal(0),
//     m_pForwardCameraData(NULL),
     m_rollAverage(AVERAGE_LEN),
     m_pitchAverage(AVERAGE_LEN),
     m_yawAverage(AVERAGE_LEN),
     m_depthAverage(AVERAGE_LEN),
     m_battAverage(AVERAGE_LEN),
     m_currAverage(AVERAGE_LEN/2),
     m_pLeftCameraData(NULL),
     m_pRightCameraData(NULL),
     m_pDownwardCameraData(NULL),
     m_downPipEnabled(false),
     m_forwardPipEnabled(false),
     m_pCompass(NULL),
     m_pPitchIndicator(NULL),
     m_pRollIndicator(NULL),
     targetDepth(0),
     depth(0),
     targetHeading(0),
     m_headingActive(false),
     m_pCamThresholdData(NULL),
     m_thresholdActive(false)
//     error(0),
//     speed(0),
//     KI(0.02),
//     KP(0.4),
//     iMax(1),
//     yOld(0),
//     eOld(0)
//     m_pImageRecBoxLabel(NULL),
//     m_pImageRecDownBoxLabel(NULL),

{
   m_pUi->setupUi(this);
   m_pJoystickTimer->setInterval(JOYSTICK_POLL_INTERVAL_MSEC);
   m_pCallbackTimer->setInterval(CALLBACK_HANDLE_INTERVAL_MSEC);

   connect(m_pJoystickTimer, SIGNAL(timeout()), this, SLOT(readJoystickInput()));
   connect(m_pCallbackTimer, SIGNAL(timeout()), this, SLOT(handleRosCallbacks()));
   connect(m_pUi->connectButton, SIGNAL(clicked()), this, SLOT(joyConnect()));
   connect(m_pUi->downPipButton, SIGNAL(clicked()), this, SLOT(toggleDownwardPiP()));
   connect(m_pUi->forwardPipButton, SIGNAL(clicked()), this, SLOT(toggleForwardPiP()));
   connect(m_pUi->thresholdPipButton, SIGNAL(clicked()), this, SLOT(toggleThresholdPip()));
   connect(m_pUi->targetHeadingLineEdit, SIGNAL(returnPressed()), this, SLOT(on_targetHeadingLineEdit_returnPressed()));
   connect(m_pUi->headingEditCheckBox, SIGNAL(stateChanged(int)), this, SLOT(on_headingEditCheckBox_stateChanged(int)));
//   connect(m_pUi->updateButton,SIGNAL(clicked()),this,SLOT(updateGains()));
//   connect(m_pUi->toggleBoxThresholdButton, SIGNAL(clicked()), this, SLOT(toggleBoxThresholding()));
//   connect(m_pUi->enableViewThresholdButton, SIGNAL(clicked()), this, SLOT(enableViewThresholds()));
//   connect(m_pUi->disableViewThresholdButton, SIGNAL(clicked()), this, SLOT(disableViewThresholds()));

   m_pCallbackTimer->start();

   //Attempt to connect to default joystick location
   joyConnect();

   //Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   //Initialize the filter buffers
//   m_rollAverageData = {0};
//   m_yawAverageData = {0};
//   m_pitchAverageData = {0};

//   m_pUi->forwardCameraImageThumb->hide();
   m_pUi->leftCameraImageThumb->hide();
   m_pUi->rightCameraImageThumb->hide();
   m_pUi->downCameraImageThumb->hide();

   //m_motorDriverPublisher = m_nodeHandle.advertise<USUbConsole::MotorMessage>("Motor_Control", 100);
   m_motorDriverPublisher = m_nodeHandle.advertise<Robosub::HighLevelControl>("High_Level_Motion", 100);
   //m_depthPublisher = m_nodeHandle.advertise<std_msgs::Float32>("Target_Depth", 100);
   m_thresholdBoxPublisher = m_nodeHandle.advertise<SubImageRecognition::ImgRecThreshold>("Threshold_Box", 100);
   m_imageRecService = m_nodeHandle.serviceClient<SubImageRecognition::UpdateAlgorithm>("img_rec/update_algorithm");
   m_listAlgorithmService = m_nodeHandle.serviceClient<SubImageRecognition::ListAlgorithms>("img_rec/list_algorithms");
   m_torpedoPublisher = m_nodeHandle.advertise<std_msgs::UInt8>("Torpedo_Launch", 10);

   m_imuSubscriber = m_nodeHandle.subscribe("IMU_Attitude", 100, &SubConsole::imuDataCallback, this);
   m_motorControllerTempSubscriber = m_nodeHandle.subscribe("Controller_Box_Temp", 100, &SubConsole::motorControllerTempCallback, this);
   m_moboTempSubscriber = m_nodeHandle.subscribe("Mobo_Temp", 100, &SubConsole::moboTempCallback, this);
   m_pressureSubscriber = m_nodeHandle.subscribe("Pressure_Data", 100, &SubConsole::pressureDataCallback, this);
   m_depthSubscriber = m_nodeHandle.subscribe("Sub_Depth", 100, &SubConsole::depthCallback, this);
   //m_forwardCameraSubscriber = m_nodeHandle.subscribe("/forward_camera/image_raw/compressed", 100, &SubConsole::forwardCameraCallback, this);
   m_leftCameraSubscriber = m_nodeHandle.subscribe("stereo/left/image_raw/compressed", 100, &SubConsole::leftCameraCallback, this);
   m_rightCameraSubscriber = m_nodeHandle.subscribe("forward_camera/image_raw/compressed", 100, &SubConsole::rightCameraCallback, this);
   m_downwardCameraSubscriber = m_nodeHandle.subscribe("/downward_camera/image_compressed/compressed", 100, &SubConsole::downwardCameraCallback, this);
   m_voltageCurrentSubscriber = m_nodeHandle.subscribe("Computer_Cur_Volt", 100, &SubConsole::currentVoltageCallback, this);
   m_errorLogSubscriber = m_nodeHandle.subscribe("Error_Log", 100, &SubConsole::errorLogCallback, this);
   m_rawAccelSubscriber = m_nodeHandle.subscribe("IMU_Accel_Debug", 100, &SubConsole::rawAccelCallback, this);
   m_motorControlSubscriber = m_nodeHandle.subscribe("Motor_Control", 100, &SubConsole::motorControlCallback, this);

   m_targetDepthSubscriber = m_nodeHandle.subscribe("Target_Depth",100, &SubConsole::targetDepthCallback,this);
   m_motorCurrentSubscriber = m_nodeHandle.subscribe("Motor_Current",100, &SubConsole::motorCurrentCallback,this);
   m_camThresholdSubscriber = m_nodeHandle.subscribe("forward_camera/threshold/compressed", 100, &SubConsole::cameraThresholdCallback, this);
   m_motorStateSubscriber = m_nodeHandle.subscribe("Motor_State",1, &SubConsole::motorStateCallback,this);
   m_targetHeadingSubscriber = m_nodeHandle.subscribe("Target_Heading",100, &SubConsole::targetHeadingCallback,this);
   printf("Finished ROS topic publish and subscription initialization\n");


   //Changed to point an actual QWidget in the GUI
   m_pCompass = m_pUi->yawCompass;//new QwtCompass(m_pUi->imuGroupBox);
   //m_pCompass->resize(80, 80);
   //m_pCompass->move(40, 25);
   m_pCompass->setNeedle(new QwtCompassMagnetNeedle());

   m_pPitchIndicator = m_pUi->pitchIndicator;//new AttitudeIndicator(m_pUi->imuGroupBox);
   //m_pPitchIndicator->resize(80, 80);
   //m_pPitchIndicator->move(165, 25);

   m_pRollIndicator = m_pUi->rollIndicator;//new AttitudeIndicator(m_pUi->imuGroupBox);
   //m_pRollIndicator->resize(80, 80);
   //m_pRollIndicator->move(290, 25);
/*
   m_pImageRecBoxLabel = new ClickableLabel(m_pUi->forwardCameraImage);
   m_pImageRecBoxLabel->resize(478, 638);
   m_pImageRecBoxLabel->move(1, 1);
   connect(m_pImageRecBoxLabel, SIGNAL(clicked()), this, SLOT(imageRecThresholdBoxDrawn()));

   m_pImageRecDownBoxLabel = new ClickableLabel(m_pUi->downwardCameraImage);
   m_pImageRecDownBoxLabel->resize(478, 638);
   m_pImageRecDownBoxLabel->move(1, 1);
   connect(m_pImageRecDownBoxLabel, SIGNAL(clicked()), this, SLOT(imageRecDownThresholdBoxDrawn()));
   */

   //Acceleration Plotting

   m_pUi->accelGraph->addGraph(); //graph for X
   m_pUi->accelGraph->graph(X_LINE)->setPen(QPen(Qt::blue)); //line
   m_pUi->accelGraph->addGraph(); //dot for X
   m_pUi->accelGraph->graph(X_DOT)->setPen(QPen(Qt::blue)); //dot
   m_pUi->accelGraph->graph(X_DOT)->setLineStyle(QCPGraph::lsNone);
   m_pUi->accelGraph->graph(X_DOT)->setScatterStyle(QCP::ssDisc);
   m_pUi->accelGraph->graph(X_DOT)->setName("X");
   m_pUi->accelGraph->legend->removeItem(0);


   m_pUi->accelGraph->addGraph(); //graph for Y
   m_pUi->accelGraph->graph(Y_LINE)->setPen(QPen(Qt::red));
   m_pUi->accelGraph->addGraph(); //dot for Y
   m_pUi->accelGraph->graph(Y_DOT)->setPen(QPen(Qt::red));
   m_pUi->accelGraph->graph(Y_DOT)->setLineStyle(QCPGraph::lsNone);
   m_pUi->accelGraph->graph(Y_DOT)->setScatterStyle(QCP::ssDisc);
   m_pUi->accelGraph->graph(Y_DOT)->setName("Y");
   m_pUi->accelGraph->legend->removeItem(1);


   m_pUi->accelGraph->addGraph(); //graph for Z
   m_pUi->accelGraph->graph(Z_LINE)->setPen(QPen(Qt::green));
   m_pUi->accelGraph->addGraph(); //dot for Z
   m_pUi->accelGraph->graph(Z_DOT)->setPen(QPen(Qt::green));
   m_pUi->accelGraph->graph(Z_DOT)->setLineStyle(QCPGraph::lsNone);
   m_pUi->accelGraph->graph(Z_DOT)->setScatterStyle(QCP::ssDisc);
   m_pUi->accelGraph->graph(Z_DOT)->setName("Z");
   m_pUi->accelGraph->legend->removeItem(2);

   //Depth Controller stuff
   m_pUi->accelGraph->addGraph(); //graph for P
   m_pUi->accelGraph->graph(P_LINE)->setPen(QPen(Qt::cyan));
   m_pUi->accelGraph->graph(P_LINE)->setName("P");

   m_pUi->accelGraph->addGraph(); //graph for I
   m_pUi->accelGraph->graph(I_LINE)->setPen(QPen(Qt::yellow));
   m_pUi->accelGraph->graph(I_LINE)->setName("I");

   m_pUi->accelGraph->addGraph(); //graph for E
   m_pUi->accelGraph->graph(E_LINE)->setPen(QPen(Qt::red));
   m_pUi->accelGraph->graph(E_LINE)->setName("Error");

   m_pUi->accelGraph->addGraph(); //graph for S
   m_pUi->accelGraph->graph(S_LINE)->setPen(QPen(Qt::green));
   m_pUi->accelGraph->graph(S_LINE)->setName("Speed");

   m_pUi->accelGraph->addGraph(); //graph for D
   m_pUi->accelGraph->graph(D_LINE)->setPen(QPen(Qt::blue));
   m_pUi->accelGraph->graph(D_LINE)->setName("Depth");

   //-------------------------------------
   m_pUi->accelGraph->xAxis->setTickLabelType(QCPAxis::ltDateTime);
   m_pUi->accelGraph->xAxis->setDateTimeFormat("hh:mm:ss");
   m_pUi->accelGraph->xAxis->setAutoTickStep(false);
   m_pUi->accelGraph->xAxis->setTickStep(2);
   m_pUi->accelGraph->xAxis->setLabel("Time");
   m_pUi->accelGraph->yAxis->setLabel("g");
   m_pUi->accelGraph->setupFullAxesBox();
   m_pUi->accelGraph->yAxis->setRange(-1.4,1.4);
   m_pUi->accelGraph->xAxis->setAutoTickStep(true);
   m_pUi->accelGraph->legend->setVisible(true);
   m_pUi->accelGraph->legend->setFont(QFont("Helvetica",9));
   m_pUi->accelGraph->legend->setPositionStyle(QCPLegend::psTopLeft);

   //m_pUi->propLineEdit->setText(QString::number(KP));
   //m_pUi->intLineEdit->setText(QString::number(KI));
   //m_pUi->maxLineEdit->setText(QString::number(iMax));
}

/**
 * @brief SubConsole dtor
 **/
SubConsole::~SubConsole(){
   delete m_pCompass;
   delete m_pPitchIndicator;
   delete m_pRollIndicator;
   delete m_pUi;

   m_pJoystickTimer->stop();
   m_pCallbackTimer->stop();
   delete m_pJoystickTimer;
   delete m_pCallbackTimer;
   delete m_pJoystick;
}

/**
 * @brief Attempts to connect to the joystick
 **/
void SubConsole::joyConnect(void)
{
   m_pJoystickTimer->stop();

   if(m_pJoystick->init(m_pUi->devLineEdit->text().toStdString().c_str()) == 0)
   {
       m_pUi->joystickStatusLabel->setText("Connected");
       m_pJoystickTimer->start();
   }
   else
   {
       m_pUi->joystickStatusLabel->setText("Disconnected");
   }
}

/**
 * @brief Periodically reads joystick states if any states have changed publish topic to motor controllers
 **/
void SubConsole::readJoystickInput(void){
   int currentXAxis = m_pJoystick->getAxis(0);
   int currentYAxis = m_pJoystick->getAxis(1);
   int currentTwistAxis = m_pJoystick->getAxis(2);
   int currentThrottleAxis = m_pJoystick->getAxis(3);

   unsigned char motorMask = 0;
   float leftDriveValue = 0;
   float rightDriveValue = 0;
   float frontTurnValue = 0;
   float rearTurnValue = 0;
   float frontDepthValue = 0;
   float rearDepthValue = 0;

   if (m_pJoystick->getButton(2))
   {
       std_msgs::UInt8 torpedoMessage;
       torpedoMessage.data = 0;

       m_torpedoPublisher.publish(torpedoMessage);

       printf("Fire Torpedo 1\n");
       usleep(500000);
   }
   else if (m_pJoystick->getButton(3))
   {
       std_msgs::UInt8 torpedoMessage;
       torpedoMessage.data = 1;

       m_torpedoPublisher.publish(torpedoMessage);

       printf("Fire Torpedo 2\n");
       usleep(500000);
   }

   if(m_lastXAxisValue != currentXAxis)   //Strafe
   {
       //Set the horizontal thrusters to opposite thrust to strafe
       float turnSpeed = (abs(currentXAxis) / (double)JOYSTICK_MAX_VALUE);

       if(currentXAxis > 0)  //Strafe right
       {
          frontTurnValue = turnSpeed * -1;
          rearTurnValue = turnSpeed;//* 0.93;
       }
       else if(currentXAxis < 0)//Strafe left
       {
          frontTurnValue = turnSpeed;// * 0.85;
          rearTurnValue = turnSpeed * -1;
       }

       motorMask |= (MOTOR_FRONT_TURN | MOTOR_REAR_TURN);

       m_lastXAxisValue = currentXAxis;
   }

   if(m_lastYAxisValue != currentYAxis)   //Move forward/backwards
   {
      //Set the forward/reverse thrusters to same value to move forwards or backwards
      float thrusterSpeed = (abs(currentYAxis) / (double)JOYSTICK_MAX_VALUE);

      //A neg number means the stick is pushed forward, if positive we actually want reverse
      if(currentYAxis < 0)
      {
         leftDriveValue = thrusterSpeed * m_leftThrustPercentage;
         rightDriveValue = thrusterSpeed * m_rightThrustPercentage;
      }
      else if(currentYAxis > 0)
      {
          leftDriveValue = thrusterSpeed  * -1 * m_leftThrustPercentage;
          rightDriveValue = thrusterSpeed  * -1 * m_rightThrustPercentage;
      }

      motorMask |= (MOTOR_LEFT_DRIVE | MOTOR_RIGHT_DRIVE);

      m_lastYAxisValue = currentYAxis;
   }

   if(m_lastTwistValue != currentTwistAxis)  //Turn
   {
      //Set the horizontal thrusters to the same direction/velocity to rotate sub
      float thrusterSpeed = (abs(currentTwistAxis) / (double)JOYSTICK_MAX_VALUE);

      if(currentTwistAxis >= 0)  //Turn right, set both thrusters to reverse
      {
          frontTurnValue = thrusterSpeed;// * 0.85;
          rearTurnValue = thrusterSpeed;// * 0.93;

      }
      else if(currentTwistAxis < 0)    //Turn left, set both thrusters to forward
      {
          frontTurnValue = thrusterSpeed * -1;
          rearTurnValue = thrusterSpeed * -1;
      }

      motorMask |= (MOTOR_FRONT_TURN | MOTOR_REAR_TURN);

      m_lastTwistValue = currentTwistAxis;
   }

   if(m_lastThrottleValue != currentThrottleAxis)  //Submerge/surface
   {
      if(m_pUi->depthCheckBox->isChecked())
      {
          m_lastThrottleValue = currentThrottleAxis;
          currentThrottleAxis += JOYSTICK_MAX_VALUE;

          float desiredDepth = MAXIMUM_DEPTH * (currentThrottleAxis / (double)(JOYSTICK_MAX_VALUE * 2));
          Robosub::HighLevelControl depthMsg;

          depthMsg.Direction = "Depth";
          depthMsg.MotionType = "Command";
          depthMsg.Value = desiredDepth;

          //m_pUi->targetDepthLineEdit->setText(QString::number(desiredDepth));

          m_motorDriverPublisher.publish(depthMsg);
          //Depth Controller Stuff
          //targetDepth = desiredDepth;
      }
      else
      {
          //Set the vertical thrusters to the same value to control depth
          float thrusterSpeed = (abs(currentThrottleAxis) / (double)JOYSTICK_MAX_VALUE);

          if(currentThrottleAxis < 0)
          {
             frontDepthValue = thrusterSpeed;
             rearDepthValue = thrusterSpeed;
          }
          else if(currentThrottleAxis > 0)
          {
              frontDepthValue = thrusterSpeed * -1;
              rearDepthValue = thrusterSpeed * -1;
          }

          motorMask |= (MOTOR_FRONT_DEPTH | MOTOR_REAR_DEPTH);

          m_lastThrottleValue = currentThrottleAxis;
      }
   }

   if(motorMask != 0x0)
   {
     sendMotorSpeedMsg(motorMask, leftDriveValue, rightDriveValue, frontDepthValue, rearDepthValue, frontTurnValue, rearTurnValue);
     //printf("Motor: F:%f, D:%f, T:%f", leftDriveValue, frontDepthValue, frontTurnValue);
   }
}

/**
 * @brief Periodically allows ROS time to handle received callbacks
 **/
void SubConsole::handleRosCallbacks(void)
{
    ros::spinOnce();
}

void SubConsole::motorControlCallback(const SubMotorController::MotorMessage::ConstPtr &msg){
    //update the motor graphs values


     m_leftFwdMotorVal      = abs(msg->Left)>60      ?   msg->Left   :   60;
     m_rightFwdMotorVal     = abs(msg->Right)>60     ?   msg->Right  :   60;
     m_frontDepthMotorVal   = abs(msg->FrontDepth)>60?   msg->FrontDepth:60;
     m_rearDepthMotorVal    = abs(msg->RearDepth)>60 ?   msg->RearDepth: 60;
     m_frontTurnMotorVal    = abs(msg->FrontTurn)>60 ?   msg->RearTurn:  60;
     m_rearTurnMotorVal     = abs(msg->RearTurn)>60  ?   msg->RearTurn:  60;

     //printf("%i, %i, %i, %i, %i, %i\n", msg->Left, msg->Right, msg->FrontDepth, msg->RearDepth, msg->FrontTurn, msg->RearTurn);

     m_pUi->leftThrustBar->setValue(abs(m_leftFwdMotorVal));
     if(m_leftFwdMotorVal<0)
         m_pUi->leftThrustBar->setInvertedAppearance(true);
     else
         m_pUi->leftThrustBar->setInvertedAppearance(false);


     m_pUi->rightThrustBar->setValue(abs(m_rightFwdMotorVal));
     if(m_rightFwdMotorVal<0)
         m_pUi->rightThrustBar->setInvertedAppearance(true);
     else
         m_pUi->rightThrustBar->setInvertedAppearance(false);


     m_pUi->frontTurnBar->setValue(abs(m_frontTurnMotorVal));
     if(m_frontTurnMotorVal<0)
         m_pUi->frontTurnBar->setInvertedAppearance(true);
     else
         m_pUi->frontTurnBar->setInvertedAppearance(false);


     m_pUi->rearTurnBar->setValue(abs(m_rearTurnMotorVal));
     if(m_rearTurnMotorVal<0)
         m_pUi->rearTurnBar->setInvertedAppearance(true);
     else
         m_pUi->rearTurnBar->setInvertedAppearance(false);


     m_pUi->frontDepthBar->setValue(abs(m_frontDepthMotorVal));
     if(m_frontDepthMotorVal<0)
         m_pUi->frontDepthBar->setInvertedAppearance(true);
     else
         m_pUi->frontDepthBar->setInvertedAppearance(false);


     m_pUi->rearDepthBar->setValue(abs(m_rearDepthMotorVal));
     if(m_rearDepthMotorVal<0)
         m_pUi->rearDepthBar->setInvertedAppearance(true);
     else
         m_pUi->rearDepthBar->setInvertedAppearance(false);

}

/**
 * @brief Publishes a Motor_Driver message
 *
 * @param motorMask Bit mask with the motors to drive at the specified speed
 **/
void SubConsole::sendMotorSpeedMsg(unsigned char motorMask, float leftDrive, float rightDrive, float frontDepth, float rearDepth, float frontTurn, float rearTurn)
{
    Robosub::HighLevelControl motorMsg;
    motorMsg.MotionType = "Manual";
    if( motorMask & (LEFT_DRIVE_BIT|RIGHT_DRIVE_BIT) ){
            //Send Forward
            motorMsg.Direction = "Forward";
            motorMsg.Value = leftDrive;

            m_motorDriverPublisher.publish(motorMsg);
    }//Forward

    if( motorMask & (FRONT_DEPTH_BIT|REAR_DEPTH_BIT) ){
            //Send Depth
            motorMsg.Direction = "Depth";

            motorMsg.Value = frontDepth;

            m_motorDriverPublisher.publish(motorMsg);
    }//Depth

    if( motorMask & (FRONT_TURN_BIT|REAR_TURN_BIT) ){
        float pre = (frontTurn*rearTurn);
        if(pre>0.0){ //Same signs
            //Send Turn
            motorMsg.Direction = "Turn";

            motorMsg.Value = frontTurn;

            m_motorDriverPublisher.publish(motorMsg);
        }else if(pre<0.0){ //Oposite signs
            //Send Strafe
            motorMsg.Direction = "Strafe";

            motorMsg.Value = frontTurn;

            m_motorDriverPublisher.publish(motorMsg);
        }else{ //pre==0
            motorMsg.Direction = "Turn";

            motorMsg.Value = 0;

            m_motorDriverPublisher.publish(motorMsg);

            motorMsg.Direction = "Strafe";
            m_motorDriverPublisher.publish(motorMsg);
        }
    }//Turn
}



/**
 * @brief ROS callback for IMU_Attitude subscription
 *
 * @param msg The received message
 **/
void SubConsole::imuDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   m_yawAverage.Update(msg->data[2]);
   m_pitchAverage.Update(msg->data[1]);
   m_rollAverage.Update(msg->data[0]);

   m_pUi->yawLineEdit->setText(QString::number(m_yawAverage.Value()));
   m_pCompass->setValue(m_yawAverage.Value());

   m_pUi->pitchLineEdit->setText(QString::number(m_pitchAverage.Value()));
   m_pPitchIndicator->setGradient(m_pitchAverage.Value()/90.0);

   m_pUi->rollLineEdit->setText(QString::number(m_rollAverage.Value()));
   m_pRollIndicator->setAngle(m_rollAverage.Value());

   //Add heding plots here
}

/**
 * @brief ROS callback for Controller_Box_Temp subscription
 *
 * @param msg The received message
 **/
void SubConsole::motorControllerTempCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   m_pUi->motorController1TempLineEdit->setText(QString::number((int)msg->data[0]));
   m_pUi->motorController2TempLineEdit->setText(QString::number((int)msg->data[1]));
   m_pUi->motorController3TempLineEdit->setText(QString::number((int)msg->data[2]));
}

/**
 * @brief ROS callback for Mobo_Temp subscription
 *
 * @param msg The received message
 **/
void SubConsole::moboTempCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   m_pUi->moboTempLineEdit->setText(QString::number(msg->data[0]));
   m_pUi->moboTemp2LineEdit->setText(QString::number(msg->data[1]));
}

/**
 * @brief ROS callback for Pressure_Data subscription
 *
 * @param msg The received message
 **/
void SubConsole::pressureDataCallback(const std_msgs::Float32::ConstPtr& msg)
{
   m_pUi->ticksDepthLineEdit->setText(QString::number(msg->data));
}

/**
 * @brief ROS callback for Sub_Depth subscription
 *
 * @param msg The received message
 **/
void SubConsole::depthCallback(const std_msgs::Float32::ConstPtr& msg)
{
   m_depthAverage.Update(msg->data);
   m_pUi->depthLineEdit->setText(QString::number(m_depthAverage.Value()));

   //Added for Depth Controller gain adjustment
    depth = m_depthAverage.Value();//msg->data;
   float error = depth - targetDepth;

 #if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
   double key = 0;
 #else
   double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
 #endif
   static double lastPointKey = 0;
   if (key-lastPointKey > 0.01) // at most add point every 10 ms
   {

     // add data to lines:
     //m_pUi->accelGraph->graph(P_LINE)->addData(key, p);
     //m_pUi->accelGraph->graph(I_LINE)->addData(key, y);
     m_pUi->accelGraph->graph(E_LINE)->addData(key, error);
     //m_pUi->accelGraph->graph(S_LINE)->addData(key, speed);
     m_pUi->accelGraph->graph(D_LINE)->addData(key, depth/10.0);

     // remove data of lines that's outside visible range:
     //m_pUi->accelGraph->graph(P_LINE)->removeDataBefore(key-8);
     //m_pUi->accelGraph->graph(I_LINE)->removeDataBefore(key-8);
     m_pUi->accelGraph->graph(E_LINE)->removeDataBefore(key-8);
     //m_pUi->accelGraph->graph(S_LINE)->removeDataBefore(key-8);
     m_pUi->accelGraph->graph(D_LINE)->removeDataBefore(key-8);
     // rescale value (vertical) axis to fit the current data:
     //m_pUi->accelGraph->graph(P_LINE)->rescaleValueAxis();
     //m_pUi->accelGraph->graph(I_LINE)->rescaleValueAxis();
     m_pUi->accelGraph->graph(E_LINE)->rescaleValueAxis();
     //m_pUi->accelGraph->graph(S_LINE)->rescaleValueAxis();
     m_pUi->accelGraph->graph(D_LINE)->rescaleValueAxis(true);
     lastPointKey = key;
   }
   // make key axis range scroll with the data (at a constant range size of 8):
   m_pUi->accelGraph->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
   m_pUi->accelGraph->replot();
   //------------------------------------------
}

/**
 * @brief ROS callback for Computer_Cur_Volt subscription
 *
 * @param msg The received message
 **/
void SubConsole::currentVoltageCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   m_battAverage.Update(msg->data[0] - 0.5f); //We seem to be reading .5V more
   m_pUi->voltageLineEdit->setText(QString::number(m_battAverage.Value()));
   m_currAverage.Update(msg->data[1]);//The sensor seems to be uncalibrated
   m_pUi->currentLineEdit->setText(QString::number(m_currAverage.Value()));

   if (msg->data[1]<12.6){
       QPalette p = m_pUi->voltageLineEdit->palette();
       p.setColor(QPalette::Base, QColor(255,45,45)); //red color
   }
}

 void SubConsole::errorLogCallback(const std_msgs::String::ConstPtr& msg)
 {
     QString txt = QString::fromStdString(msg->data);

     if(txt.contains("BUTN", Qt::CaseSensitive)){

         return; //Avoid printing this line
     }
     QDate date = QDate::currentDate();
     QString dateString = date.toString();

     m_pUi->errorLogTextEdit->appendPlainText(dateString + txt);
 }

 void SubConsole::rawAccelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
     //Update and plot the accelerometer vector
     /*
     // calculate two new data points:
   #if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
     double key = 0;
   #else
     double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;
   #endif
     static double lastPointKey = 0;
     if (key-lastPointKey > 0.01) // at most add point every 10 ms
     {

       // add data to lines:
       m_pUi->accelGraph->graph(X_LINE)->addData(key, msg->data[0]);
       m_pUi->accelGraph->graph(Y_LINE)->addData(key, msg->data[1]);
       m_pUi->accelGraph->graph(Z_LINE)->addData(key, msg->data[2]);
       // set data of dots:
       m_pUi->accelGraph->graph(X_DOT)->clearData();
       m_pUi->accelGraph->graph(X_DOT)->addData(key, msg->data[0]);
       m_pUi->accelGraph->graph(Y_DOT)->clearData();
       m_pUi->accelGraph->graph(Y_DOT)->addData(key, msg->data[1]);
       m_pUi->accelGraph->graph(Z_DOT)->clearData();
       m_pUi->accelGraph->graph(Z_DOT)->addData(key, msg->data[2]);
       // remove data of lines that's outside visible range:
       m_pUi->accelGraph->graph(X_LINE)->removeDataBefore(key-8);
       m_pUi->accelGraph->graph(Y_LINE)->removeDataBefore(key-8);
       m_pUi->accelGraph->graph(Z_LINE)->removeDataBefore(key-8);
       // rescale value (vertical) axis to fit the current data:
       m_pUi->accelGraph->graph(X_LINE)->rescaleValueAxis();
       m_pUi->accelGraph->graph(Y_LINE)->rescaleValueAxis();
       m_pUi->accelGraph->graph(Z_LINE)->rescaleValueAxis(true);
       lastPointKey = key;
     }
     // make key axis range scroll with the data (at a constant range size of 8):
     m_pUi->accelGraph->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
     m_pUi->accelGraph->replot();
     */
 }

//Camera methods

///**
// * @brief ROS callback for Forward_Camera subscription
// *
// * @param msg The received message
// **/
 /*
void SubConsole::forwardCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
   QImage image;
   QPixmap pixmap;

   if(m_pForwardCameraData != NULL)
   {
       delete [] m_pForwardCameraData;
   }

   m_pForwardCameraData = new unsigned char[msg->data.size()];
   std::copy(msg->data.begin(), msg->data.end(), m_pForwardCameraData);

   image.loadFromData(m_pForwardCameraData, msg->data.size(), "JPG");
   m_pUi->forwardCameraImage->setPixmap(pixmap.fromImage(image, 0));

   if(m_forwardPipEnabled)
   {
      m_pUi->forwardCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(192), 0));
   }
}
*/

 /**
  * @brief ROS callback for Left_Camera subscription
  *
  * @param msg The received message
  **/

 void SubConsole::leftCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    if(m_thresholdActive) //Avoid running if not needed
        return;
    QImage image;
    QPixmap pixmap;

    if(m_pLeftCameraData != NULL)
    {
        delete [] m_pLeftCameraData;
    }

    m_pLeftCameraData = new unsigned char[msg->data.size()];
    std::copy(msg->data.begin(), msg->data.end(), m_pLeftCameraData);

    image.loadFromData(m_pLeftCameraData, msg->data.size(), "JPG");
    m_pUi->leftCameraImage->setPixmap(pixmap.fromImage(image.scaledToHeight(320), 0));

    if(m_forwardPipEnabled)
    {
       m_pUi->leftCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
    }
 }

 /**
  * @brief ROS callback for Right_Camera subscription
  *
  * @param msg The received message
  **/

 void SubConsole::rightCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
 {

    QImage image;
    QPixmap pixmap;

    if(m_pRightCameraData != NULL)
    {
        delete [] m_pRightCameraData;
    }

    m_pRightCameraData = new unsigned char[msg->data.size()];
    std::copy(msg->data.begin(), msg->data.end(), m_pRightCameraData);

    image.loadFromData(m_pRightCameraData, msg->data.size(), "JPG");
    m_pUi->rightCameraImage->setPixmap(pixmap.fromImage(image.scaledToHeight(320), 0));

    if(m_forwardPipEnabled)
    {
       m_pUi->rightCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
    }
 }


 /**
 * @brief ROS callback for Downward_Camera subscription
 *
 * @param msg The received message
 **/
void SubConsole::downwardCameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    QImage image;
    QPixmap pixmap;

    if(m_pDownwardCameraData != NULL)
    {
        delete [] m_pDownwardCameraData;
    }

    m_pDownwardCameraData = new unsigned char[msg->data.size()];
    std::copy(msg->data.begin(), msg->data.end(), m_pDownwardCameraData);

    image.loadFromData(m_pDownwardCameraData, msg->data.size(), "JPG");
    m_pUi->downwardCameraImage->setPixmap(pixmap.fromImage(image, 0));

    if(m_downPipEnabled)
    {
        m_pUi->downCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(192), 0));
    }
}

/**
 * @brief Reads the camera thresholds image and displays on the right camera box
 * @param msg
 **/
void SubConsole::cameraThresholdCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
    if(!m_thresholdActive) //Avoid running if not needed
        return;
   QImage image;
   QPixmap pixmap;

   if(m_pCamThresholdData != NULL)
   {
       delete [] m_pCamThresholdData;
   }

   m_pCamThresholdData = new unsigned char[msg->data.size()];
   std::copy(msg->data.begin(), msg->data.end(), m_pCamThresholdData);

   image.loadFromData(m_pCamThresholdData, msg->data.size(), "JPG");
   m_pUi->leftCameraImage->setPixmap(pixmap.fromImage(image.scaledToHeight(320), 0));

   if(m_forwardPipEnabled)
   {
      m_pUi->leftCameraImageThumb->setPixmap(pixmap.fromImage(image.scaledToHeight(144), 0));
   }
}

/**
 * @brief Toggles the visibility of the downward picture-in-picture
 **/
void SubConsole::toggleDownwardPiP(void)
{
    if(m_downPipEnabled)
    {
        m_pUi->downCameraImageThumb->hide();
        m_downPipEnabled = false;
    }
    else
    {
        m_pUi->downCameraImageThumb->show();
        m_downPipEnabled = true;
    }
}

/**
 * @brief Toggles the visibility of the forward camera picture-in-picture
 **/
void SubConsole::toggleForwardPiP(void)
{
    if(m_forwardPipEnabled)
    {
        m_pUi->leftCameraImageThumb->hide();
        m_pUi->rightCameraImageThumb->hide();
        m_forwardPipEnabled = false;
    }
    else
    {
        m_pUi->leftCameraImageThumb->show();
        m_pUi->rightCameraImageThumb->show();
        m_forwardPipEnabled = true;
    }
}

void SubConsole::toggleThresholdPip(void)
{
    m_thresholdActive = !m_thresholdActive;
}

/*
//Depth Controller Stuff
void SubConsole::updateGains(void){
    //Read the boxes
     KP = m_pUi->propLineEdit->text().toFloat();
    KI = m_pUi->intLineEdit->text().toFloat();
    int i = system("pkill -f SubDepthController");
    if (i==0)
        system("rosrun SubDepthController SubDepthController ");
    //iMax = m_pUi->maxLineEdit->text().toFloat();


}
*/
void SubConsole::targetDepthCallback(const std_msgs::Float32::ConstPtr& msg){
    m_pUi->targetDepthLineEdit->setText(QString::number(msg->data));

    //Depth Controller Stuff
    targetDepth = msg->data;
}

void SubConsole::targetHeadingCallback(const std_msgs::Float32::ConstPtr& msg){
    if(!m_headingActive)
        m_pUi->targetHeadingLineEdit->setText(QString::number(msg->data));

    //For plotting
    targetHeading  = msg->data;
}

void SubConsole::motorCurrentCallback(const SubMotorController::MotorCurrentMsg::ConstPtr &msg){
    //Update the current boxes as they arrive
    //Forward
    if(msg->motorName =="/dev/controller_drive"){
        if(msg->motorPosition == "Left")
            m_pUi->frontFLineEdit->setText(QString::number(msg->motorCurrent));
        else
            m_pUi->rearFLineEdit->setText(QString::number(msg->motorCurrent));
    } else if (msg->motorName =="/dev/controller_turn"){
        if(msg->motorPosition == "Left")
            m_pUi->frontTLineEdit->setText(QString::number(msg->motorCurrent));
        else
            m_pUi->rearTLineEdit->setText(QString::number(msg->motorCurrent));
    }else if(msg->motorName =="/dev/controller_dive"){
        if(msg->motorPosition == "Left")
            m_pUi->frontDLineEdit->setText(QString::number(msg->motorCurrent));
        else
            m_pUi->rearDLineEdit->setText(QString::number(msg->motorCurrent));
    }else{
        printf("Unknown motor current device reported\n");
    }

}

void SubConsole::motorStateCallback(const std_msgs::UInt8::ConstPtr  &msg){
    bool killed = !msg->data;
    if(killed && !m_pUi->killSwitchCheckBox->isChecked())
        m_pUi->killSwitchCheckBox->setCheckState(Qt::Checked);
    if(!killed && m_pUi->killSwitchCheckBox->isChecked())
        m_pUi->killSwitchCheckBox->setCheckState(Qt::Unchecked);



}


//Image Record Algorithms

/* Unused for the moment - Ivan
void SubConsole::toggleBoxThresholding(void)
{
    if (m_pUi->toggleBoxThresholdButton->text() == "Enable Box Thresholding")
    {
        m_pUi->toggleBoxThresholdButton->setText("Disable Box Thresholding");
        m_pImageRecBoxLabel->rectangleDrawState(true);
        m_pImageRecDownBoxLabel->rectangleDrawState(true);
    }
    else
    {
        m_pUi->toggleBoxThresholdButton->setText("Enable Box Thresholding");
        m_pImageRecBoxLabel->rectangleDrawState(false);
        m_pImageRecDownBoxLabel->rectangleDrawState(false);
        m_pImageRecBoxLabel->clearRectangle();
        m_pImageRecDownBoxLabel->clearRectangle();
    }
}

void SubConsole::enableViewThresholds(void)
{
    SubImageRecognition::UpdateAlgorithm::Request updateAlgorithmService;
    SubImageRecognition::UpdateAlgorithm::Response updateAlgorithmResponse;

    updateAlgorithmService.algorithm.name = getSelectedAlgorithm();
    updateAlgorithmService.algorithm.flags = 2;

    if (m_imageRecService.call(updateAlgorithmService, updateAlgorithmResponse))
    {
        // printf("Update and view thresholds status: %i\n", updateAlgorithmService.response.result);
    }
    else
    {
        printf("Error, failed to send update algorithm service\n");
    }
}

void SubConsole::disableViewThresholds(void)
{
    SubImageRecognition::UpdateAlgorithm::Request updateAlgorithmService;
    SubImageRecognition::UpdateAlgorithm::Response updateAlgorithmResponse;

    updateAlgorithmService.algorithm.name = getSelectedAlgorithm();
    updateAlgorithmService.algorithm.flags = 1;

    if (m_imageRecService.call(updateAlgorithmService, updateAlgorithmResponse))
    {
        // printf("Update and view thresholds status: %i\n", updateAlgorithmService.response.result);
    }
    else
    {
        printf("Error, failed to send update algorithm service\n");
    }
}

std::string SubConsole::getSelectedAlgorithm(void)
{
    std::string selected = "";

    if (m_pUi->algorithmComboBox->currentText() == "Red Buoy")
    {
        selected = "buoys/red";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Green Buoy")
    {
        selected = "buoys/green";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Yellow Buoy")
    {
        selected = "buoys/yellow";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Gate")
    {
        selected = "gate";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Obstacle Course")
    {
        selected = "obstacle_course";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Torpedo Target")
    {
        selected = "torpedo";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Path")
    {
        selected = "paths";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Obstacle Down")
    {
        selected = "obstacle_course_downward";
    }
    else if (m_pUi->algorithmComboBox->currentText() == "Gate Distant")
    {
        selected = "gate_distant";
    }

    return selected;
}

void SubConsole::imageRecThresholdBoxDrawn(void)
{
    SubImageRecognition::ImgRecThreshold thresholdMsg;

    thresholdMsg.name = getSelectedAlgorithm();
    thresholdMsg.x1 = m_pImageRecBoxLabel->getX1();
    thresholdMsg.y1 = m_pImageRecBoxLabel->getY1();
    thresholdMsg.x2 = m_pImageRecBoxLabel->getX2();
    thresholdMsg.y2 = m_pImageRecBoxLabel->getY2();

    if(thresholdMsg.name != "")
    {
        m_thresholdBoxPublisher.publish(thresholdMsg);
        printf("Publishing threshold message for %s\n", thresholdMsg.name.c_str());
    }
}

void SubConsole::imageRecDownThresholdBoxDrawn(void)
{
    SubImageRecognition::ImgRecThreshold thresholdMsg;

    thresholdMsg.name = getSelectedAlgorithm();
    thresholdMsg.x1 = m_pImageRecDownBoxLabel->getX1();
    thresholdMsg.y1 = m_pImageRecDownBoxLabel->getY1();
    thresholdMsg.x2 = m_pImageRecDownBoxLabel->getX2();
    thresholdMsg.y2 = m_pImageRecDownBoxLabel->getY2();

    if(thresholdMsg.name != "")
    {
        m_thresholdBoxPublisher.publish(thresholdMsg);
    }
}
*/

void SubConsole::on_targetHeadingLineEdit_returnPressed()
{
    Robosub::HighLevelControl msg;

    msg.Direction = "Turn";
    msg.MotionType = "Command";
    msg.Value = m_pUi->targetHeadingLineEdit->text().toFloat();

    m_motorDriverPublisher.publish(msg);
    m_headingActive = false;
    m_pUi->headingEditCheckBox->setCheckState(Qt::Unchecked);

}


void SubConsole::on_headingEditCheckBox_stateChanged(int arg1)
{
    if(arg1 == Qt::Checked)
        m_headingActive = true;
}
