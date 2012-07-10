#ifndef _IMU_TUNER_HPP
#define _IMU_TUNER_HPP

#include <QMainWindow>
#include <QTimer>
#include <QVector>
#include "qwt/qwt_plot.h"
#include "qwt/qwt_plot_curve.h"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

namespace Ui
{
  class ImuTuner;
}

class ImuTuner : public QMainWindow
{
   Q_OBJECT
public:
   ImuTuner(QWidget* pParent = 0);
   ~ImuTuner();

   void setupPlot(QwtPlot* pPlot, const char* plotTitle);
   void magDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
   void accelDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:
   Ui::ImuTuner* m_pUi;                    //!< Pointer to UI object
   ros::NodeHandle m_nodeHandle;           //!< ROS node handle
   ros::Subscriber m_magDebugSubscriber;   //!< Subscribes to the Mag_Debug topic
   ros::Subscriber m_accelDebugSubscriber; //!< Subscribes to the Accel_Debug topic
   QTimer* m_pCallbackTimer;               //!< Timer used to give processing time to ROS to handle callbacks

   QwtPlot* m_pMagPlotOne;
   QwtPlotCurve* m_pMagResidual0Curve;
   QwtPlotCurve* m_p3Pr0Curve;
   QwtPlotCurve* m_pNeg3Pr0Curve;
   QVector<double> m_debugMagSamples;
   QVector<double> m_magResidual0;
   QVector<double> m_3Pr0;
   QVector<double> m_neg3Pr0;

   QwtPlot* m_pMagPlotTwo;
   QwtPlotCurve* m_pMagResidual1Curve;
   QwtPlotCurve* m_p3Pr3Curve;
   QwtPlotCurve* m_pNeg3Pr3Curve;
   QVector<double> m_magResidual1;
   QVector<double> m_3Pr3;
   QVector<double> m_neg3Pr3;

   QwtPlot* m_pMagPlotThree;
   QwtPlotCurve* m_pMagResidual2Curve;
   QwtPlotCurve* m_p3Pr5Curve;
   QwtPlotCurve* m_pNeg3Pr5Curve;
   QVector<double> m_magResidual2;
   QVector<double> m_3Pr5;
   QVector<double> m_neg3Pr5;

   QwtPlot* m_pAccelPlotOne;
   QwtPlotCurve* m_pAccelResidual0Curve;
   QwtPlotCurve* m_pAccel3Pr0Curve;
   QwtPlotCurve* m_pAccelNeg3Pr0Curve;
   QVector<double> m_debugAccelSamples;
   QVector<double> m_accelResidual0;
   QVector<double> m_accel3Pr0;
   QVector<double> m_accelNeg3Pr0;

   QwtPlot* m_pAccelPlotTwo;
   QwtPlotCurve* m_pAccelResidual1Curve;
   QwtPlotCurve* m_pAccel3Pr3Curve;
   QwtPlotCurve* m_pAccelNeg3Pr3Curve;
   QVector<double> m_accelResidual1;
   QVector<double> m_accel3Pr3;
   QVector<double> m_accelNeg3Pr3;

   QwtPlot* m_pAccelPlotThree;
   QwtPlotCurve* m_pAccelResidual2Curve;
   QwtPlotCurve* m_pAccel3Pr5Curve;
   QwtPlotCurve* m_pAccelNeg3Pr5Curve;
   QVector<double> m_accelResidual2;
   QVector<double> m_accel3Pr5;
   QVector<double> m_accelNeg3Pr5;

   int m_magSampleCount;
   int m_accelSampleCount;

private slots:
   void handleRosCallbacks(void);
};

#endif // _IMU_TUNER_HPP
