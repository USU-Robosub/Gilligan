#include <QPoint>
#include <QDesktopWidget>
#include <QDebug>
#include <QPen>
#include <iostream>
#include <math.h>
#include <vector>

#include "ImuTuner.hpp"
#include "ui_ImuTuner.h"
#include "qwt/qwt_legend.h"
#include "qwt/qwt_plot_curve.h"

/**
 * @brief ImuTuner ctor
 *
 * @param pParent Poitner to parent widget
 **/
ImuTuner::ImuTuner(QWidget* pParent)
   : QMainWindow(pParent),
     m_pUi(new Ui::ImuTuner),
     m_nodeHandle(),
     m_magDebugSubscriber(),
     m_accelDebugSubscriber(),
     m_pCallbackTimer(new QTimer(this)),
     m_pMagPlotOne(NULL),
     m_pMagResidual0Curve(new QwtPlotCurve("Mag Residual[0]")),
     m_p3Pr0Curve(new QwtPlotCurve("3*sqrt(Pr[0])")),
     m_pNeg3Pr0Curve(new QwtPlotCurve("-3*sqrt(Pr[0])")),
     m_debugMagSamples(601),
     m_magResidual0(601),
     m_3Pr0(601),
     m_neg3Pr0(601),
     m_pMagPlotTwo(NULL),
     m_pMagResidual1Curve(new QwtPlotCurve("Mag Residual[1]")),
     m_p3Pr3Curve(new QwtPlotCurve("3*sqrt(Pr[3])")),
     m_pNeg3Pr3Curve(new QwtPlotCurve("-3*sqrt(Pr[3])")),
     m_magResidual1(601),
     m_3Pr3(601),
     m_neg3Pr3(601),
     m_pMagPlotThree(NULL),
     m_pMagResidual2Curve(new QwtPlotCurve("Mag Residual[2]")),
     m_p3Pr5Curve(new QwtPlotCurve("3*sqrt(Pr[5])")),
     m_pNeg3Pr5Curve(new QwtPlotCurve("-3*sqrt(Pr[5])")),
     m_magResidual2(601),
     m_3Pr5(601),
     m_neg3Pr5(601),
     m_pAccelPlotOne(NULL),
     m_pAccelResidual0Curve(new QwtPlotCurve("Accel Residual[0]")),
     m_pAccel3Pr0Curve(new QwtPlotCurve("3*sqrt(Pr[0])")),
     m_pAccelNeg3Pr0Curve(new QwtPlotCurve("-3*sqrt(Pr[0])")),
     m_debugAccelSamples(601),
     m_accelResidual0(601),
     m_accel3Pr0(601),
     m_accelNeg3Pr0(601),
     m_pAccelPlotTwo(NULL),
     m_pAccelResidual1Curve(new QwtPlotCurve("Accel Residual[1]")),
     m_pAccel3Pr3Curve(new QwtPlotCurve("3*sqrt(Pr[3])")),
     m_pAccelNeg3Pr3Curve(new QwtPlotCurve("-3*sqrt(Pr[3])")),
     m_accelResidual1(601),
     m_accel3Pr3(601),
     m_accelNeg3Pr3(601),
     m_pAccelPlotThree(NULL),
     m_pAccelResidual2Curve(new QwtPlotCurve("Accel Residual[2]")),
     m_pAccel3Pr5Curve(new QwtPlotCurve("3*sqrt(Pr[5])")),
     m_pAccelNeg3Pr5Curve(new QwtPlotCurve("-3*sqrt(Pr[5])")),
     m_accelResidual2(601),
     m_accel3Pr5(601),
     m_accelNeg3Pr5(601),
     m_magSampleCount(0),
     m_accelSampleCount(0)
{
   m_pUi->setupUi(this);
   m_pCallbackTimer->setInterval(20);

   connect(m_pCallbackTimer, SIGNAL(timeout()), this, SLOT(handleRosCallbacks()));

   m_pCallbackTimer->start();

   QPen residualPen;
   residualPen.setColor(Qt::darkBlue);
   residualPen.setWidth(1);

   QPen limitsPen;
   limitsPen.setColor(Qt::darkRed);
   limitsPen.setWidth(1);

   m_pMagPlotOne = new QwtPlot(this);
   m_pMagResidual0Curve->setPen(residualPen);
   m_p3Pr0Curve->setPen(limitsPen);
   m_pNeg3Pr0Curve->setPen(limitsPen);
   setupPlot(m_pMagPlotOne, "Mag Plot #1");

   m_pMagPlotTwo = new QwtPlot(this);
   m_pMagResidual1Curve->setPen(residualPen);
   m_p3Pr3Curve->setPen(limitsPen);
   m_pNeg3Pr3Curve->setPen(limitsPen);
   setupPlot(m_pMagPlotTwo, "Mag Plot #2");
   m_pMagPlotTwo->move(525, 0);

   m_pMagPlotThree = new QwtPlot(this);
   m_pMagResidual2Curve->setPen(residualPen);
   m_p3Pr5Curve->setPen(limitsPen);
   m_pNeg3Pr5Curve->setPen(limitsPen);
   setupPlot(m_pMagPlotThree, "Mag Plot #3");
   m_pMagPlotThree->move(1050, 0);

   m_pAccelPlotOne = new QwtPlot(this);
   m_pAccelResidual0Curve->setPen(residualPen);
   m_pAccel3Pr0Curve->setPen(limitsPen);
   m_pAccelNeg3Pr0Curve->setPen(limitsPen);
   setupPlot(m_pAccelPlotOne, "Accel Plot #1");
   m_pAccelPlotOne->move(0, 415);

   m_pAccelPlotTwo = new QwtPlot(this);
   m_pAccelResidual1Curve->setPen(residualPen);
   m_pAccel3Pr3Curve->setPen(limitsPen);
   m_pAccelNeg3Pr3Curve->setPen(limitsPen);
   setupPlot(m_pAccelPlotTwo, "Accel Plot #2");
   m_pAccelPlotTwo->move(525, 415);

   m_pAccelPlotThree = new QwtPlot(this);
   m_pAccelResidual2Curve->setPen(residualPen);
   m_pAccel3Pr5Curve->setPen(limitsPen);
   m_pAccelNeg3Pr5Curve->setPen(limitsPen);
   setupPlot(m_pAccelPlotThree, "Accel Plot #3");
   m_pAccelPlotThree->move(1050, 415);

   // Center window on screen
   move(qApp->desktop()->availableGeometry(this).center()-rect().center());

   m_magDebugSubscriber = m_nodeHandle.subscribe("Mag_Debug", 100, &ImuTuner::magDebugCallback, this);
   m_accelDebugSubscriber = m_nodeHandle.subscribe("Accel_Debug", 100, &ImuTuner::accelDebugCallback, this);

   printf("Finished ROS topic publish and subscription initialization\n");
}

/**
 * @brief ImuTuner dtor
 **/
ImuTuner::~ImuTuner()
{
   delete m_pUi;
   m_pCallbackTimer->stop();
   delete m_pCallbackTimer;
}

void ImuTuner::setupPlot(QwtPlot* pPlot, const char* plotTitle)
{
    pPlot->setTitle(plotTitle);
    pPlot->setCanvasBackground(QColor(Qt::white));

    pPlot->setAutoReplot(true);

    // legend
    QwtLegend* pLegend = new QwtLegend();
    pLegend->setFrameStyle(QFrame::Box|QFrame::Sunken);
    pPlot->insertLegend(pLegend, QwtPlot::BottomLegend);

    // axis
    pPlot->setAxisTitle(QwtPlot::xBottom, "Samples");
    pPlot->setAxisTitle(QwtPlot::yLeft, "Values");
    pPlot->setAxisScale(QwtPlot::yLeft, -0.4, 0.4);

    pPlot->setGeometry(0,0,500,400);
}

/**
 * @brief Periodically allows ROS time to handle received callbacks
 **/
void ImuTuner::handleRosCallbacks(void)
{
    ros::spinOnce();
}

/**
 * @brief ROS callback for Mag_Debug subscription
 *
 * @param msg The received message
 **/
void ImuTuner::magDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if(m_debugMagSamples.count() > 600)
    {
        m_debugMagSamples.pop_front();

        m_magResidual0.pop_front();
        m_3Pr0.pop_front();
        m_neg3Pr0.pop_front();

        m_magResidual1.pop_front();
        m_3Pr3.pop_front();
        m_neg3Pr3.pop_front();

        m_magResidual2.pop_front();
        m_3Pr5.pop_front();
        m_neg3Pr5.pop_front();
    }

    m_debugMagSamples.push_back(m_magSampleCount++);

    // Plot 1
    m_magResidual0.push_back(msg->data[0]);
    m_3Pr0.push_back(3.0*sqrt(msg->data[3]));
    m_neg3Pr0.push_back(-3.0*sqrt(msg->data[3]));

    m_pMagResidual0Curve->setSamples(m_debugMagSamples, m_magResidual0);
    m_p3Pr0Curve->setSamples(m_debugMagSamples, m_3Pr0);
    m_pNeg3Pr0Curve->setSamples(m_debugMagSamples, m_neg3Pr0);

    m_pMagResidual0Curve->attach(m_pMagPlotOne);
    m_p3Pr0Curve->attach(m_pMagPlotOne);
    m_pNeg3Pr0Curve->attach(m_pMagPlotOne);

    m_pMagPlotOne->replot();

    // Plot 2
    m_magResidual1.push_back(msg->data[1]);
    m_3Pr3.push_back(3.0*sqrt(msg->data[4]));
    m_neg3Pr3.push_back(-3.0*sqrt(msg->data[4]));

    m_pMagResidual1Curve->setSamples(m_debugMagSamples, m_magResidual1);
    m_p3Pr3Curve->setSamples(m_debugMagSamples, m_3Pr3);
    m_pNeg3Pr3Curve->setSamples(m_debugMagSamples, m_neg3Pr3);

    m_pMagResidual1Curve->attach(m_pMagPlotTwo);
    m_p3Pr3Curve->attach(m_pMagPlotTwo);
    m_pNeg3Pr3Curve->attach(m_pMagPlotTwo);

    m_pMagPlotTwo->replot();

    // Plot 3
    m_magResidual2.push_back(msg->data[2]);
    m_3Pr5.push_back(3.0*sqrt(msg->data[5]));
    m_neg3Pr5.push_back(-3.0*sqrt(msg->data[5]));

    m_pMagResidual2Curve->setSamples(m_debugMagSamples, m_magResidual2);
    m_p3Pr5Curve->setSamples(m_debugMagSamples, m_3Pr5);
    m_pNeg3Pr5Curve->setSamples(m_debugMagSamples, m_neg3Pr5);

    m_pMagResidual2Curve->attach(m_pMagPlotThree);
    m_p3Pr5Curve->attach(m_pMagPlotThree);
    m_pNeg3Pr5Curve->attach(m_pMagPlotThree);

    m_pMagPlotThree->replot();
}

/**
 * @brief ROS callback for Accel_Debug subscription
 *
 * @param msg The received message
 **/
void ImuTuner::accelDebugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if(m_debugAccelSamples.count() > 600)
    {
        m_debugAccelSamples.pop_front();

        m_accelResidual0.pop_front();
        m_accel3Pr0.pop_front();
        m_accelNeg3Pr0.pop_front();

        m_accelResidual1.pop_front();
        m_accel3Pr3.pop_front();
        m_accelNeg3Pr3.pop_front();

        m_accelResidual2.pop_front();
        m_accel3Pr5.pop_front();
        m_accelNeg3Pr5.pop_front();
    }

    m_debugAccelSamples.push_back(m_accelSampleCount++);

    // Plot 1
    m_accelResidual0.push_back(msg->data[0]);
    m_accel3Pr0.push_back(3.0*sqrt(msg->data[3]));
    m_accelNeg3Pr0.push_back(-3.0*sqrt(msg->data[3]));

    m_pAccelResidual0Curve->setSamples(m_debugAccelSamples, m_accelResidual0);
    m_pAccel3Pr0Curve->setSamples(m_debugAccelSamples, m_accel3Pr0);
    m_pAccelNeg3Pr0Curve->setSamples(m_debugAccelSamples, m_accelNeg3Pr0);

    m_pAccelResidual0Curve->attach(m_pAccelPlotOne);
    m_pAccel3Pr0Curve->attach(m_pAccelPlotOne);
    m_pAccelNeg3Pr0Curve->attach(m_pAccelPlotOne);

    m_pAccelPlotOne->replot();

    // Plot 2
    m_accelResidual1.push_back(msg->data[1]);
    m_accel3Pr3.push_back(3.0*sqrt(msg->data[4]));
    m_accelNeg3Pr3.push_back(-3.0*sqrt(msg->data[4]));

    m_pAccelResidual1Curve->setSamples(m_debugAccelSamples, m_accelResidual1);
    m_pAccel3Pr3Curve->setSamples(m_debugAccelSamples, m_accel3Pr3);
    m_pAccelNeg3Pr3Curve->setSamples(m_debugAccelSamples, m_accelNeg3Pr3);

    m_pAccelResidual1Curve->attach(m_pAccelPlotTwo);
    m_pAccel3Pr3Curve->attach(m_pAccelPlotTwo);
    m_pAccelNeg3Pr3Curve->attach(m_pAccelPlotTwo);

    m_pAccelPlotTwo->replot();

    // Plot 3
    m_accelResidual2.push_back(msg->data[2]);
    m_accel3Pr5.push_back(3.0*sqrt(msg->data[5]));
    m_accelNeg3Pr5.push_back(-3.0*sqrt(msg->data[5]));

    m_pAccelResidual2Curve->setSamples(m_debugAccelSamples, m_accelResidual2);
    m_pAccel3Pr5Curve->setSamples(m_debugAccelSamples, m_accel3Pr5);
    m_pAccelNeg3Pr5Curve->setSamples(m_debugAccelSamples, m_accelNeg3Pr5);

    m_pAccelResidual2Curve->attach(m_pAccelPlotThree);
    m_pAccel3Pr5Curve->attach(m_pAccelPlotThree);
    m_pAccelNeg3Pr5Curve->attach(m_pAccelPlotThree);

    m_pAccelPlotThree->replot();
}
