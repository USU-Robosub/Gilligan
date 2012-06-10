#include <QtGui/QApplication>
#include "ImuTuner.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setStyle("plastique");

    ros::init(argc, argv, "ImuTuner");

    ImuTuner w;
    w.show();

    return a.exec();
}
