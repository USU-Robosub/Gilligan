#include <QtGui/QApplication>
#include "SubConsole.hpp"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QApplication::setStyle("plastique");

    ros::init(argc, argv, "USUbConsole");

    SubConsole w;
    w.show();

    return a.exec();
}
