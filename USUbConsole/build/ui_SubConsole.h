/********************************************************************************
** Form generated from reading UI file 'SubConsole.ui'
**
** Created: Thu Feb 16 19:00:10 2012
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SUBCONSOLE_H
#define UI_SUBCONSOLE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SubConsole
{
public:
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLabel *joystickStatusLabel;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *devLineEdit;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *connectButton;
    QSpacerItem *horizontalSpacer_3;
    QGroupBox *groupBox_2;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_2;
    QLabel *label_4;
    QLabel *label_3;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *yawLineEdit;
    QLineEdit *pitchLineEdit;
    QLineEdit *rollLineEdit;
    QGroupBox *groupBox_3;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_9;
    QVBoxLayout *verticalLayout_4;
    QLineEdit *motorControllerTempLineEdit;
    QLineEdit *motorCaseTempLineEdit;
    QLineEdit *pressureLineEdit;
    QGroupBox *groupBox_4;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_7;
    QLabel *label_8;
    QVBoxLayout *verticalLayout_6;
    QLineEdit *motorStateLineEdit;
    QLineEdit *missionStateLineEdit;
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *forwardCameraImage;
    QWidget *tab_2;
    QLabel *downwardCameraImage;

    void setupUi(QMainWindow *SubConsole)
    {
        if (SubConsole->objectName().isEmpty())
            SubConsole->setObjectName(QString::fromUtf8("SubConsole"));
        SubConsole->setEnabled(true);
        SubConsole->resize(938, 566);
        centralWidget = new QWidget(SubConsole);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(690, 420, 241, 131));
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 30, 171, 94));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        horizontalLayout->addWidget(label);

        joystickStatusLabel = new QLabel(layoutWidget);
        joystickStatusLabel->setObjectName(QString::fromUtf8("joystickStatusLabel"));

        horizontalLayout->addWidget(joystickStatusLabel);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        devLineEdit = new QLineEdit(layoutWidget);
        devLineEdit->setObjectName(QString::fromUtf8("devLineEdit"));

        horizontalLayout_2->addWidget(devLineEdit);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        connectButton = new QPushButton(layoutWidget);
        connectButton->setObjectName(QString::fromUtf8("connectButton"));

        horizontalLayout_6->addWidget(connectButton);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);


        verticalLayout->addLayout(horizontalLayout_6);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(690, 10, 241, 141));
        layoutWidget1 = new QWidget(groupBox_2);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 30, 141, 97));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_2 = new QLabel(layoutWidget1);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_3->addWidget(label_2);

        label_4 = new QLabel(layoutWidget1);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_3->addWidget(label_4);

        label_3 = new QLabel(layoutWidget1);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_3->addWidget(label_3);


        horizontalLayout_3->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        yawLineEdit = new QLineEdit(layoutWidget1);
        yawLineEdit->setObjectName(QString::fromUtf8("yawLineEdit"));
        yawLineEdit->setEnabled(true);

        verticalLayout_2->addWidget(yawLineEdit);

        pitchLineEdit = new QLineEdit(layoutWidget1);
        pitchLineEdit->setObjectName(QString::fromUtf8("pitchLineEdit"));

        verticalLayout_2->addWidget(pitchLineEdit);

        rollLineEdit = new QLineEdit(layoutWidget1);
        rollLineEdit->setObjectName(QString::fromUtf8("rollLineEdit"));

        verticalLayout_2->addWidget(rollLineEdit);


        horizontalLayout_3->addLayout(verticalLayout_2);

        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(690, 160, 241, 131));
        layoutWidget2 = new QWidget(groupBox_3);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(10, 30, 221, 97));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        label_5 = new QLabel(layoutWidget2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(label_5);

        label_6 = new QLabel(layoutWidget2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(label_6);

        label_9 = new QLabel(layoutWidget2);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(label_9);


        horizontalLayout_4->addLayout(verticalLayout_5);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        motorControllerTempLineEdit = new QLineEdit(layoutWidget2);
        motorControllerTempLineEdit->setObjectName(QString::fromUtf8("motorControllerTempLineEdit"));
        motorControllerTempLineEdit->setEnabled(true);

        verticalLayout_4->addWidget(motorControllerTempLineEdit);

        motorCaseTempLineEdit = new QLineEdit(layoutWidget2);
        motorCaseTempLineEdit->setObjectName(QString::fromUtf8("motorCaseTempLineEdit"));

        verticalLayout_4->addWidget(motorCaseTempLineEdit);

        pressureLineEdit = new QLineEdit(layoutWidget2);
        pressureLineEdit->setObjectName(QString::fromUtf8("pressureLineEdit"));

        verticalLayout_4->addWidget(pressureLineEdit);


        horizontalLayout_4->addLayout(verticalLayout_4);

        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(690, 310, 211, 101));
        widget = new QWidget(groupBox_4);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(10, 30, 191, 64));
        horizontalLayout_5 = new QHBoxLayout(widget);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        label_7 = new QLabel(widget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_7->addWidget(label_7);

        label_8 = new QLabel(widget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_7->addWidget(label_8);


        horizontalLayout_5->addLayout(verticalLayout_7);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        motorStateLineEdit = new QLineEdit(widget);
        motorStateLineEdit->setObjectName(QString::fromUtf8("motorStateLineEdit"));
        motorStateLineEdit->setEnabled(true);

        verticalLayout_6->addWidget(motorStateLineEdit);

        missionStateLineEdit = new QLineEdit(widget);
        missionStateLineEdit->setObjectName(QString::fromUtf8("missionStateLineEdit"));
        missionStateLineEdit->setEnabled(true);

        verticalLayout_6->addWidget(missionStateLineEdit);


        horizontalLayout_5->addLayout(verticalLayout_6);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 661, 541));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        forwardCameraImage = new QLabel(tab);
        forwardCameraImage->setObjectName(QString::fromUtf8("forwardCameraImage"));
        forwardCameraImage->setGeometry(QRect(10, 20, 640, 480));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(forwardCameraImage->sizePolicy().hasHeightForWidth());
        forwardCameraImage->setSizePolicy(sizePolicy);
        forwardCameraImage->setFrameShape(QFrame::Box);
        forwardCameraImage->setFrameShadow(QFrame::Plain);
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        downwardCameraImage = new QLabel(tab_2);
        downwardCameraImage->setObjectName(QString::fromUtf8("downwardCameraImage"));
        downwardCameraImage->setGeometry(QRect(10, 20, 640, 480));
        sizePolicy.setHeightForWidth(downwardCameraImage->sizePolicy().hasHeightForWidth());
        downwardCameraImage->setSizePolicy(sizePolicy);
        downwardCameraImage->setFrameShape(QFrame::Box);
        downwardCameraImage->setFrameShadow(QFrame::Plain);
        tabWidget->addTab(tab_2, QString());
        SubConsole->setCentralWidget(centralWidget);

        retranslateUi(SubConsole);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(SubConsole);
    } // setupUi

    void retranslateUi(QMainWindow *SubConsole)
    {
        SubConsole->setWindowTitle(QApplication::translate("SubConsole", "SubConsole", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SubConsole", "Joystick", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SubConsole", "Status:  ", 0, QApplication::UnicodeUTF8));
        joystickStatusLabel->setText(QApplication::translate("SubConsole", "Disconnected", 0, QApplication::UnicodeUTF8));
        devLineEdit->setText(QApplication::translate("SubConsole", "/dev/input/js2", 0, QApplication::UnicodeUTF8));
        connectButton->setText(QApplication::translate("SubConsole", "Connect", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SubConsole", "IMU Data", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SubConsole", "Yaw", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SubConsole", "Pitch", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SubConsole", "Roll", 0, QApplication::UnicodeUTF8));
        yawLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        pitchLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        rollLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SubConsole", "Sensor Readings", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SubConsole", "Motor Controller Temp", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SubConsole", "Motor Case Temp", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("SubConsole", "Pressure", 0, QApplication::UnicodeUTF8));
        motorControllerTempLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        motorCaseTempLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        pressureLineEdit->setText(QApplication::translate("SubConsole", "0.0", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("SubConsole", "Motor State", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SubConsole", "Motor State", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SubConsole", "Mission State", 0, QApplication::UnicodeUTF8));
        motorStateLineEdit->setText(QApplication::translate("SubConsole", "Unknown", 0, QApplication::UnicodeUTF8));
        missionStateLineEdit->setText(QApplication::translate("SubConsole", "Unknown", 0, QApplication::UnicodeUTF8));
        forwardCameraImage->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SubConsole", "Forward Camera", 0, QApplication::UnicodeUTF8));
        downwardCameraImage->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SubConsole", "Downward Camera", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SubConsole: public Ui_SubConsole {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SUBCONSOLE_H
