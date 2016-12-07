/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the GPLv2 or later, see GPL.TXT
 */


#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QToolBar>
#include <QDebug>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QFont>
#include <QFontMetrics>
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <string>
#include "robot_interfaces.h"
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp::dev;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),  ui(new Ui::MainWindow)
{
    m_robot = 0;
    ui->setupUi(this);

    connect(ui->radioButton,    SIGNAL(clicked()), this, SLOT(Radio1Selected()));
    connect(ui->radioButton_2,  SIGNAL(clicked()), this, SLOT(Radio2Selected()));
    connect(ui->radioButton_3,  SIGNAL(clicked()), this, SLOT(Radio3Selected()));
    connect(ui->radioButton_4,  SIGNAL(clicked()), this, SLOT(Radio4Selected()));
    connect(ui->radioButton_5,  SIGNAL(clicked()), this, SLOT(Radio5Selected()));
    connect(ui->radioButton_6,  SIGNAL(clicked()), this, SLOT(Radio6Selected()));
    connect(ui->radioButton_7,  SIGNAL(clicked()), this, SLOT(Radio7Selected()));
    connect(ui->radioButton_8,  SIGNAL(clicked()), this, SLOT(Radio8Selected()));
    connect(ui->radioButton_9,  SIGNAL(clicked()), this, SLOT(Radio9Selected()));
    connect(ui->radioButton_10, SIGNAL(clicked()), this, SLOT(Radio10Selected()));
    connect(ui->radioButton_11, SIGNAL(clicked()), this, SLOT(Radio11Selected()));
    connect(ui->radioButton_12, SIGNAL(clicked()), this, SLOT(Radio12Selected()));
    connect(ui->radioButton_13, SIGNAL(clicked()), this, SLOT(Radio13Selected()));
    connect(ui->radioButton_14, SIGNAL(clicked()), this, SLOT(Radio14Selected()));
    connect(ui->radioButton_15, SIGNAL(clicked()), this, SLOT(Radio15Selected()));
    connect(ui->radioButton_16, SIGNAL(clicked()), this, SLOT(Radio16Selected()));
    connect(ui->radioButton_17, SIGNAL(clicked()), this, SLOT(Radio17Selected()));
    connect(ui->radioButton_18, SIGNAL(clicked()), this, SLOT(Radio18Selected()));
    connect(ui->radioButton_19, SIGNAL(clicked()), this, SLOT(Radio19Selected()));
    connect(ui->radioButton_20, SIGNAL(clicked()), this, SLOT(Radio20Selected()));
    connect(ui->radioButton_21, SIGNAL(clicked()), this, SLOT(Radio21Selected()));
    connect(ui->radioButton_22, SIGNAL(clicked()), this, SLOT(Radio22Selected()));
    connect(ui->radioButton_23, SIGNAL(clicked()), this, SLOT(Radio23Selected()));
    connect(ui->radioButton_24, SIGNAL(clicked()), this, SLOT(Radio24Selected()));
    connect(ui->radioButton_25, SIGNAL(clicked()), this, SLOT(Radio25Selected()));

    QLocale::setDefault(QLocale::C);
    setWindowTitle("demoForceControl");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::disable_left_arm()
{
    ui->radioButton->setDisabled(true);
    ui->radioButton_2->setDisabled(true);
    ui->radioButton_3->setDisabled(true);
    ui->radioButton_4->setDisabled(true);
    ui->radioButton_5->setDisabled(true);
}

void MainWindow::disable_right_arm()
{
    ui->radioButton_6->setDisabled(true);
    ui->radioButton_7->setDisabled(true);
    ui->radioButton_8->setDisabled(true);
    ui->radioButton_9->setDisabled(true);
    ui->radioButton_10->setDisabled(true);
}

void MainWindow::disable_left_leg()
{
    ui->radioButton_11->setDisabled(true);
    ui->radioButton_12->setDisabled(true);
    ui->radioButton_13->setDisabled(true);
    ui->radioButton_14->setDisabled(true);
    ui->radioButton_15->setDisabled(true);
}

void MainWindow::disable_right_leg()
{
    ui->radioButton_16->setDisabled(true);
    ui->radioButton_17->setDisabled(true);
    ui->radioButton_18->setDisabled(true);
    ui->radioButton_19->setDisabled(true);
    ui->radioButton_20->setDisabled(true);
}

void MainWindow::disable_torso()
{
    ui->radioButton_21->setDisabled(true);
    ui->radioButton_22->setDisabled(true);
    ui->radioButton_23->setDisabled(true);
    ui->radioButton_24->setDisabled(true);
    ui->radioButton_25->setDisabled(true);
}

bool MainWindow::slot_changed_trq(int kk)
{
    if (kk<0 || kk >= 5 || !m_robot->icmd[kk])
    {
        yError() << "Critical fail";
        return false;
    }

    switch (kk)
    {
        case LEFT_ARM:
        case RIGHT_ARM:
            m_robot->iimp[kk]->setImpedance(0, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(1, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(2, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(3, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(4, 0.0, 0.0);
            m_robot->icmd[kk]->setTorqueMode(0);
            m_robot->icmd[kk]->setTorqueMode(1);
            m_robot->icmd[kk]->setTorqueMode(2);
            m_robot->icmd[kk]->setTorqueMode(3);
            m_robot->icmd[kk]->setTorqueMode(4);
            break;
        case LEFT_LEG:
        case RIGHT_LEG:
            m_robot->iimp[kk]->setImpedance(0, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(1, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(2, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(3, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(4, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(5, 0.0, 0.0);
            m_robot->icmd[kk]->setTorqueMode(0);
            m_robot->icmd[kk]->setTorqueMode(1);
            m_robot->icmd[kk]->setTorqueMode(2);
            m_robot->icmd[kk]->setTorqueMode(3);
            m_robot->icmd[kk]->setTorqueMode(4);
            m_robot->icmd[kk]->setTorqueMode(5);
            break;
        case TORSO:
            m_robot->iimp[kk]->setImpedance(0, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(1, 0.0, 0.0);
            m_robot->iimp[kk]->setImpedance(2, 0.1, 0.0);
            m_robot->icmd[kk]->setTorqueMode(0);
            m_robot->icmd[kk]->setTorqueMode(1);
            //m_robot->icmd[kk]->setTorqueMode(2);
            m_robot->icmd[kk]->setPositionMode(2);
            m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            break;
        }
    update_labels();
    return true;
}

bool MainWindow::slot_changed_imp_soft(int kk)
{
    if (kk<0 || kk >= 5 || !m_robot->icmd[kk])
    {
        yError() << "Critical fail";
        return false;
    }

    switch (kk)
    {
        case LEFT_ARM:
        case RIGHT_ARM:
            m_robot->iimp[kk]->setImpedance(0,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(1,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(2,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(3,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(4,0.1,0.0);
            m_robot->icmd[kk]->setPositionMode(0);
            m_robot->icmd[kk]->setPositionMode(1);
            m_robot->icmd[kk]->setPositionMode(2);
            m_robot->icmd[kk]->setPositionMode(3);
            m_robot->icmd[kk]->setPositionMode(4);
            m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
        break;
        case LEFT_LEG:
        case RIGHT_LEG:
            m_robot->iimp[kk]->setImpedance(0,0.3,0.0);
            m_robot->iimp[kk]->setImpedance(1,0.3,0.0);
            m_robot->iimp[kk]->setImpedance(2,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(3,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(4,0.2,0.0);
            m_robot->iimp[kk]->setImpedance(5,0.2,0.0);
            m_robot->icmd[kk]->setPositionMode(0);
            m_robot->icmd[kk]->setPositionMode(1);
            m_robot->icmd[kk]->setPositionMode(2);
            m_robot->icmd[kk]->setPositionMode(3);
            m_robot->icmd[kk]->setPositionMode(4);
            m_robot->icmd[kk]->setPositionMode(5);
            m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
        break;
        case TORSO:
            m_robot->iimp[kk]->setImpedance(0,0.1,0.0);
            m_robot->iimp[kk]->setImpedance(1,0.1,0.0);
            m_robot->iimp[kk]->setImpedance(2,0.1,0.0);
            m_robot->icmd[kk]->setPositionMode(0);
            m_robot->icmd[kk]->setPositionMode(1);
            m_robot->icmd[kk]->setPositionMode(2);
            m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
            m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
        break;
    }

    update_labels();
    return true;
}


bool MainWindow::slot_changed_imp_medium(int kk)
{
    if (kk<0 || kk >= 5 || !m_robot->icmd[kk])
    {
        yError() << "Critical fail";
        return false;
    }

    switch (kk)
    {
    case LEFT_ARM:
    case RIGHT_ARM:
        m_robot->iimp[kk]->setImpedance(0,0.4,0.03);
        m_robot->iimp[kk]->setImpedance(1,0.4,0.03);
        m_robot->iimp[kk]->setImpedance(2,0.4,0.03);
        m_robot->iimp[kk]->setImpedance(3,0.2,0.01);
        m_robot->iimp[kk]->setImpedance(4,0.2,0.00);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
    break;
    case LEFT_LEG:
    case RIGHT_LEG:
        m_robot->iimp[kk]->setImpedance(0,0.6,0.01);
        m_robot->iimp[kk]->setImpedance(1,0.6,0.01);
        m_robot->iimp[kk]->setImpedance(2,0.4,0.01);
        m_robot->iimp[kk]->setImpedance(3,0.4,0.01);
        m_robot->iimp[kk]->setImpedance(4,0.4,0.01);
        m_robot->iimp[kk]->setImpedance(5,0.4,0.01);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->icmd[kk]->setPositionMode(5);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
    break;
    case TORSO:
        m_robot->iimp[kk]->setImpedance(0,0.3,0.0);
        m_robot->iimp[kk]->setImpedance(1,0.3,0.0);
        m_robot->iimp[kk]->setImpedance(2,0.3,0.0);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
    break;
    }

    update_labels();
    return true;
}



bool  MainWindow::slot_changed_imp_hard(int kk)
{
    if (kk<0 || kk >= 5 || !m_robot->icmd[kk])
    {
        yError() << "Critical fail";
        return false;
    }

    switch (kk)
    {
    case LEFT_ARM:
    case RIGHT_ARM:
        m_robot->iimp[kk]->setImpedance(0,0.6,0.06);
        m_robot->iimp[kk]->setImpedance(1,0.6,0.06);
        m_robot->iimp[kk]->setImpedance(2,0.6,0.06);
        m_robot->iimp[kk]->setImpedance(3,0.3,0.02);
        m_robot->iimp[kk]->setImpedance(4,0.2,0.00);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
    break;
    case LEFT_LEG:
    case RIGHT_LEG:
        m_robot->iimp[kk]->setImpedance(0,1.0,0.02);
        m_robot->iimp[kk]->setImpedance(1,1.0,0.02);
        m_robot->iimp[kk]->setImpedance(2,0.7,0.02);
        m_robot->iimp[kk]->setImpedance(3,0.6,0.02);
        m_robot->iimp[kk]->setImpedance(4,0.6,0.02);
        m_robot->iimp[kk]->setImpedance(5,0.6,0.02);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->icmd[kk]->setPositionMode(5);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(5, VOCAB_IM_COMPLIANT);
    break;
    case TORSO:
        m_robot->iimp[kk]->setImpedance(0,0.7,0.015);
        m_robot->iimp[kk]->setImpedance(1,0.7,0.015);
        m_robot->iimp[kk]->setImpedance(2,0.7,0.015);
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_COMPLIANT);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_COMPLIANT);
    break;
    }

    update_labels();
    return true;
}

bool MainWindow::slot_changed_pos(int kk)
{
    if (kk<0 || kk>=5 || !m_robot->icmd[kk])
    {
        yError() << "Critical fail";
        return false;
    }

    switch (kk)
    {
    case LEFT_ARM:
    case RIGHT_ARM:
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_STIFF);
        break;
    case LEFT_LEG:
    case RIGHT_LEG:
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->icmd[kk]->setPositionMode(3);
        m_robot->icmd[kk]->setPositionMode(4);
        m_robot->icmd[kk]->setPositionMode(5);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(3, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(4, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(5, VOCAB_IM_STIFF);
        break;
    case TORSO:
        m_robot->icmd[kk]->setPositionMode(0);
        m_robot->icmd[kk]->setPositionMode(1);
        m_robot->icmd[kk]->setPositionMode(2);
        m_robot->iint[kk]->setInteractionMode(0, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(1, VOCAB_IM_STIFF);
        m_robot->iint[kk]->setInteractionMode(2, VOCAB_IM_STIFF);
        break;
     default:
        yError() << "Critical error";
        return false;
        break;
    }
    
    update_labels();
    return true;
}

bool MainWindow::init(QString robotName, ResourceFinder *finder, robot_interfaces *robot)
{
    m_robot = robot;

    if (m_robot->dd[LEFT_ARM] == 0)
    {
        disable_left_arm();
    }
    else
    {
        ui->radioButton->setChecked(true);
        slot_changed_pos(LEFT_ARM);
    }

    if (m_robot->dd[RIGHT_ARM] == 0)
    {
        disable_right_arm();
    }
    else
    {
        ui->radioButton_6->setChecked(true);
        slot_changed_pos(RIGHT_ARM);
    }

    if (m_robot->dd[LEFT_LEG] == 0)
    {
        disable_left_leg();
    }
    else
    {
        ui->radioButton_11->setChecked(true);
        slot_changed_pos(LEFT_LEG);
    }

    if (m_robot->dd[RIGHT_LEG] == 0)
    {
        disable_right_leg();
    }
    else
    {
        ui->radioButton_16->setChecked(true);
        slot_changed_pos(RIGHT_LEG);
    }

    if (m_robot->dd[TORSO] == 0)
    {
        disable_torso();
    }
    else
    {
        ui->radioButton_21->setChecked(true);
        slot_changed_pos(TORSO);
    }

    return true;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    this->setVisible(false);
    QMainWindow::closeEvent(event);
}

void MainWindow::update_labels()
{
    QLabel* ql[5];
    ql[0] = ui->label1;
    ql[1] = ui->label2;
    ql[2] = ui->label3;
    ql[3] = ui->label4;
    ql[4] = ui->label5;

    char buff[255];
    for (int i = 0; i<5; i++)
    {
        int jmax = 0;
        std::string txt;
        switch (i)
        {
        case LEFT_ARM:
        case RIGHT_ARM:
            jmax = 5;
            break;
        case LEFT_LEG:
        case RIGHT_LEG:
            jmax = 6;
            break;
        case TORSO:
            jmax = 3;
            break;
        }
        for (int j = 0; j<jmax; j++)
        {
            double stiff = 0;
            double damp = 0;
            sprintf(buff, "\n \nJ%d:\n", j); txt += std::string(buff);
            if (m_robot->iimp[i]) m_robot->iimp[i]->getImpedance(j, &stiff, &damp);
            sprintf(buff, "stiff: %3.3f Nm/deg \n", stiff); txt += std::string(buff);
            sprintf(buff, "damp:  %3.3f Nm/(deg/s) \n", damp); txt += std::string(buff);
        }
        ql[i]->setText(QString(txt.c_str()));
    }
}
