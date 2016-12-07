/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the GPLv2 or later, see GPL.TXT
 */


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <yarp/os/ResourceFinder.h>

#include <QMainWindow>
#include <QResizeEvent>
#include <QLabel>
#include <QTimer>
#include <QAction>
#include <QMutex>
#include <QTreeWidget>
#include "robot_interfaces.h"

namespace Ui {
class MainWindow;
}

using namespace yarp::os;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    bool init(QString robotName, ResourceFinder *finder, robot_interfaces *robot);
    bool slot_changed_pos(int kk);
    bool slot_changed_trq(int kk);
    bool slot_changed_imp_soft  (int kk);
    bool slot_changed_imp_medium(int kk);
    bool slot_changed_imp_hard(int kk);
    void disable_left_arm();
    void disable_left_leg();
    void disable_right_arm();
    void disable_right_leg();
    void disable_torso();
    
    ~MainWindow();
    void update_labels();

protected:
    void closeEvent(QCloseEvent *event);
private:
    Ui::MainWindow *ui;
    robot_interfaces *m_robot;

private:

private slots:
    void Radio1Selected()  { slot_changed_pos(LEFT_ARM); }
    void Radio2Selected()  { slot_changed_trq(LEFT_ARM); }
    void Radio3Selected()  { slot_changed_imp_soft(LEFT_ARM); }
    void Radio4Selected()  { slot_changed_imp_medium(LEFT_ARM); }
    void Radio5Selected()  { slot_changed_imp_hard(LEFT_ARM); }
    void Radio6Selected()  { slot_changed_pos(RIGHT_ARM); }
    void Radio7Selected()  { slot_changed_trq(RIGHT_ARM); }
    void Radio8Selected()  { slot_changed_imp_soft(RIGHT_ARM); }
    void Radio9Selected()  { slot_changed_imp_medium(RIGHT_ARM); }
    void Radio10Selected() { slot_changed_imp_hard(RIGHT_ARM); }
    void Radio11Selected() { slot_changed_pos(LEFT_LEG); }
    void Radio12Selected() { slot_changed_trq(LEFT_LEG); }
    void Radio13Selected() { slot_changed_imp_soft(LEFT_LEG); }
    void Radio14Selected() { slot_changed_imp_medium(LEFT_LEG); }
    void Radio15Selected() { slot_changed_imp_hard(LEFT_LEG); }
    void Radio16Selected() { slot_changed_pos(RIGHT_LEG); }
    void Radio17Selected() { slot_changed_trq(RIGHT_LEG); }
    void Radio18Selected() { slot_changed_imp_soft(RIGHT_LEG); }
    void Radio19Selected() { slot_changed_imp_medium(RIGHT_LEG); }
    void Radio20Selected() { slot_changed_imp_hard(RIGHT_LEG); }
    void Radio21Selected() { slot_changed_pos(TORSO); }
    void Radio22Selected() { slot_changed_trq(TORSO); }
    void Radio23Selected() { slot_changed_imp_soft(TORSO); }
    void Radio24Selected() { slot_changed_imp_medium(TORSO); }
    void Radio25Selected() { slot_changed_imp_hard(TORSO); }
signals:

};

class ModesTreeWidget : public QTreeWidget
{
    Q_OBJECT

public:
    ModesTreeWidget(QWidget * parent = 0);


};

#endif // MAINWINDOW_H
