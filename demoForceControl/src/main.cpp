/*
  * Copyright (C) 2016  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <string>
#include <QApplication>
#include "mainwindow.h"
#include "robot_interfaces.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

int main(int argc, char * argv[])
{
    QApplication a(argc, argv);

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("ERROR: check Yarp network.");
        return 0;
    }

    yarp::os::ResourceFinder* rf = new ResourceFinder;
    rf->setVerbose();
    rf->configure(argc, argv);
    
    robot_interfaces *robot;
    robot = new robot_interfaces();
    std::string robot_name = "icub";

    if (rf->check("robot"))
    {
        robot_name=rf->find("robot").asString();
    }

    if (robot->init(robot_name) == false)
    {
        /*
        yError("Failed to connect to robot");
        delete rf;
        delete robot;
        return 0;
        */
    }
    
    MainWindow w;
    w.init("icub", rf, robot);
    w.show();

    int appRet = a.exec();

    delete rf;
    delete robot;

    return (appRet != 0 ? 1 : 0);
}


