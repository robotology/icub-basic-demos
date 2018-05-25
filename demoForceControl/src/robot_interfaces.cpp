/*
  * Copyright (C) 2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "robot_interfaces.h"
#include <string>

using namespace yarp::os;
using namespace yarp::dev;

robot_interfaces::robot_interfaces()
{
    for (int i=0; i<5; i++)
    {
        ipos[i]=0;
        itrq[i]=0;
        iimp[i]=0;
        icmd[i]=0;
        ienc[i]=0;
        ipid[i]=0;
        ivel[i]=0;
        iamp[i]=0;
        iint[i]=0;
        dd[i]=0;
    }
}

robot_interfaces::~robot_interfaces()
{
    for (int i = 0; i < 5; i++)
    {
        if (dd[i] != 0)
        {
            dd[i]->close();
            delete dd[i];
            dd[i] = 0;
        }
    }
}

bool robot_interfaces::init(std::string robot)
{
    std::string part;
    std::string localPort;
    std::string remotePort;
    bool ok = true;

    for (int i=0; i<5; i++)
    {
        switch (i) 
        {
            case LEFT_ARM:     part = "left_arm";   break;
            case RIGHT_ARM:    part = "right_arm";  break;
            case LEFT_LEG:     part = "left_leg";   break;
            case RIGHT_LEG:    part = "right_leg";  break;
            case TORSO:        part = "torso";      break;
        }

        localPort  = "/demoForceControl/" + part;
        remotePort = "/" + robot + "/" + part;
        options[i].put("robot",robot);
        options[i].put("part",part);
        options[i].put("device","remote_controlboard");
        options[i].put("local",localPort);
        options[i].put("remote",remotePort);

        dd[i] = new PolyDriver(options[i]);
        if(!dd[i] || !(dd[i]->isValid()))
        {
            yError("Problems instantiating the device driver %d\n", i);
            delete dd[i];
            dd[i] = 0;
            ok = false;
            continue;
        }

        ok = ok & dd[i]->view(ipos[i]);
        ok = ok & dd[i]->view(itrq[i]);
        ok = ok & dd[i]->view(iimp[i]);
        ok = ok & dd[i]->view(icmd[i]);
        ok = ok & dd[i]->view(ivel[i]);
        ok = ok & dd[i]->view(ienc[i]);
        ok = ok & dd[i]->view(ipid[i]);
        ok = ok & dd[i]->view(iamp[i]);
        ok = ok & dd[i]->view(iint[i]);
    }
    return ok;
}


