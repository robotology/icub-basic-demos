/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Lorenzo Natale
 * email:  lorenzo.natale@iit.it
 * website: www.robotcub.org
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

/**
@ingroup demo_modules
\defgroup src_demoYoga demoYoga

Demo for moving the robot joints in a coordinated fashion.

\section intro_sec Description
This modules move the robot into a series of positions. Repeat forever or until quit
(whichever comes first).

\section lib_sec Libraries
YARP libraries.

\section parameters_sec Parameters
--robot robotname
--positions filename.txt

Optionally:
--verbose: print sequence of positions and speeds

\section portsa_sec Ports Accessed
Access robotInterface ports.

\section portsc_sec Ports Created
Creates remote_controlboard for each of the robot parts.

\section conf_file_sec Configuration Files
The module requires a sequence of positions:
--positions.

The file consists in a few sections:
\code
time 2
\endcode
specifies the time between two movements.

Follows a file for each robot parts. The file contains the list of positions to cycle.
\code
RIGHTARM right_arm.ini
LEFTARM left_arm.ini
RIGHTLEG right_leg.ini
LEFTLEG left_leg.ini
HEAD head.ini
TORSO torso.ini
\endcode

\section tested_os_sec Tested OS
Linux and Windows.

\author Lorenzo Natale
*/

#include <list>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Property.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Terminator.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ResourceFinder.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

const int FIXED_TIME_MOVE=5;
const int SAMPLER_RATE=100;
////////////////////////////

struct LimbInterfaces
{
    PolyDriver *dd;
    IPositionControl *ipos;
    IEncoders *ienc;
    Vector encoders;
    Vector speed;
    Vector cmd;
    bool *md;
    int nj;
    LimbInterfaces() : dd(0), ipos(0), ienc(0), md(0)
    { }

    ~LimbInterfaces()
    {
        if (!md)
            delete[] md;
    }

    void resize(int n)
    {
        nj=n;
        cmd.resize(nj);
        encoders.resize(nj);
        speed.resize(nj);
        md=new bool[nj];
    }

};

typedef std::list<Vector> PositionList;
typedef PositionList::iterator PositionListIterator;
typedef std::vector<PositionList> RobotSequence;
typedef std::vector<LimbInterfaces> RobotInterfaces;
typedef RobotInterfaces::iterator RobotInterfacesIterator;

enum Index
{
    RIGHTARM=0,
    LEFTARM=1,
    HEAD=2,
    TORSO=3,
    RIGHTLEG=4,
    LEFTLEG=5,
    LIMBS=6};

class DemoSequences
{
public:
    std::vector<std::string> tags;
    RobotSequence sequences;

    DemoSequences(const std::vector<std::string> &t)
    {
        tags=t;
        sequences.resize(LIMBS);
    }

    bool fromFile(const char *filename)
    {
        Property config;
        config.fromConfigFile(filename);
        bool ret=true;

        for (int k=0; k<LIMBS; k++)
        {
            Bottle seqFile=config.findGroup(tags[k].c_str());
            int length=(seqFile.findGroup("DIMENSIONS").find("numberOfPoses")).asInt();
            int nj=seqFile.findGroup("DIMENSIONS").find("numberOfJoints").asInt();
            for (int ii=0; ii<length; ii++)
            {
                char tmp[80];
                if (seqFile.findGroup("REORDER").isNull())
                    sprintf(tmp,"POSITION%d",ii);
                else
                    sprintf(tmp,"POSITION%d",seqFile.findGroup("REORDER").findGroup("order").get(ii+1).asInt());
                Bottle &xtmp=seqFile.findGroup(tmp).findGroup("jointPositions");
                Vector vect;
                vect.resize(nj);
                if (nj!=xtmp.size()-1)
                    yWarning("**** WARNING: mismatch of sizes in the input file! nj=%d, xtmp=%d \n",nj,xtmp.size());
                for (int l=0; l<xtmp.size()-1; l++)
                    vect[l]=xtmp.get(l+1).asDouble();
                sequences[k].push_back(vect);
            }
        }

        return ret;
    }

    void dump()
    {
        for (int l=0; l<LIMBS; l++)
        {
            yInfo("Sequence for %s:",tags[l].c_str());

            int s=sequences[l].size();
            PositionListIterator it=sequences[l].begin();
            while (it!=sequences[l].end())
            {
                Vector &v=(*it);
                yInfo("%s",v.toString().c_str());
                it++;
            }

            yInfo("-----");
        }
    }
};

class Robot
{
public:

    std::vector<std::string> tags;
    std::vector<Property> options;
    RobotInterfaces interfaces;

    Robot()
    {
        tags.resize(LIMBS);
        options.resize(LIMBS);
        interfaces.resize(LIMBS);
    }

    ~Robot()
    {
        RobotInterfacesIterator it;
        for (it=interfaces.begin(); it!=interfaces.end(); it++)
        {
            if ((*it).dd!=0)
            {
                (*it).dd->close();
                delete (*it).dd;
            }
        }
    }

    bool createDevices()
    {
        bool success=true;
        for (int k=0; k<LIMBS; k++)
        {
            LimbInterfaces tmp;
            tmp.dd=new PolyDriver;
            tmp.dd->open(options[k]);
            if (!tmp.dd->isValid())
            {
                yError("Error opening %s",tags[k].c_str());
                return false;
            }

            bool ret=true;
            ret=ret && tmp.dd->view(tmp.ipos);
            ret=ret && tmp.dd->view(tmp.ienc);

            interfaces[k]=tmp;
            success=success && ret;
        }
        return success;
    }
};

class RobotMover : public RateThread
{
private:
    RobotSequence sequences;
    bool sequencesF;

    bool motion_done;
    int nJoints;

    int count;
    Robot *robot;
    bool done;

    bool verbose;

    PositionListIterator *its;
    PositionListIterator *itends;
    double motionTime;
    bool spoke;
public:
    RobotMover(Robot *r, int rate) : RateThread(rate)
    {
        sequencesF=false;
        robot=r;
        verbose=false;
        its=new PositionListIterator[LIMBS];
        itends=new PositionListIterator[LIMBS];
        motionTime=FIXED_TIME_MOVE;
        spoke=false;
    }

    ~RobotMover()
    {
        delete[] its;
        delete[] itends;
    }

    void setVerbose(bool f)
    { verbose=f; }

    void waitMotion(double dT, bool checkDone=false)
    {
        bool done=false;
        Time::delay(dT);
        if (checkDone)
        {
            double acc=0;
            while (!done)
            {
                done=true;
                for (int k=0; k<LIMBS; k++)
                {
                    bool d=true;
                    bool *tmp=robot->interfaces[k].md;
                    for (int j=0; j<robot->interfaces[k].nj; j++)
                    {
                        //robot->interfaces[k].ipos->checkMotionDone(j, tmp);
                        *tmp=true;
                        done=done && (*tmp);
                        tmp++;
                    }

                    Time::delay(0.05);
                    acc+=0.05;
                }
                Time::delay(0.1);
                acc+=0.1;

                if (acc>2)
                    done=true;
            }

            for (int k=0; k<LIMBS; k++)
            {
                bool d=true;
                yInfo("%s dumping MotionDone:",robot->tags[k].c_str());
                for (int j=0; j<robot->interfaces[k].nj; j++)
                    yInfo("%s",robot->interfaces[k].md[j]?"true":"false");
            }
        }
    }

    void computeSpeed(double dT)
    {
        if (dT<=0)
            return;

        for (int l=0; l<LIMBS; l++)
        {
            int nj=robot->interfaces[l].nj;
            Vector &encs=robot->interfaces[l].encoders;
            Vector &sp=robot->interfaces[l].speed;
            Vector &cmd=robot->interfaces[l].cmd;

            for (int j=0; j<nj; j++)
            {
                double dp=fabs(encs[j]-cmd[j]);
                double tmp=dp/dT;
                if (tmp<0.1)
                    tmp=0.1;

                sp[j]=tmp;
            }
        }
    }

    void resetSequence()
    {
        for (int l=0; l<LIMBS; l++)
        {
            its[l]=sequences[l].begin();
            itends[l]=sequences[l].end();
        }
    }

    bool threadInit()
    {
        yInfo("Starting controller...");
        if (!sequencesF)
        {
            yError("sequences not set, returning false");
            return false;
        }

        resetSequence();
        for (int l=0; l<LIMBS; l++)
        {
            int nj;
            robot->interfaces[l].ipos->getAxes(&nj);
            robot->interfaces[l].resize(nj);
            robot->interfaces[l].speed=0;
            robot->interfaces[l].encoders=0;
            robot->interfaces[l].cmd=0;
        }

        done=false;
        return true;
    }

    void run()
    {
        yInfo("Running...");
        spoke=false;
        if (!done)
        {
            for (int l=0; l<LIMBS; l++)
            {
                Vector &p=*(its[l]);
                while (!robot->interfaces[l].ienc->getEncoders(robot->interfaces[l].encoders.data()))
                {
                    if (!spoke)
                    {
                        yDebug("Waiting for encoders!\n");
                        spoke=true;
                    }

                }
                robot->interfaces[l].cmd=p;
                its[l]++;
            }

            computeSpeed(motionTime);

            for (int l=0; l<LIMBS; l++)
            {
                robot->interfaces[l].ipos->setRefSpeeds(robot->interfaces[l].speed.data());
                robot->interfaces[l].ipos->positionMove(robot->interfaces[l].cmd.data());

                if (verbose)
                {
                    yInfo("%s",robot->tags[l].c_str());
                    yInfo("going to: %s ",robot->interfaces[l].cmd.toString().c_str());
                    yInfo("speed: %s ",robot->interfaces[l].speed.toString().c_str());
                    yInfo("-------");
                }
            }

            //wait a bit less, adjustment
            waitMotion(motionTime);
        }

        if (its[0]==itends[0])
        {
            static int count=1;
            resetSequence();
            yInfo("Sequence %d completed",count);
            count++;
            //done=true;
        }
    }

    void setTime(double dT)
    {
        motionTime=dT;
    }

    void waitDone()
    {
        while (!done)
            Time::delay(0.5);
    }

    void suspend()
    {

    }

    void resume()
    {

    }

    void threadRelease()
    {
        yInfo("Stopping controller...");
    }

    void setSequences(const RobotSequence &seq)
    {
        sequences=seq;
        sequencesF=true;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    Time::turboBoost();

    // pick up name of main configuration file

    // find configuration file via ResourceFinder if needed
    // This is trivial if full path has already been provided,
    // but less trivial if just a filename has been given.
    // There is also the possibility of user overrides in
    // standardized forms not anticipated in this program.
    ResourceFinder finder;
    finder.setDefaultContext("demoYoga");
    finder.configure(argc,argv);
    finder.setVerbose(true);
    finder.setDefault("positions","yoga.ini");
    finder.setDefault("robot","icub");
    std::string inifile=finder.findFile("positions").c_str();
    std::string robotName=finder.find("robot").asString().c_str();

    bool verbose=finder.check("verbose");
    bool diagnostic=finder.check("diagnostic");

    if (inifile=="")
    {
        yError("No position file specified");
        return 0;
    }
    else
    {
        yInfo("Reading positions file from %s",inifile.c_str());
    }

    std::string prefix=string("/")+robotName+"/";

    Robot icub;

    icub.options[0].put("device","remote_controlboard");
    icub.options[0].put("local",(prefix+"demoYoga/right_arm/client").c_str());
    icub.options[0].put("remote",(prefix+"right_arm").c_str());
    icub.tags[0]="RIGHTARM";

    icub.options[1].put("device","remote_controlboard");
    icub.options[1].put("local",(prefix+"demoYoga/left_arm/client").c_str());
    icub.options[1].put("remote",(prefix+"left_arm").c_str());
    icub.tags[1]="LEFTARM";

    icub.options[2].put("device","remote_controlboard");
    icub.options[2].put("local",(prefix+"demoYoga/head/client").c_str());
    icub.options[2].put("remote",(prefix+"head").c_str());
    icub.tags[2]="HEAD";

    icub.options[3].put("device","remote_controlboard");
    icub.options[3].put("local",(prefix+"demoYoga/torso/client").c_str());
    icub.options[3].put("remote",(prefix+"torso").c_str());
    icub.tags[3]="TORSO";

    icub.options[4].put("device","remote_controlboard");
    icub.options[4].put("local",(prefix+"demoYoga/right_leg/client").c_str());
    icub.options[4].put("remote",(prefix+"right_leg").c_str());
    icub.tags[4]="RIGHTLEG";

    icub.options[5].put("device","remote_controlboard");
    icub.options[5].put("local",(prefix+"demoYoga/left_leg/client").c_str());
    icub.options[5].put("remote",(prefix+"left_leg").c_str());
    icub.tags[5]="LEFTLEG";

    if (diagnostic)
    {
        for (int k=0; k<6; k++)
            icub.options[k].put("diagnostic","");
    }

    DemoSequences *sequences=new DemoSequences(icub.tags);

    sequences->fromFile(inifile.c_str());
    if (verbose)
        sequences->dump();

    Property op;
    op.fromConfigFile(inifile.c_str());
    double dT=-1;
    if (op.check("time"))
        dT=op.find("time").asDouble();

    if (!icub.createDevices())
    {
        yError("Error creating devices, check parameters");
        return -1;
    }

    RobotMover *robot_mover=new RobotMover(&icub,SAMPLER_RATE);
    robot_mover->setVerbose(verbose);
    robot_mover->setSequences(sequences->sequences);
    if (dT>0)
    {
        yInfo("Setting time to %g",dT);
        robot_mover->setTime(dT);
    }

    robot_mover->start();

    std::string reply;
    while (reply!="quit")
    {
        std::cin>>reply;
    }

    yInfo("Received a quit message");
    robot_mover->stop();

    delete robot_mover;
    delete sequences;

    return 0;
}
