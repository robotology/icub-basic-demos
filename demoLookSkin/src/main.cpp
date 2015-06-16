/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone & Ugo Pattacini
 * email:  alessandro.roncone@iit.it, ugo.pattacini@iit.it
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

#include <cstdio>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/math.h>
#include <iCub/skinDynLib/skinContactList.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;

#define NOARM                   0
#define LEFTARM                 1
#define RIGHTARM                2
#define USEDARM                 3

#define OPENHAND                0
#define CLOSEHAND               1

#define STATE_IDLE              0
#define STATE_LOOK              1
#define STATE_WAIT              2
#define STATE_RELEASE           3


/***********************************************************/
class LookingModule: public RFModule
{
    PolyDriver drvLArm, drvRArm;
    PolyDriver drvRCart,drvLCart,drvGCart;

    IEncoders         *iencLA;
    IPositionControl2 *iposLA;
    IEncoders         *iencRA;
    IPositionControl2 *iposRA;

    ICartesianControl *icartR;
    ICartesianControl *icartL;
    IGazeControl      *igaze;

    BufferedPort<skinContactList> skinPort;
    RpcClient rpcPortOut;
    RpcServer rpcPortIn;

    Port outportSpeech;

    bool isRUAenabled;   // is right upper arm enabled
    bool isRFAenabled;   // is right fore  arm enabled
    bool isRHenabled;    // is right hand      enabled
    bool isTenabled;
    bool isLUAenabled;   // is left  upper arm enabled
    bool isLFAenabled;   // is left  fore  arm enabled
    bool isLHenabled;    // is left  hand      enabled

    Mutex mutex;

    bool isRunning;

    std::vector<string> speech;
    string touchedSkinPart;

    bool state;
    Vector targetPos;
    double timeNow;

    Vector openHandPoss, closeHandPoss;
    Vector handVels;

    void moveHand(const int action, const int sel=USEDARM)
    {
        IPositionControl *ipos=NULL;
        Vector           *poss=NULL;

        switch (action)
        {
            case OPENHAND:
                poss=&openHandPoss;
                break;
            case CLOSEHAND:
                poss=&closeHandPoss;
                break;
            default:
                return;
        }

        if (sel==LEFTARM)
        {    
            ipos=iposRA;
            drvLArm.view(ipos);
        }
        else if (sel==RIGHTARM)
        {    
            ipos=iposLA;
            drvRArm.view(ipos);
        }
        else
            return;

        for (size_t j=0; j<handVels.length(); j++)
        {
            int k=7.0+j;
            ipos->setRefSpeed(k,handVels[j]);
            ipos->positionMove(k,(*poss)[j]);
        }
    }

    void openHand(const int sel=USEDARM)
    {
        moveHand(OPENHAND,sel);
    }

    void closeHand(const int sel=USEDARM)
    {        
        moveHand(CLOSEHAND,sel);
    }

public:
    /***********************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        LockGuard lg(mutex);

        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        string cmd=command.get(0).asString().c_str();
        if (cmd=="start")
        {
            isRunning=true;
            reply.addVocab(ack);
        }
        else if (cmd=="stop")
        {
            isRunning=false;
            state=STATE_IDLE;
            openHand(RIGHTARM);
            openHand(LEFTARM);
            touchedSkinPart="";
            reply.addVocab(ack);
        }
        else if (cmd=="get")
        {
            reply.addVocab(ack);
            reply.addString(isRunning?"on":"off");
            reply.addString("State is:");
            reply.addDouble(double(state));
        }
        else
            return RFModule::respond(command,reply);

        return true; 
    }

    /***********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("lookSkin")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();

        isRunning=rf.check("autoStart");

        isRUAenabled=!rf.check("noRightUpperArm");
        isRFAenabled=!rf.check("noRightForeArm");
        isRHenabled=!rf.check("noRightHand");
        isTenabled=!rf.check("noTorso");
        isLUAenabled=!rf.check("noLeftUpperArm");
        isLFAenabled=!rf.check("noLeftForeArm");
        isLHenabled=!rf.check("noLeftHand");

        Property optRCart;
        optRCart.put("device","cartesiancontrollerclient");
        optRCart.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
        optRCart.put("local",("/"+name+"/cartesian/right_arm").c_str());

        Property optLCart;
        optLCart.put("device","cartesiancontrollerclient");
        optLCart.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
        optLCart.put("local",("/"+name+"/cartesian/left_arm").c_str());

        Property optGCart;
        optGCart.put("device","gazecontrollerclient");
        optGCart.put("remote","/iKinGazeCtrl");
        optGCart.put("local",("/"+name+"/gaze").c_str());

        Property optLArm;
        optLArm.put("device","remote_controlboard");
        optLArm.put("remote",("/"+robot+"/left_arm").c_str());
        optLArm.put("local",("/"+name+"/left_arm").c_str());

        Property optRArm;
        optRArm.put("device","remote_controlboard");
        optRArm.put("remote",("/"+robot+"/right_arm").c_str());
        optRArm.put("local",("/"+name+"/right_arm").c_str());

        outportSpeech.open(("/"+name+"/speech:o").c_str());

        if (!drvRCart.open(optRCart) || !drvLCart.open(optLCart) ||
            !drvGCart.open(optGCart) ||
            !drvRArm.open(optRArm)   || !drvLArm.open(optLArm))
        {
            drvRCart.close();
            drvLCart.close();
            drvGCart.close();
            drvRArm.close();
            drvLArm.close();
            return false;
        }

        drvRCart.view(icartR);
        drvLCart.view(icartL);
        drvGCart.view(igaze);
        drvRArm.view(iencRA);
        drvRArm.view(iposRA);
        drvLArm.view(iencLA);
        drvLArm.view(iposLA);

        skinPort.open(("/"+name+"/skin:i").c_str());
        rpcPortOut.open(("/"+name+"/rpc:o").c_str());
        rpcPortIn.open(("/"+name+"/rpc:i").c_str());
        attach(rpcPortIn);

        Rand::init();
        speech.push_back("""Ouch!""");
        speech.push_back("""how can I help you?""");

        state=0;
        targetPos.resize(3,0.0);
        timeNow=Time::now();

        openHandPoss.resize(9,0.0);

        closeHandPoss.resize(9,0.0);
        closeHandPoss[1]=0.0;   closeHandPoss[2]=80.0;  closeHandPoss[3]=30.0;
        closeHandPoss[4]=70.0;  closeHandPoss[5]=40.0;  closeHandPoss[6]=45.0;
        closeHandPoss[7]=40.0;  closeHandPoss[8]=55.0;  closeHandPoss[9]=250.0;

        handVels.resize(9,0.0);
        handVels[1]=20.0;   handVels[2]=60.0;   handVels[3]=30.0;
        handVels[4]=50.0;   handVels[5]=30.0;   handVels[6]=30.0;
        handVels[7]=30.0;   handVels[8]=40.0;   handVels[9]=200.0;

        touchedSkinPart="";

        return true;
    }

    /***********************************************************/
    bool interruptModule()
    {
        skinPort.interrupt();
        rpcPortOut.interrupt();
        rpcPortIn.interrupt();

        return true;
    }

    /***********************************************************/
    bool close()
    {
        drvRCart.close();
        drvLCart.close();
        drvGCart.close();
        drvLArm.close();
        drvRArm.close();

        skinPort.close();
        rpcPortOut.close();
        rpcPortIn.close();

        outportSpeech.interrupt();
        outportSpeech.close();

        return true;
    }

    /***********************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /***********************************************************/
    bool detectContact(skinContactList *sCL, Vector &x)
    {
        for (skinContactList::iterator it=sCL->begin(); it!=sCL->end(); it++)
        {
            if (it->getPressure()>40)
            {
                yDebug("CONTACT! Skin part: %s",it->getSkinPartName().c_str());

                ICartesianControl **icart;
                if ((it->getSkinPartName()=="skin_right_upper_arm" && isRUAenabled) ||
                    (it->getSkinPartName()=="skin_right_forearm" && isRFAenabled))
                {
                    icart=&icartR;
                }
                else if ((it->getSkinPartName()=="skin_right_hand" && isRHenabled))
                {
                    touchedSkinPart="skin_right_hand";
                    icart=&icartR;
                }
                else if ((it->getSkinPartName()=="skin_left_upper_arm" && isLUAenabled) ||
                    (it->getSkinPartName()=="skin_left_forearm" && isLFAenabled))
                {
                    icart=&icartL;
                }
                else if ((it->getSkinPartName()=="skin_left_hand" && isLHenabled))
                {
                    touchedSkinPart="skin_left_hand";
                    icart=&icartL;
                }
                else if (it->getSkinPartName()=="skin_front_torso" && isTenabled)
                {
                    x.resize(3,0.0);
                    x[0]=-0.1;
                    return true;
                }
                else
                    return false;
                
                Vector pos,orien;
                (*icart)->getPose(3+it->getLinkNumber(),pos,orien);
                Matrix H=axis2dcm(orien);
                H(0,3)=pos[0];
                H(1,3)=pos[1];
                H(2,3)=pos[2];
                x=it->getCoP();
                x.push_back(1.0);
                x=H*x;
                x.pop_back();

                return true;
            }
        }

        return false;
    }

    void sendSpeak(const string &txt)
    {
        if (outportSpeech.getOutputCount()>0)
        {
            Bottle msg,reply;
            msg.addString(txt);
            outportSpeech.write(msg);
        }
    }

    /***********************************************************/
    bool updateModule()
    {
        printf("State is %i\n",state );
        LockGuard lg(mutex);

        if (isRunning)
        {
            if (state==STATE_IDLE)
            {
                targetPos.resize(3,0.0);

                skinContactList *contacts=skinPort.read();
                if (contacts==NULL)
                    return true;

                if (detectContact(contacts,targetPos))
                {
                    state=STATE_LOOK;
                }
            }
            else if (state==STATE_LOOK)
            {
                if (rpcPortOut.getOutputCount()>0)
                {
                    Bottle cmd;
                    cmd.addVocab(Vocab::encode("stop"));
                    rpcPortOut.write(cmd);
                }

                yDebug("Looking at: %s",targetPos.toString().c_str());
                igaze->lookAtFixationPoint(targetPos);
                sendSpeak(speech[Rand::scalar(0,speech.size()-1e-3)]);

                if (touchedSkinPart=="skin_right_hand" || touchedSkinPart=="skin_left_hand")
                {
                    yDebug("Closing : %s",touchedSkinPart.c_str());
                    closeHand(touchedSkinPart=="skin_left_hand"?LEFTARM:RIGHTARM);
                }

                timeNow=Time::now();
                state=STATE_WAIT;
            }
            else if (state==STATE_WAIT)
            {
                if (Time::now()-timeNow>3.0)
                {
                    state=STATE_RELEASE;
                }
            }
            else if (state==STATE_RELEASE)
            {
                if (rpcPortOut.getOutputCount()>0)
                {
                    Bottle cmd;
                    cmd.addVocab(Vocab::encode("start"));
                    rpcPortOut.write(cmd);
                }

                igaze->lookAtAbsAngles(Vector(3,0.0));

                if (touchedSkinPart=="skin_right_hand" || touchedSkinPart=="skin_left_hand")
                {
                    yDebug("Opening: %s",touchedSkinPart.c_str());
                    openHand(touchedSkinPart=="skin_left_hand"?LEFTARM:RIGHTARM);
                }

                state=STATE_IDLE;
            }
        }

        return true;
    }
};


/***********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server is unavailable!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    LookingModule mod;
    return mod.runModule(rf);
}
