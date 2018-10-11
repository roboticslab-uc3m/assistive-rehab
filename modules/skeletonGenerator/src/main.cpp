/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file skeletonGenerator.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cmath>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "AssistiveRehab/skeleton.h"
#include "src/skeletonGenerator_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class skeletonGenerator : public RFModule, public skeletonGenerator_IDL
{
    SkeletonWaist skeleton;
    double radius,phase,t0;
    BufferedPort<Bottle> outPort;
    RpcServer rpcPort;
    RpcClient opcPort;

    ResourceFinder rf;

    string joint;
    vector<int> dir_nochange,dir_cos,dir_sin;
    Vector elbowLeft_init;
    Vector elbowRight_init;
    Vector handLeft_init;
    Vector handRight_init;
    Vector head_init;
    Vector shoulderCenter_init;
    Vector shoulderLeft_init;
    Vector shoulderRight_init;
    Vector hipLeft_init;
    Vector hipRight_init;
    Vector kneeLeft_init;
    Vector kneeRight_init;
    Vector ankleLeft_init;
    Vector ankleRight_init;
    bool move;

    int opc_id;

    Mutex mutex;

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf_) override
    {
        rf=rf_;
        string moduleName=rf.check("name", Value("skeletonGenerator")).asString();
        setName(moduleName.c_str());

        loadInit();
        vector<pair<string,Vector>> unordered;
        setInitialPose(unordered);
        skeleton.update(unordered);

        Vector d=skeleton[KeyPointTag::head]->getPoint()-
                 skeleton[KeyPointTag::shoulder_center]->getPoint();
        radius=norm(d);
        phase=atan2(d[1],d[0]);

        outPort.open("/" + moduleName + "/skel:o");
        rpcPort.open("/" + moduleName + "/cmd:rpc");
        opcPort.open("/" + moduleName + "/opc:rpc");
        attach(rpcPort);

        move=false;
        t0=Time::now();

        return true;
    }

    /****************************************************************/
    bool loadInit()
    {
        Bottle bGeneral=rf.findGroup("general");
        if(!bGeneral.isNull())
        {
            Bottle *bElbowL = bGeneral.find("elbow_left_init").asList();
            Bottle *bElbowR = bGeneral.find("elbow_right_init").asList();
            Bottle *bHandL = bGeneral.find("hand_left_init").asList();
            Bottle *bHandR = bGeneral.find("hand_right_init").asList();
            Bottle *bHead = bGeneral.find("head_init").asList();
            Bottle *bShoulderC = bGeneral.find("shoulder_center_init").asList();
            Bottle *bShoulderL = bGeneral.find("shoulder_left_init").asList();
            Bottle *bShoulderR = bGeneral.find("shoulder_right_init").asList();
            Bottle *bHipL = bGeneral.find("hip_left_init").asList();
            Bottle *bHipR = bGeneral.find("hip_right_init").asList();
            Bottle *bKneeL = bGeneral.find("knee_left_init").asList();
            Bottle *bKneeR = bGeneral.find("knee_right_init").asList();
            Bottle *bAnkleL = bGeneral.find("ankle_left_init").asList();
            Bottle *bAnkleR = bGeneral.find("ankle_right_init").asList();
            if(bElbowL && bElbowR && bHandL && bHandR && bHead && bShoulderC
                    && bShoulderL && bShoulderR && bHipL && bHipR
                    && bKneeL && bKneeR && bAnkleL && bAnkleR)
            {
                elbowLeft_init.clear();
                elbowRight_init.clear();
                handLeft_init.clear();
                handRight_init.clear();
                head_init.clear();
                shoulderCenter_init.clear();
                shoulderLeft_init.clear();
                shoulderRight_init.clear();
                hipLeft_init.clear();
                hipRight_init.clear();
                kneeLeft_init.clear();
                kneeRight_init.clear();
                ankleLeft_init.clear();
                ankleRight_init.clear();

                elbowLeft_init.resize(3);
                elbowRight_init.resize(3);
                handLeft_init.resize(3);
                handRight_init.resize(3);
                head_init.resize(3);
                shoulderCenter_init.resize(3);
                shoulderLeft_init.resize(3);
                shoulderRight_init.resize(3);
                hipLeft_init.resize(3);
                hipRight_init.resize(3);
                kneeLeft_init.resize(3);
                kneeRight_init.resize(3);
                ankleLeft_init.resize(3);
                ankleRight_init.resize(3);

                elbowLeft_init[0]=bElbowL->get(0).asDouble();
                elbowLeft_init[1]=bElbowL->get(1).asDouble();
                elbowLeft_init[2]=bElbowL->get(2).asDouble();

                elbowRight_init[0]=bElbowR->get(0).asDouble();
                elbowRight_init[1]=bElbowR->get(1).asDouble();
                elbowRight_init[2]=bElbowR->get(2).asDouble();

                handLeft_init[0]=bHandL->get(0).asDouble();
                handLeft_init[1]=bHandL->get(1).asDouble();
                handLeft_init[2]=bHandL->get(2).asDouble();

                handRight_init[0]=bHandR->get(0).asDouble();
                handRight_init[1]=bHandR->get(1).asDouble();
                handRight_init[2]=bHandR->get(2).asDouble();

                head_init[0]=bHead->get(0).asDouble();
                head_init[1]=bHead->get(1).asDouble();
                head_init[2]=bHead->get(2).asDouble();

                shoulderCenter_init[0]=bShoulderC->get(0).asDouble();
                shoulderCenter_init[1]=bShoulderC->get(1).asDouble();
                shoulderCenter_init[2]=bShoulderC->get(2).asDouble();

                shoulderLeft_init[0]=bShoulderL->get(0).asDouble();
                shoulderLeft_init[1]=bShoulderL->get(1).asDouble();
                shoulderLeft_init[2]=bShoulderL->get(2).asDouble();

                shoulderRight_init[0]=bShoulderR->get(0).asDouble();
                shoulderRight_init[1]=bShoulderR->get(1).asDouble();
                shoulderRight_init[2]=bShoulderR->get(2).asDouble();

                hipLeft_init[0]=bHipL->get(0).asDouble();
                hipLeft_init[1]=bHipL->get(1).asDouble();
                hipLeft_init[2]=bHipL->get(2).asDouble();

                hipRight_init[0]=bHipR->get(0).asDouble();
                hipRight_init[1]=bHipR->get(1).asDouble();
                hipRight_init[2]=bHipR->get(2).asDouble();

                kneeLeft_init[0]=bKneeL->get(0).asDouble();
                kneeLeft_init[1]=bKneeL->get(1).asDouble();
                kneeLeft_init[2]=bKneeL->get(2).asDouble();

                kneeRight_init[0]=bKneeR->get(0).asDouble();
                kneeRight_init[1]=bKneeR->get(1).asDouble();
                kneeRight_init[2]=bKneeR->get(2).asDouble();

                ankleLeft_init[0]=bAnkleL->get(0).asDouble();
                ankleLeft_init[1]=bAnkleL->get(1).asDouble();
                ankleLeft_init[2]=bAnkleL->get(2).asDouble();

                ankleRight_init[0]=bAnkleR->get(0).asDouble();
                ankleRight_init[1]=bAnkleR->get(1).asDouble();
                ankleRight_init[2]=bAnkleR->get(2).asDouble();
            }
            else
            {
                yError() << "Unable to find initial pose";
                return false;
            }
        }
        else
        {
            yError() << "Unable to find group general";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool loadInit(const Bottle &bMotion)
    {
        bool elbowL = bMotion.check("elbow_left_init");
        bool elbowR = bMotion.check("elbow_right_init");
        bool handL = bMotion.check("hand_left_init");
        bool handR = bMotion.check("hand_right_init");
        bool head = bMotion.check("head_init");
        bool shoulderC = bMotion.check("shoulder_center_init");
        bool shoulderL = bMotion.check("shoulder_left_init");
        bool shoulderR = bMotion.check("shoulder_right_init");
        bool hipL = bMotion.check("hip_left_init");
        bool hipR = bMotion.check("hip_right_init");
        bool kneeL = bMotion.check("knee_left_init");
        bool kneeR = bMotion.check("knee_right_init");
        bool ankleL = bMotion.check("ankle_left_init");
        bool ankleR = bMotion.check("ankle_right_init");
        if(elbowL)
        {
            Bottle *bElbowL = bMotion.find("elbow_left_init").asList();
            elbowLeft_init[0]=bElbowL->get(0).asDouble();
            elbowLeft_init[1]=bElbowL->get(1).asDouble();
            elbowLeft_init[2]=bElbowL->get(2).asDouble();
        }
        if(elbowR)
        {
            Bottle *bElbowR = bMotion.find("elbow_right_init").asList();
            elbowRight_init[0]=bElbowR->get(0).asDouble();
            elbowRight_init[1]=bElbowR->get(1).asDouble();
            elbowRight_init[2]=bElbowR->get(2).asDouble();
        }
        if(handL)
        {
            Bottle *bHandL = bMotion.find("hand_left_init").asList();
            handLeft_init[0]=bHandL->get(0).asDouble();
            handLeft_init[1]=bHandL->get(1).asDouble();
            handLeft_init[2]=bHandL->get(2).asDouble();
        }
        if(handR)
        {
            Bottle *bHandR = bMotion.find("hand_right_init").asList();
            handRight_init[0]=bHandR->get(0).asDouble();
            handRight_init[1]=bHandR->get(1).asDouble();
            handRight_init[2]=bHandR->get(2).asDouble();
        }
        if(head)
        {
            Bottle *bHead = bMotion.find("head_init").asList();
            head_init[0]=bHead->get(0).asDouble();
            head_init[1]=bHead->get(1).asDouble();
            head_init[2]=bHead->get(2).asDouble();
        }
        if(shoulderC)
        {
            Bottle *bShoulderC = bMotion.find("shoulder_center_init").asList();
            shoulderCenter_init[0]=bShoulderC->get(0).asDouble();
            shoulderCenter_init[1]=bShoulderC->get(1).asDouble();
            shoulderCenter_init[2]=bShoulderC->get(2).asDouble();
        }
        if(shoulderL)
        {
            Bottle *bShoulderL = bMotion.find("shoulder_left_init").asList();
            shoulderLeft_init[0]=bShoulderL->get(0).asDouble();
            shoulderLeft_init[1]=bShoulderL->get(1).asDouble();
            shoulderLeft_init[2]=bShoulderL->get(2).asDouble();
        }
        if(shoulderR)
        {
            Bottle *bShoulderR = bMotion.find("shoulder_right_init").asList();
            shoulderRight_init[0]=bShoulderR->get(0).asDouble();
            shoulderRight_init[1]=bShoulderR->get(1).asDouble();
            shoulderRight_init[2]=bShoulderR->get(2).asDouble();
        }
        if(hipL)
        {
            Bottle *bHipL = bMotion.find("hip_left_init").asList();
            hipLeft_init[0]=bHipL->get(0).asDouble();
            hipLeft_init[1]=bHipL->get(1).asDouble();
            hipLeft_init[2]=bHipL->get(2).asDouble();
        }
        if(hipR)
        {
            Bottle *bHipR = bMotion.find("hip_right_init").asList();
            hipRight_init[0]=bHipR->get(0).asDouble();
            hipRight_init[1]=bHipR->get(1).asDouble();
            hipRight_init[2]=bHipR->get(2).asDouble();
        }
        if(kneeL)
        {
            Bottle *bKneeL = bMotion.find("knee_left_init").asList();
            kneeLeft_init[0]=bKneeL->get(0).asDouble();
            kneeLeft_init[1]=bKneeL->get(1).asDouble();
            kneeLeft_init[2]=bKneeL->get(2).asDouble();
        }
        if(kneeR)
        {
            Bottle *bKneeR = bMotion.find("knee_right_init").asList();
            kneeRight_init[0]=bKneeR->get(0).asDouble();
            kneeRight_init[1]=bKneeR->get(1).asDouble();
            kneeRight_init[2]=bKneeR->get(2).asDouble();
        }
        if(ankleL)
        {
            Bottle *bAnkleL = bMotion.find("ankle_left_init").asList();
            ankleLeft_init[0]=bAnkleL->get(0).asDouble();
            ankleLeft_init[1]=bAnkleL->get(1).asDouble();
            ankleLeft_init[2]=bAnkleL->get(2).asDouble();
        }
        if(ankleR)
        {
            Bottle *bAnkleR = bMotion.find("ankle_right_init").asList();
            ankleRight_init[0]=bAnkleR->get(0).asDouble();
            ankleRight_init[1]=bAnkleR->get(1).asDouble();
            ankleRight_init[2]=bAnkleR->get(2).asDouble();
        }

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 0.01;
    }

    /****************************************************************/
    bool updateModule() override
    {
        if(isMoving())
        {
            vector<Vector> p;
            vector<const KeyPoint*> children;
            generateTarget(p,children);

            vector<pair<string,Vector>> unordered=skeleton.get_unordered();
            for(int i=0; i<unordered.size(); i++)
            {
                for(int j=0; j<children.size(); j++)
                {
                    if(unordered[i].first == children[j]->getTag().c_str())
                        unordered[i] = make_pair(children[j]->getTag(),p[j]);
                }
            }

            skeleton.update(unordered);
            if(!opcSet())
                return false;

        }
        Property prop=skeleton.toProperty();
        Bottle &msg=outPort.prepare();
        msg.clear();
        msg.addList().read(prop);
        outPort.write();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        outPort.close();
        rpcPort.close();
        opcPort.close();
        return true;
    }

    /****************************************************************/
    bool generateSkeleton(const string &motion_type) override
    {
        LockGuard lg(mutex);

        skeleton.setTag(motion_type);
        Bottle bMotion=rf.findGroup(motion_type.c_str());
        if(!bMotion.isNull())
        {
            joint=bMotion.find("center").asString();
            Bottle *bDir_nochange=bMotion.find("dir_nochange").asList();
            Bottle *bDir_cos=bMotion.find("dir_cos").asList();
            Bottle *bDir_sin=bMotion.find("dir_sin").asList();
            if(bDir_nochange && bDir_cos && bDir_sin)
            {
                dir_nochange.clear();
                dir_cos.clear();
                dir_sin.clear();

                dir_nochange.resize(2);
                dir_cos.resize(2);
                dir_sin.resize(2);

                dir_nochange[0]=bDir_nochange->get(0).asInt();
                dir_nochange[1]=bDir_nochange->get(1).asInt();

                dir_cos[0]=bDir_cos->get(0).asInt();
                dir_cos[1]=bDir_cos->get(1).asInt();

                dir_sin[0]=bDir_sin->get(0).asInt();
                dir_sin[1]=bDir_sin->get(1).asInt();
            }
        }

        yInfo() << "Setting initial pose for " + motion_type;
        loadInit(bMotion);
        vector<pair<string, Vector>> unordered;
        setInitialPose(unordered);
        skeleton.update(unordered);

        return true;
    }

    /****************************************************************/
    bool start() override
    {
        LockGuard lg(mutex);
        move=true;
        if(!opcAdd())
            return false;
        return true;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        move=false;
        loadInit();
        vector<pair<string, Vector>> unordered;
        setInitialPose(unordered);
        skeleton.update(unordered);
        return true;
    }

    /****************************************************************/
    bool isMoving()
    {
        return move;
    }

    /****************************************************************/
    void generateTarget(vector<Vector> &p, vector<const KeyPoint*> &children)
    {
        double t=Time::now()-t0;
        double theta=2*M_PI*0.1*t;

        Vector c(skeleton[joint]->getPoint());
        int numchild=skeleton[joint]->getNumChild();
        int totnumchild=0;

        totnumchild+=numchild;
        for(int j=0; j<numchild; j++)
        {
            const KeyPoint *keypoint=skeleton[joint]->getChild(j);
            children.push_back(keypoint);

            int numchild2=skeleton[keypoint->getTag()]->getNumChild();
            if(numchild2)
            {
                const KeyPoint* keypoint2=skeleton[keypoint->getTag()]->getChild(0);
                children.push_back(keypoint2);
                totnumchild+=numchild2;
            }
        }

        //define trajectory
        p.resize(totnumchild);
        for(int j=0; j<totnumchild; j++)
        {
            p[j].resize(3);
            p[j][dir_nochange[0]]=c[dir_nochange[0]];
            p[j][dir_cos[0]]=c[dir_cos[0]]+dir_cos[1]*(j+1)*radius*fabs(cos(theta+phase));
            p[j][dir_sin[0]]=c[dir_sin[0]]+dir_sin[1]*(j+1)*radius*fabs(sin(theta+phase));
        }
    }

    /****************************************************************/
    void setInitialPose(vector<pair<string, Vector>> &unordered)
    {
        unordered.push_back(make_pair(KeyPointTag::shoulder_center,shoulderCenter_init));
        unordered.push_back(make_pair(KeyPointTag::head,head_init));
        unordered.push_back(make_pair(KeyPointTag::shoulder_left,shoulderLeft_init));
        unordered.push_back(make_pair(KeyPointTag::elbow_left,elbowLeft_init));
        unordered.push_back(make_pair(KeyPointTag::hand_left,handLeft_init));
        unordered.push_back(make_pair(KeyPointTag::shoulder_right,shoulderRight_init));
        unordered.push_back(make_pair(KeyPointTag::elbow_right,elbowRight_init));
        unordered.push_back(make_pair(KeyPointTag::hand_right,handRight_init));
        unordered.push_back(make_pair(KeyPointTag::hip_left,hipLeft_init));
        unordered.push_back(make_pair(KeyPointTag::knee_left,kneeLeft_init));
        unordered.push_back(make_pair(KeyPointTag::ankle_left,ankleLeft_init));
        unordered.push_back(make_pair(KeyPointTag::hip_right,hipRight_init));
        unordered.push_back(make_pair(KeyPointTag::knee_right,kneeRight_init));
        unordered.push_back(make_pair(KeyPointTag::ankle_right,ankleRight_init));
    }

    /****************************************************************/
    bool opcAdd()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));
            Property prop=skeleton.toProperty();
            cmd.addList().read(prop);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    opc_id=rep.get(1).asList()->get(1).asInt();
                    yInfo()<<opc_id;
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("set"));
            Bottle &pl=cmd.addList();
            Property prop=skeleton.toProperty();
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt(opc_id);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(opc_id);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    opc_id=-1;
                    return true;
                }
            }
        }

        return false;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    skeletonGenerator generator;
    ResourceFinder rf;

    rf.setDefaultContext("skeletonGenerator");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    return generator.runModule(rf);
}
