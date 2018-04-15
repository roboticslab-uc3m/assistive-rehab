/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <vector>
#include <iterator>
#include <string>
#include <fstream>

#include <yarp/os/all.h>

#include "AssistiveRehab/skeleton.h"
#include "src/skeletonPlayer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace assistive_rehab;


/****************************************************************/
struct MetaSkeleton
{
    shared_ptr<Skeleton> s;
    double t;
};


/****************************************************************/
class Player : public RFModule, public skeletonPlayer_IDL
{
    Mutex mutex;
    vector<MetaSkeleton> skeletons;
    vector<MetaSkeleton>::iterator it,it_begin,it_end;
    enum class State { idle, loaded, opced, running } state;

    const int opc_id_invalid=-1;
    int opc_id;

    double opacity;
    int n_sessions;
    double t_warp;
    double T0;
    double t_origin;

    BufferedPort<Bottle> viewerPort;
    RpcClient opcPort;
    RpcServer cmdPort;

    /****************************************************************/
    void viewerUpdate(Property &prop)
    {
        if (viewerPort.getOutputCount()>0)
        {
            prop.put("opacity",opacity);
            Bottle &msg=viewerPort.prepare();
            msg.clear();
            msg.addList().read(prop);
            viewerPort.writeStrict();
        }
    }

    /****************************************************************/
    bool opcAdd()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));
            Property prop=it->s->toProperty();
            cmd.addList().read(prop);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    opc_id=rep.get(1).asList()->get(1).asInt();
                    viewerUpdate(prop);
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
            Property prop=it->s->toProperty();
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
                    viewerUpdate(prop);
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
                    opc_id=opc_id_invalid;
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool findFrameDirect(const vector<MetaSkeleton>::iterator &it_begin,
                         const double t_begin, vector<MetaSkeleton>::iterator &it,
                         const double t_warp=1.0)
    {
        for (auto it_=it_begin; it_!=end(skeletons); it_++)
        {
            if (t_warp*(it_->t-T0)>=t_begin)
            {
                it=it_;
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool findFrameReverse(const vector<MetaSkeleton>::iterator &it_end,
                          const double t_end, vector<MetaSkeleton>::iterator &it,
                          const double t_warp=1.0)
    {
        for (auto it_=it_end; it_--!=begin(skeletons);)
        {
            if (t_warp*(skeletons.back().t-it_->t)>=t_end)
            {
                it=it_;
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool load(const string& file, const string& context) override
    {
        LockGuard lg(mutex);
        ResourceFinder rf;

        rf.setQuiet();
        rf.setDefaultContext(context);
        rf.configure(0,nullptr);

        string abspathFile=rf.findFile(file);
        if (abspathFile.empty())
        {
            yError()<<"Unable to find"<<file;
            return false;
        }

        ifstream fin(abspathFile);
        if (!fin.is_open())
        {
            yError()<<"Unable to open"<<file;
            return false;
        }

        bool ok=true;
        vector<MetaSkeleton> skeletons_;
        for (string line; getline(fin,line);)
        {
            Bottle bottle(line);
            if (bottle.size()!=4)
            {
                ok=false;
                break;
            }

            MetaSkeleton sk;
            sk.t=bottle.get(1).asDouble();

            Property prop;
            if (Bottle *b=bottle.get(3).asList())
            {
                b->write(prop);
                sk.s=shared_ptr<Skeleton>(skeleton_factory(prop));
            }
            else
            {
                ok=false;
                break;
            }

            skeletons_.push_back(sk);
        }
        fin.close();

        if (ok && !skeletons_.empty())
        {
            skeletons=skeletons_;
            T0=skeletons.front().t;
            if ((state==State::opced) || (state==State::running))
            {
                opcDel();
            }
            state=State::loaded;

            yInfo()<<"found file:"<<abspathFile;
            yInfo()<<"loaded #"<<skeletons.size()<<"skeletons";
            yInfo()<<"time span of"<<skeletons.back().t-T0<<"seconds";
            yInfo()<<"average frame time of"<<(skeletons.back().t-T0)/(double)skeletons.size()<<"seconds";
        }
        else
        {
            yError()<<"Wrong file format!";
        }
        return ok;
    }

    /****************************************************************/
    bool start(const int n_sessions, const double t_warp,
               const double t_begin, const double t_end) override
    {
        LockGuard lg(mutex);
        if ((state==State::loaded) || (state==State::opced))
        {
            if (findFrameDirect(begin(skeletons),t_begin,it_begin) &&
                findFrameReverse(end(skeletons),t_end,it_end))
            {
                this->n_sessions=n_sessions;
                this->t_warp=t_warp;
                it=it_begin;
                if (state==State::loaded)
                {
                    if (!opcAdd())
                    {
                        yError()<<"Unable to stream!";
                        return false;
                    }
                }
                yInfo()<<"Streaming started";
                state=State::running;
                t_origin=Time::now();
                return true;
            }
        }

        yError()<<"Unable to stream!";
        return false;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        if (state==State::running)
        {
            yInfo()<<"Streaming ended";
            state=State::opced;
        }
        return true;
    }

    /****************************************************************/
    bool is_running() override
    {
        LockGuard lg(mutex);
        return (state==State::running);
    }

    /****************************************************************/
    bool put_in_opc(const double t_begin) override
    {
        LockGuard lg(mutex);
        if (skeletons.empty())
        {
            yError()<<"No file loaded yet!";
            return false;
        }
        else if (state==State::running)
        {
            state=State::opced;
        }

        if (findFrameDirect(begin(skeletons),t_begin,it))
        {
            if (state==State::opced)
            {
                return opcSet();
            }
            else if (opcAdd())
            {
                state=State::opced;
                return true;
            }
        }

        yError()<<"Skeleton not present in OPC!";
        return false;
    }

    /****************************************************************/
    bool remove_from_opc() override
    {
        LockGuard lg(mutex);
        if ((state==State::opced) || (state==State::running))
        {
            if (opcDel())
            {
                state=State::loaded;
                return true;
            }
        }

        yError()<<"Skeleton not present in OPC!";
        return false;
    }

    /****************************************************************/
    bool set_tag(const string& new_tag) override
    {
        LockGuard lg(mutex);
        if (skeletons.empty())
        {
            yError()<<"No file loaded yet!";
            return false;
        }
        
        for (auto &sk:skeletons)
        {
            sk.s->setTag(new_tag);
        }
        return true;
    }

    /****************************************************************/
    bool set_opacity(const double new_opacity) override
    {
        LockGuard lg(mutex);
        opacity=new_opacity;
        return true;
    }

    /****************************************************************/
    bool attach(RpcServer &source)
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        viewerPort.open("/skeletonPlayer/viewer:o");
        opcPort.open("/skeletonPlayer/opc:rpc");
        cmdPort.open("/skeletonPlayer/cmd:rpc");
        attach(cmdPort);

        state=State::idle;
        opc_id=opc_id_invalid;
        opacity=0.2;
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
        LockGuard lg(mutex);
        double t=Time::now()-t_origin;
        if (state==State::running)
        {
            vector<MetaSkeleton>::iterator it_next=end(skeletons);
            findFrameDirect(it,t,it_next,t_warp);

            if (it_next<=it_end)
            {
                it=it_next;
                yInfo()<<"Streaming frame #"<<distance(begin(skeletons),it)
                       <<"in ["<<distance(begin(skeletons),it_begin)
                       <<","<<distance(begin(skeletons),it_end)<<"]";
            }
            else if (n_sessions!=1)
            {
                yInfo()<<"Session ended";
                it=it_begin;
                t_origin=Time::now();
                if (n_sessions>1)
                {
                    n_sessions--;
                }
            }
            else
            {
                yInfo()<<"Streaming ended";
                state=State::opced;
            }
        }
        if ((state==State::opced) || (state==State::running))
        {
            opcSet();
        }
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        viewerPort.close();
        opcPort.close();
        cmdPort.close();
        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Player player;
    return player.runModule(rf);
}
