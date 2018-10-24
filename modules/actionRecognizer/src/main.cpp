/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <stdio.h>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/sig/Matrix.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/cc/client/client_session.h>
#include <tensorflow/cc/ops/standard_ops.h>
#include <tensorflow/cc/ops/array_ops.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace tensorflow;
using namespace assistive_rehab;

/****************************************************************/
class Recognizer : public BufferedPort<Bottle>
{
    string moduleName;
    unordered_map<string,int> keypoint2int;
    unordered_map<int,string> class_map;
    int nsteps;
    vector<float> min,max;

    Status status;
    Tensor input;
    Session* session;
    SessionOptions sess_opts;
    int step;

public:
    /****************************************************************/
    Recognizer(const string &moduleName, const int nsteps, const unordered_map<int,string> &class_map,
               const vector<float> &min, const vector<float> &max, const unordered_map<string,int> &keypoint2int )
    {
        this->moduleName = moduleName;
        this->nsteps = nsteps;
        this->class_map = class_map;
        this->min = min;
        this->max = max;
        this->keypoint2int = keypoint2int;

        // Set up input paths
        const string pathToGraph = "/home/vvasco/dev/action-recognition/lstm/fdg-model/2d/model-chevalier/train_test-1/model-4/model.meta";
        const string checkpointPath = "/home/vvasco/dev/action-recognition/lstm/fdg-model/2d/model-chevalier/train_test-1/model-4/model";

        sess_opts.config.mutable_gpu_options()->set_allow_growth(true); //to limit GPU usage
        session = NewSession(sess_opts);
        if (session == nullptr)
        {
            throw runtime_error("Could not create Tensorflow session");
        }

        // Read in the protobuf graph we exported
        MetaGraphDef graph_def;
        status = ReadBinaryProto(Env::Default(), pathToGraph, &graph_def);
        if (!status.ok())
        {
            throw runtime_error("Error reading graph definition from " + pathToGraph + ": " + status.ToString());
        }

        // Add the graph to the session
        status = session->Create(graph_def.graph_def());
        if (!status.ok())
        {
            throw runtime_error("Error creating graph: " + status.ToString());
        }

        // Read weights from the saved checkpoint
        Tensor checkpointPathTensor(DT_STRING, TensorShape());
        checkpointPathTensor.scalar<std::string>()() = checkpointPath;
        status = session->Run(
                {{ graph_def.saver_def().filename_tensor_name(), checkpointPathTensor },},
                {},
                {graph_def.saver_def().restore_op_name()},
                nullptr);
        if (!status.ok())
        {
            throw runtime_error("Error loading checkpoint from " + checkpointPath + ": " + status.ToString());
        }

        input = Tensor(DT_FLOAT, TensorShape({1,nsteps,36}));
        step = 0;

        for(size_t i=0; i<nsteps; i++)
        {
            for(size_t j=0; j<36; j++)
            {
                input.tensor<float,3>()(0,i,j) = 0.0;
            }
        }
    }

    /********************************************************/
    ~Recognizer()
    {
    };

    /****************************************************************/
    bool updateInput(const string & tag, const int i, const float &u, const float &v)
    {
        int j = keypoint2int[tag];
        input.tensor<float,3>()(0,i,j) = (u-min[j])/(max[j]-min[j]);
        input.tensor<float,3>()(0,i,j+1) = (v-min[j+1])/(max[j+1]-min[j+1]);

        return true;
    }

    /****************************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<Bottle >::open( "/" + moduleName + "/keypoints:i" );
        return true;
    }

    /****************************************************************/
    void onRead(Bottle &data)
    {
        if(data.size()!=0)
        {
            if (Bottle *b1=data.get(0).asList())
            {
                for (size_t i=0; i<b1->size(); i++)
                {
                    Bottle *b2=b1->get(i).asList();
                    for (size_t i=0; i<b2->size(); i++)
                    {
                        if (Bottle *k=b2->get(i).asList())
                        {
                            if (k->size()==4)
                            {
                                string tag=k->get(0).asString();
                                float u=k->get(1).asDouble();
                                float v=k->get(2).asDouble();
                                updateInput(tag,step,u,v);
                            }
                        }
                    }
                }
            }

            if(step < nsteps)
            {
                step++;
            }
            else
            {
                //run the inference
                //                cout << (input).tensor<float, (3)>() << endl;
                //                cout << "input" << input.DebugString() << endl;
                string input_layer = "x:0";
                string output_layer = "ArgMax:0";
                vector<Tensor> outputTensors;
                status = session->Run({{input_layer, input}}, {output_layer}, {}, &outputTensors);
                //                cout << "output " << outputTensors[0].DebugString() << endl;

                auto out = outputTensors[0].scalar<int64>();
                int pred = out(0);
                cout << "prediction: " << class_map[pred] << endl;

                step = 0;
            }
        }
    }

    /********************************************************/
    void close()
    {
        session->Close();
        delete session;
        BufferedPort<yarp::os::Bottle >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
    }
};

/********************************************************/
class Module : public RFModule
{
    ResourceFinder *rf;
    Recognizer *recognizer;

public:

    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        string moduleName = rf.check("name", Value("actionRecognizer")).asString();
        setName(moduleName.c_str());

        int nsteps = rf.check("nsteps",Value(50)).asInt(); //must be the one defined when training the model

        Bottle &bGroup=rf.findGroup("general");
        if (bGroup.isNull())
        {
            yError()<<"Unable to find group \"general\"";
            return false;
        }
        if (!bGroup.check("num-sections"))
        {
            yError()<<"Unable to find key \"num-sections\"";
            return false;
        }
        unordered_map<int,string> class_map;
        int num_sections=bGroup.find("num-sections").asInt();
        for (int i=0; i<num_sections; i++)
        {
            ostringstream section;
            section<<"section-"<<i;
            Bottle &bSection=rf.findGroup(section.str());
            if (bSection.isNull())
            {
                string msg="Unable to find section";
                msg+="\""+section.str()+"\"";
                yError()<<msg;
                return false;
            }
            if (!bSection.check("label") || !bSection.check("value"))
            {
                yError()<<"Unable to find key \"key\" and/or \"value\"";
                return false;
            }
            int value=bSection.find("value").asInt();
            string label=bSection.find("label").asString();
            class_map[value]=label;
        }

        ifstream file( "/home/vvasco/dev/action-recognition/lstm/fdg-data/2d/train_test-1/scale_values.txt" );
        string line = "";
        vector<float> min,max;
        while (getline(file, line))
        {
            vector<string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(","));
            min.push_back(stof((vec[0]).c_str(),0));
            max.push_back(stof((vec[1]).c_str(),0));
        }
        unordered_map<string,int> keypoint2int;
        keypoint2int["Nose"] = 0;
        keypoint2int["Neck"] = 2;
        keypoint2int["RShoulder"] = 4;
        keypoint2int["RElbow"] = 6;
        keypoint2int["RWrist"] = 8;
        keypoint2int["LShoulder"] = 10;
        keypoint2int["LElbow"] = 12;
        keypoint2int["LWrist"] = 14;
        keypoint2int["RHip"] = 16;
        keypoint2int["RKnee"] = 18;
        keypoint2int["RAnkle"] = 20;
        keypoint2int["LHip"] = 22;
        keypoint2int["LKnee"] = 24;
        keypoint2int["LAnkle"] = 26;
        keypoint2int["REye"] = 28;
        keypoint2int["LEye"] = 30;
        keypoint2int["REar"] = 32;
        keypoint2int["LEar"] = 34;

        recognizer = new Recognizer( moduleName, nsteps, class_map, min, max, keypoint2int );
        /* now start the thread to do the work */
        recognizer->open();

        return true;
    }

    /**********************************************************/
    bool close()
    {
        recognizer->interrupt();
        recognizer->close();
        delete recognizer;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
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
    rf.setDefaultContext("actionRecognizer");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Module module;
    return module.runModule(rf);
}


