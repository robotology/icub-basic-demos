/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include <boost/bind.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>

namespace gazebo {

/******************************************************************************/
class ModelMover : public gazebo::ModelPlugin
{
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr renderer_connection;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    ignition::math::Vector3d starting_pos;

    /**************************************************************************/
    void onWorldFrame()
    {
        if (auto* b = port.read(false))
        {
            if (b->size() >= 3)
            {
                if (model->GetJoint("fixed_to_ground"))
                {
                    if (model->RemoveJoint("fixed_to_ground"))
                    {
                        yInfo() << "Removed fixed_to_ground joint";
                    }
                }

                const auto x = starting_pos.X() + b->get(0).asFloat64();
                const auto y = starting_pos.Y() + b->get(1).asFloat64();
                const auto z = starting_pos.Z() + b->get(2).asFloat64();
                ignition::math::Pose3d new_pose(x, y, z, 0.0, 0.0, 0.0);
                yInfo() << "New pose:" << x << y << z;
                model->SetWorldPose(new_pose);

                physics::LinkPtr child = model->GetLink("red-ball::root_link");
                physics::LinkPtr parent = model->GetLink("world");
                if (child || parent)
                {
                    if (model->CreateJoint("fixed_to_ground", "fixed", parent, child))
                    {
                        yInfo() << "Added fixed_to_ground joint";
                    }
                }
            }
        }
    }

public:
    /**************************************************************************/
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr)
    {
        this->model = model;
        auto model_sdf = model->GetSDF();
        if( model_sdf->HasElement("pose") )
        {
            starting_pos = model_sdf->Get<ignition::math::Vector3d>("pose");
        }
        else
        {
            starting_pos = ignition::math::Vector3d(0.0, 0.0, 0.0);
        }

        port.open("/" + model->GetName() + "/mover:i");
        auto bind = boost::bind(&ModelMover::onWorldFrame, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
    }

    /**************************************************************************/
    virtual ~ModelMover()
    {
        if (!port.isClosed())
        {
            port.close();
        }
    }
};

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::ModelMover)
