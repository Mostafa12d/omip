#include "omip_core/joint_tracker/DisconnectedJointFilter.h"

#include <random>

using namespace omip;

DisconnectedJointFilter::DisconnectedJointFilter() :
    JointFilter()
{
    _unnormalized_model_probability = 0.8;
}

DisconnectedJointFilter::~DisconnectedJointFilter()
{

}

DisconnectedJointFilter::DisconnectedJointFilter(const DisconnectedJointFilter &disconnected_joint) :
    JointFilter(disconnected_joint)
{
}

void DisconnectedJointFilter::initialize()
{

}

double DisconnectedJointFilter::getProbabilityOfJointFilter() const
{
    return (_unnormalized_model_probability / this->_normalizing_term);//this->_measurements_likelihood;
}

TwistWithCovariance DisconnectedJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    // We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Twistd srb_delta_pose_in_sf_next = Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    TwistWithCovariance hypothesis;

    hypothesis.twist = srb_delta_pose_in_sf_next;

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance(i, j) = 1.0e6;
            }else
            {
                hypothesis.covariance(i, j) = 1.0e3;
            }
        }
    }

    return hypothesis;
}

TwistWithCovariance DisconnectedJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    // We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Twistd srb_delta_pose_in_sf_next = Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    TwistWithCovariance hypothesis;

    hypothesis.twist = srb_delta_pose_in_sf_next;

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance(i, j) = 1.0e6;
            }else
            {
                hypothesis.covariance(i, j) = 1.0e3;
            }
        }
    }

    return hypothesis;
}

TwistWithCovariance DisconnectedJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    // If the two rigid bodies are disconnecte, the pose of the second rigid body is INDEPENDENT of the motion of the reference rigid body

    // OPTION1: This prediction is the same as predicting in the rigid body level using velocity
    //Twistd delta_motion_srb = this->_srb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    //Twistd srb_pose_in_sf_next = se3Log(se3Exp(delta_motion_srb,1e-12)*se3Exp(this->_srb_current_pose_in_sf,1e-12),1e-12);

    // OPTION2: We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Twistd srb_pose_in_sf_next = Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    TwistWithCovariance hypothesis;
    hypothesis.twist = srb_pose_in_sf_next;

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance(i, j) = 1.0e6;
            }else
            {
                hypothesis.covariance(i, j) = 1.0e3;
            }
        }
    }

    return hypothesis;
}

std::vector<DebugGeometry> DisconnectedJointFilter::getJointDebugGeometryInRRBFrame() const
{
    // Delete other markers
    std::vector<DebugGeometry> empty_vector;
    DebugGeometry empty_marker;
    empty_marker.position = Eigen::Vector3d(0., 0., 0.);
    empty_marker.type = DebugGeometryType::Sphere;
    empty_marker.action = DebugGeometryAction::Delete;
    empty_marker.scale = Eigen::Vector3d(0.0001, 0.0001, 0.0001); //Using start and end points, scale.x is the radius of the array body
    empty_marker.color = Eigen::Vector4d(0.0, 0.0, 1.0, 0.3);
    empty_marker.ns = "kinematic_structure";
    empty_marker.id = 3 * this->_joint_id;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);
    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);

    return empty_vector;
}

JointFilterType DisconnectedJointFilter::getJointFilterType() const
{
    return DISCONNECTED_JOINT;
}

std::string DisconnectedJointFilter::getJointFilterTypeStr() const
{
    return std::string("DisconnectedJointFilter");
}
