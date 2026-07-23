#include "omip_core/joint_tracker/RigidJointFilter.h"

#include "omip_core/OMIPUtils.h"

#include <boost/math/special_functions/round.hpp>

using namespace omip;

RigidJointFilter::RigidJointFilter():
    JointFilter()
{
    // Initially the most probable joint type is rigid
    _measurements_likelihood = 1.0;
    _motion_memory_prior = 1.0;
}

RigidJointFilter::~RigidJointFilter()
{

}

RigidJointFilter::RigidJointFilter(const RigidJointFilter &rigid_joint) :
    JointFilter(rigid_joint)
{
}

void RigidJointFilter::initialize()
{
    JointFilter::initialize();
    // Initially the most probable joint type is rigid
    _measurements_likelihood = 1.0;
}

void RigidJointFilter::setMaxTranslationRigid(double max_trans)
{
    this->_rig_max_translation = max_trans;
}

void RigidJointFilter::setMaxRotationRigid(double max_rot)
{
    this->_rig_max_rotation = max_rot;
}

void RigidJointFilter::predictMeasurement()
{
    this->_predicted_delta_pose_in_rrbf = Twistd(0., 0., 0., 0., 0., 0.);

    Eigen::Isometry3d predicted_delta = se3Exp(this->_predicted_delta_pose_in_rrbf, 1e-20);
    Eigen::Isometry3d T_rrbf_srbf_t0 = se3Exp(this->_srb_initial_pose_in_rrbf, 1.0e-20);
    Eigen::Isometry3d T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = se3Log(T_rrbf_srbf_t_next, 1.0e-20);
}

void RigidJointFilter::estimateMeasurementHistoryLikelihood()
{
    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    double accumulated_error = 0.;

    double frame_counter = 0.;

    // Estimate the number of samples used for likelihood estimation
    size_t trajectory_length = this->_delta_poses_in_rrbf.size();
    size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);

    // Estimate the delta for the iterator over the number of samples
    // This makes that we take _likelihood_sample_num uniformly distributed over the whole trajectory, instead of the last _likelihood_sample_num
    double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);

    size_t current_idx = 0;

    // We test the last window_length motions
    for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
    {
        current_idx = boost::math::round(sample_idx*delta_idx_samples);
        Eigen::Isometry3d rb2_last_delta_relative_displ = se3Exp(this->_delta_poses_in_rrbf.at(current_idx), 1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.translation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.linear());

        Eigen::Vector3d rigid_joint_translation = Eigen::Vector3d(0.,0.,0.);
        Eigen::Isometry3d rb2_last_delta_relative_displ_rigid_hyp = se3Exp(Twistd(0.,
                                                                                     0.,
                                                                                     0.,
                                                                                     rigid_joint_translation.x(),
                                                                                     rigid_joint_translation.y(),
                                                                                     rigid_joint_translation.z()), 1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation_rigid_hyp = rb2_last_delta_relative_displ_rigid_hyp.translation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation_rigid_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_rigid_hyp.linear());

        // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
        double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_rigid_hyp).norm();

        if(translation_error > this->_rig_max_translation)
        {
            _motion_memory_prior = 0.0;
        }

        Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_rigid_hyp;
        double rotation_error_angle = se3Log(Eigen::Isometry3d(rotation_error), 1e-12).norm();


        if(rotation_error_angle > this->_rig_max_rotation)
        {
            _motion_memory_prior = 0.0;
        }

        accumulated_error += translation_error + fabs(rotation_error_angle);

        p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(translation_error/sigma_translation, 2)) *
                (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

        p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

        frame_counter++;
    }

    this->_measurements_likelihood = p_all_meas_given_model_params;
}

void RigidJointFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood*_motion_memory_prior;
}

TwistWithCovariance RigidJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    // The delta in the pose of the SRB is the delta in the pose of the RRB (its velocity!)
    Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    Twistd delta_srb_in_sf = delta_rrb_in_sf;

    TwistWithCovariance hypothesis;

    hypothesis.twist = delta_srb_in_sf;

    hypothesis.covariance = this->_rrb_vel_cov_in_sf*(this->_loop_period_ns/1e9);

    return hypothesis;
}

TwistWithCovariance RigidJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    // The delta in the pose of the SRB is the delta in the pose of the RRB (its velocity!)
    Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf;
    Twistd delta_srb_in_sf = delta_rrb_in_sf;

    TwistWithCovariance hypothesis;

    hypothesis.twist = delta_srb_in_sf;

    hypothesis.covariance = this->_rrb_vel_cov_in_sf;

    return hypothesis;
}

TwistWithCovariance RigidJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    Twistd rrb_next_pose_in_sf = se3Log(se3Exp(delta_rrb_in_sf, 1e-12)*se3Exp(this->_rrb_current_pose_in_sf, 1e-12), 1e-12);

    //Twistd rrb_next_pose_in_sf = this->_rrb_current_pose_in_sf + this->_rrb_current_vel_in_sf;
    Eigen::Isometry3d T_sf_rrbf_next = se3Exp(rrb_next_pose_in_sf, 1e-12);
    Eigen::Isometry3d T_rrbf_srbf_next = se3Exp(this->_srb_predicted_pose_in_rrbf, 1e-12);

    Eigen::Isometry3d T_sf_srbf_next = T_rrbf_srbf_next*T_sf_rrbf_next;

    Twistd srb_next_pose_in_sf = se3Log(T_sf_srbf_next, 1e-12);

    TwistWithCovariance hypothesis;

    hypothesis.twist = srb_next_pose_in_sf;

    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf;
    hypothesis.covariance = new_pose_covariance;

    return hypothesis;
}

std::vector<DebugGeometry> RigidJointFilter::getJointDebugGeometryInRRBFrame() const
{
    std::vector<DebugGeometry> rigid_markers;
    // CONNECTION MARKER ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    DebugGeometry connection_marker;
    connection_marker.ns = "kinematic_structure";
    connection_marker.action = DebugGeometryAction::Add;
    connection_marker.type = DebugGeometryType::Arrow;
    connection_marker.id = 3 * this->_joint_id;
    connection_marker.scale = Eigen::Vector3d(0.02f, 0.f, 0.f);
    connection_marker.color = Eigen::Vector4d(0.f, 0.f, 1.f, 1.f);
    // Estimate position from supporting features:
    Eigen::Isometry3d current_ref_pose = se3Exp(this->_rrb_current_pose_in_sf, 1e-12);
    Eigen::Vector3d second_centroid_relative_to_ref_body = current_ref_pose.inverse() * this->_srb_centroid_in_sf;
    connection_marker.points.push_back(second_centroid_relative_to_ref_body);
    // The markers are now defined wrt to the reference frame and I want the rigid joint marker to go from the centroid of
    // the second rigid body to the centroid of the reference rigid body
    Eigen::Vector3d first_centroid_relative_to_ref_body = current_ref_pose.inverse() * this->_rrb_centroid_in_sf;
    connection_marker.points.push_back(first_centroid_relative_to_ref_body);

    rigid_markers.push_back(connection_marker);

    // Delete other markers
    DebugGeometry empty_marker;
    empty_marker.position = Eigen::Vector3d(0., 0., 0.);
    empty_marker.type = DebugGeometryType::Sphere;
    empty_marker.action = DebugGeometryAction::Delete;
    empty_marker.scale = Eigen::Vector3d(0.0001, 0.0001, 0.0001); //Using start and end points, scale.x is the radius of the array body
    empty_marker.color = Eigen::Vector4d(0.0, 0.0, 1.0, 0.3);
    empty_marker.ns = "kinematic_structure";
    empty_marker.id = 3 * this->_joint_id + 1;

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3 * this->_joint_id + 2;
    rigid_markers.push_back(empty_marker);

    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 1;

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 2;

    rigid_markers.push_back(empty_marker);

    return rigid_markers;
}

JointFilterType RigidJointFilter::getJointFilterType() const
{
    return RIGID_JOINT;
}

std::string RigidJointFilter::getJointFilterTypeStr() const
{
    return std::string("RigidJointFilter");
}
