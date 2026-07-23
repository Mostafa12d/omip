#include "omip_core/rb_tracker/StaticEnvironmentFilter.h"
#include "omip_core/OMIPUtils.h"
#include "omip_core/Log.h"
#include <boost/foreach.hpp>

using namespace omip;

StaticEnvironmentFilter::StaticEnvironmentFilter(double loop_period_ns, FeaturesDataBase::Ptr feats_database,
                                                 double environment_motion_threshold) :
    RBFilter()
{
    this->_loop_period_ns = loop_period_ns;
    this->_id = 0;
    this->_features_database = feats_database;
    this->_estimation_error_threshold = environment_motion_threshold;

    this->_tf_epsilon_linear = 1e-8;
    this->_tf_epsilon_angular = 1.7e-9;
    this->_max_iterations = 100;

    this->_pose = Eigen::Matrix4d::Identity();
    this->_velocity = Twistd(0.,0.,0.,0.,0.,0);

    this->_motion_constraint = NO_MOTION;

    this->_computation_type = STATIC_ENVIRONMENT_EKF_TRACKER;

    // To avoid segmentation fault because the features are as old as the rigid body
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
}

void StaticEnvironmentFilter::Init()
{
    RBFilter::Init();
    _pose_covariance.setZero();
    _velocity_covariance.setZero();
}

void StaticEnvironmentFilter::predictState(double time_interval_ns)
{
    if(this->_motion_constraint == NO_MOTION)
    {
        this->_predicted_pose_vh = _pose;
        this->_predicted_pose_bh = _pose;
        return;
    }

    switch(this->_computation_type)
    {

    case STATIC_ENVIRONMENT_ICP_TRACKER:
    {
        Twistd predicted_delta_pose_vh = (this->_velocity*time_interval_ns/1e9);
        this->_predicted_pose_vh = se3Exp(predicted_delta_pose_vh, 1e-12).matrix()*_pose;
        this->_predicted_pose_bh = _pose;
    }
        break;
    case STATIC_ENVIRONMENT_EKF_TRACKER:
    {
        RBFilter::predictState(time_interval_ns);
    }
        break;

    default:
        OMIP_ERROR_STREAM_NAMED("StaticEnvironmentFilter.predictState","Wrong StaticEnvironment computation_type!");
        break;
    }
}

void StaticEnvironmentFilter::correctState()
{
    // The problem is that the StaticEnvironmentFilter is created at the beggining when only one location per feature is available
    // There is no way to correct the state with only one location per feature
    static bool first_time = true;
    if(first_time)
    {
        this->_trajectory.push_back(this->_pose);
        first_time = false;
        return;
    }

    if(this->_motion_constraint == NO_MOTION)
    {
        this->_velocity = Twistd(0,0,0,0,0,0);
        this->_trajectory.push_back(_pose);
        return;
    }

    switch(this->_computation_type)
    {

    case STATIC_ENVIRONMENT_ICP_TRACKER:
    {
        // I follow the convention of "Introduction to robotics" by Craig
        // The first fram is the reference (left upper index) and the second frame is the referred (left lower index)
        // se -> static environment
        // set -> static environment at time t
        // Example: setnext_set_T is describes the frame "static environment at time t" wrt the frame "static environment at time tnext"
        Eigen::Isometry3d set_setnext_Tf;
        this->estimateDeltaMotion(this->_supporting_features_ids, set_setnext_Tf);

        Eigen::Matrix4d set_setnext_T = set_setnext_Tf.matrix();

        Twistd set_setnext_Twist;
        TransformMatrix2Twist(set_setnext_T, set_setnext_Twist);

        this->_velocity = set_setnext_Twist/(this->_loop_period_ns/1e9);

        // Accumulate the new delta into the absolute pose of the static environment wrt the camera
        this->_pose = this->_pose*set_setnext_T;

        this->_trajectory.push_back(this->_pose);

        constrainMotion();
    }
        break;
    case STATIC_ENVIRONMENT_EKF_TRACKER:
    {
        RBFilter::correctState();

        constrainMotion();
    }
        break;

    default:
        OMIP_ERROR_STREAM_NAMED("StaticEnvironmentFilter.correctState","Wrong StaticEnvironment computation_type!");
        break;
    }
}

void StaticEnvironmentFilter::setMotionConstraint(int motion_constraint)
{
    this->_motion_constraint = (MotionConstraint)motion_constraint;
}

void StaticEnvironmentFilter::setComputationType(static_environment_tracker_t computation_type)
{
    this->_computation_type = computation_type;
}

void StaticEnvironmentFilter::addSupportingFeature(Feature::Id supporting_feat_id)
{
    this->_supporting_features_ids.push_back(supporting_feat_id);

    Feature::Location predicted_location_velocity;
    Feature::Location predicted_location_brake;
    FeaturePCLwc predicted_feature_pcl;
    // Predict feature location based on velocity hypothesis

    predicted_location_velocity = this->_features_database->getFeatureLastLocation(supporting_feat_id);
    LocationAndId2FeaturePCLwc(predicted_location_velocity, supporting_feat_id, predicted_feature_pcl);
    this->_predicted_measurement_pc_vh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_vh[supporting_feat_id] = predicted_location_velocity;

    // Predict feature location based on brake hypothesis (same as last)
    predicted_location_brake = this->_features_database->getFeatureLastLocation(supporting_feat_id);
    LocationAndId2FeaturePCLwc(predicted_location_brake, supporting_feat_id, predicted_feature_pcl);
    this->_predicted_measurement_pc_bh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_bh[supporting_feat_id] = predicted_location_brake;
}

void StaticEnvironmentFilter::estimateDeltaMotion(std::vector<Feature::Id>& supporting_features_ids, Eigen::Isometry3d& previous_current_Tf)
{
    // Prepare the 2 point clouds with the locations of the features in current and previous frames
    pcl::PointCloud<pcl::PointXYZ> previous_locations, current_locations;
    BOOST_FOREACH(Feature::Id supporting_feat_id, supporting_features_ids)
    {
        if (this->_features_database->isFeatureStored(supporting_feat_id) && this->_features_database->getFeatureAge(supporting_feat_id) > 2)
        {
            Feature::Location previous_location = this->_features_database->getFeatureNextToLastLocation(supporting_feat_id);
            Feature::Location current_location = this->_features_database->getFeatureLastLocation(supporting_feat_id);
            previous_locations.push_back(pcl::PointXYZ(previous_location.get<0>(), previous_location.get<1>() ,previous_location.get<2>()));
            current_locations.push_back(pcl::PointXYZ(current_location.get<0>(), current_location.get<1>() ,current_location.get<2>()));
        }
    }

    if (previous_locations.size() ==0)
    {
        OMIP_ERROR_STREAM_NAMED("StaticEnvironmentFilter.estimateDeltaMotion","There are no features supporting the static environment!");
        previous_current_Tf.setIdentity();
    }else{
        iterativeICP(previous_locations, current_locations, previous_current_Tf);
    }
}

void StaticEnvironmentFilter::iterativeICP( pcl::PointCloud<pcl::PointXYZ>& previous_locations,
                                            pcl::PointCloud<pcl::PointXYZ>& current_locations, Eigen::Isometry3d& previous_current_Tf)
{
    // initialize the result transform
    Eigen::Matrix4f previous_current_T;
    previous_current_T.setIdentity();

    // create indices
    std::vector<int> previous_indices, current_indices;
    for(int i=0; i<previous_locations.size();i++)
    {
        previous_indices.push_back(i);
        current_indices.push_back(i);
    }

    for (int iteration = 0; iteration < this->_max_iterations; ++iteration)
    {
        // estimate transformation
        Eigen::Matrix4f previous_current_T_new;
        _svdecomposer.estimateRigidTransformation (previous_locations, previous_indices,
                                                   current_locations, current_indices,
                                                   previous_current_T_new);

        // transform
        pcl::transformPointCloud(previous_locations, previous_locations, previous_current_T_new);

        // accumulate incremental tf
        previous_current_T = previous_current_T_new * previous_current_T;

        // check for convergence
        double linear, angular;

        previous_current_Tf = Eigen::Isometry3d(previous_current_T.cast<double>());
        linear = previous_current_Tf.translation().norm();
        double trace = previous_current_Tf.linear().trace();
        angular = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
        if (linear  < this->_tf_epsilon_linear && angular < this->_tf_epsilon_angular)
        {
            break;
        }
    }
}

void StaticEnvironmentFilter::constrainMotion()
{
    // The last estimated and unconstrained pose is contained in the variable _pose = cam_setnext_T
    //
    // This function was already a no-op in the original ROS build: every
    // branch of its per-motion-constraint logic (NO_ROLL_PITCH, NO_ROTATION,
    // ROBOT_XY_BASELINK_PLANE, ...) was commented out in the source we
    // ported from (see git history of rb_tracker/src/StaticEnvironmentFilter.cpp
    // prior to this port) — only the ROBOT_XY_BASELINK_PLANE branch had any
    // real logic (querying a "/base_link" <-> "/camera_rgb_optical_frame"
    // tf transform), and even that was dead/commented, never active. Ported
    // as the same no-op per PORTING_NOTES.md Phase 3 rather than reviving
    // never-finished code.
}
