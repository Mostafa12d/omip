#include "omip_core/joint_tracker/PrismaticJointFilter.h"

#include <Eigen/Geometry>

#include <sstream>
#include <iomanip>

#include <Eigen/Eigenvalues>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/special_functions/round.hpp>

#include "omip_core/OMIPUtils.h"
#include "omip_core/Log.h"

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

// Dimensions of the system state of the filter that tracks a prismatic joint: orientation (2 values), joint variable, and joint velocity
#define PRISM_STATE_DIM 4
#define MEAS_DIM 6

/**
 * EKF internal state:
 *
 * x(1) =  PrismJointOrientation_phi
 * x(2) =  PrismJointOrientation_theta
 * PrismJointOrientation is represented in spherical coords
 * NOTE: I try to keep theta between 0 and pi and phi between 0 and two pi
 * x(4) = PrismJointVariable
 * x(5) = PrismJointVariable_d
 *
 * EKF measurement:
 *
 * m(1) = TwistLinearPart_x
 * m(2) = TwistLinearPart_y
 * m(3) = TwistLinearPart_z
 * m(4) = TwistAngularPart_x
 * m(5) = TwistAngularPart_y
 * m(6) = TwistAngularPart_z
 */

PrismaticJointFilter::PrismaticJointFilter():
    JointFilter(),
    _sys_PDF(NULL),
    _sys_MODEL(NULL),
    _meas_PDF(NULL),
    _meas_MODEL(NULL),
    _ekf(NULL),
    _sigma_delta_meas_uncertainty_linear(-1)
{

}

void PrismaticJointFilter::setCovarianceDeltaMeasurementLinear(double sigma_delta_meas_uncertainty_linear)
{
    this->_sigma_delta_meas_uncertainty_linear = sigma_delta_meas_uncertainty_linear;
}

void PrismaticJointFilter::initialize()
{
    JointFilter::initialize();

    this->_joint_orientation = Eigen::Vector3d(
                this->_current_delta_pose_in_rrbf.vx(),
                this->_current_delta_pose_in_rrbf.vy(),
                this->_current_delta_pose_in_rrbf.vz());
    this->_joint_state = this->_joint_orientation.norm();

    this->_joint_states_all.push_back(this->_joint_state);
    //this->_joint_velocity = this->_joint_state/(this->_loop_period_ns/1e9);
    // Setting it to 0 is better.
    // The best approximation would be to (this->_rev_variable/num_steps_to_rev_variable)/(this->_loop_period_ns/1e9)
    // but we don't know how many steps passed since we estimated the first time the joint variable
    this->_joint_velocity = 0.0;
    this->_joint_orientation.normalize();

    // The position of the prismatic joint is the centroid of the features that belong to the second rigid body (in RBF)
    Eigen::Isometry3d current_ref_pose = se3Exp(this->_rrb_current_pose_in_sf, 1e-12);
    this->_joint_position = current_ref_pose.inverse() * this->_srb_centroid_in_sf;

    this->_initializeSystemModel();
    this->_initializeMeasurementModel();
    this->_initializeEKF();
}

void PrismaticJointFilter::_initializeSystemModel()
{
    // create SYSTEM MODEL
    Matrix A(PRISM_STATE_DIM, PRISM_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(3, 4) = this->_loop_period_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    ColumnVector sys_noise_MU(PRISM_STATE_DIM);
    sys_noise_MU = 0;

    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_jv* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jvd*(this->_loop_period_ns/1e9);

    // Initialize System Model
    Gaussian system_uncertainty_PDF(sys_noise_MU, sys_noise_COV);
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( A, system_uncertainty_PDF);
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( this->_sys_PDF);
}

void PrismaticJointFilter::_initializeMeasurementModel()
{
    // create MEASUREMENT MODEL
    ColumnVector meas_noise_MU(MEAS_DIM);
    meas_noise_MU = 0.0;
    SymmetricMatrix meas_noise_COV(MEAS_DIM);
    meas_noise_COV = 0.0;
    for (unsigned int i = 1; i <= MEAS_DIM; i++)
        meas_noise_COV(i, i) = this->_sigma_meas_noise;

    Gaussian meas_uncertainty_PDF(meas_noise_MU, meas_noise_COV);

    this->_meas_PDF = new NonLinearPrismaticMeasurementPdf(meas_uncertainty_PDF);
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty(this->_meas_PDF);
}

void PrismaticJointFilter::_initializeEKF()
{
    ColumnVector prior_MU(PRISM_STATE_DIM);
    prior_MU = 0.0;

    SymmetricMatrix prior_COV(PRISM_STATE_DIM);
    prior_COV = 0.0;

    prior_MU(1) = atan2(this->_joint_orientation.y() , this->_joint_orientation.x());

    // NOTE: I make phi between 0 and 2pi
    //    if(prior_MU(1) < 0.0)
    //    {
    //        prior_MU(1) += 2*M_PI;
    //    }

    prior_MU(2) = acos(this->_joint_orientation.z());
    prior_MU(3) = this->_joint_state;
    prior_MU(4) = this->_joint_velocity;

    OMIP_INFO_STREAM_NAMED( "PrismaticJointFilter::_initializeEKF",
                           "Prismatic initial state (phi, theta, jv, jv_dot): " << prior_MU(1) << " " << prior_MU(2) << " " << prior_MU(3) << " " << prior_MU(4));

    for (int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        prior_COV(i, i) = this->_prior_cov_vel;
    }

    Gaussian prior_PDF(prior_MU, prior_COV);

    this->_ekf = new ExtendedKalmanFilter(&prior_PDF);
}

PrismaticJointFilter::~PrismaticJointFilter()
{
    if (this->_sys_PDF)
    {
        delete this->_sys_PDF;
        this->_sys_PDF = NULL;
    }
    if (this->_sys_MODEL)
    {
        delete this->_sys_MODEL;
        this->_sys_MODEL = NULL;
    }
    if (this->_meas_PDF)
    {
        delete this->_meas_PDF;
        this->_meas_PDF = NULL;
    }
    if (this->_meas_MODEL)
    {
        delete this->_meas_MODEL;
        this->_meas_MODEL = NULL;
    }
    if (this->_ekf)
    {
        delete this->_ekf;
        this->_ekf = NULL;
    }
}

PrismaticJointFilter::PrismaticJointFilter(const PrismaticJointFilter &prismatic_joint) :
    JointFilter(prismatic_joint)
{
    this->_sigma_delta_meas_uncertainty_linear = prismatic_joint._sigma_delta_meas_uncertainty_linear;
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( *(prismatic_joint._sys_PDF));
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( *(prismatic_joint._sys_MODEL));
    this->_meas_PDF = new NonLinearPrismaticMeasurementPdf( *(prismatic_joint._meas_PDF));
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty( *(prismatic_joint._meas_MODEL));
    this->_ekf = new ExtendedKalmanFilter(*(prismatic_joint._ekf));
}

void PrismaticJointFilter::predictState(double time_interval_ns)
{
    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_jv* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jvd*(time_interval_ns/1e9);

    // Estimate the new updating matrix which also depends on the time elapsed between the previous and the current measurement
    // x(t+1) = x(t) + v(t) * delta_t
    Matrix A(PRISM_STATE_DIM, PRISM_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(3, 4) = time_interval_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    this->_sys_PDF->MatrixSet(0, A);
    this->_sys_PDF->AdditiveNoiseSigmaSet(sys_noise_COV);
    this->_ekf->Update(this->_sys_MODEL);
}

TwistWithCovariance PrismaticJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    Eigen::Matrix<double, 6, 6> adjoint;
    computeAdjoint(this->_rrb_current_pose_in_sf, adjoint);
    Twistd predicted_delta_pose_in_sf = adjoint*this->_predicted_delta_pose_in_rrbf;

    TwistWithCovariance hypothesis;

    hypothesis.twist = predicted_delta_pose_in_sf;

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             hypothesis.covariance(i, j) = measurement_cov(i+1,j+1);
        }
    }

    return hypothesis;
}

TwistWithCovariance PrismaticJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    Eigen::Matrix<double, 6, 6> adjoint;
    computeAdjoint(this->_rrb_current_pose_in_sf, adjoint);
    Twistd predicted_delta_pose_in_sf = adjoint*(this->_predicted_delta_pose_in_rrbf/(_loop_period_ns/1e9));

    TwistWithCovariance hypothesis;

    hypothesis.twist = predicted_delta_pose_in_sf;

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             hypothesis.covariance(i, j) = measurement_cov(i+1,j+1);
        }
    }

    return hypothesis;
}

void PrismaticJointFilter::predictMeasurement()
{
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    OMIP_DEBUG_STREAM_NAMED( "PrismaticJointFilter::UpdateJointParameters",
                            "Prismatic state after state update: " << state_updated_state(1) << " " << state_updated_state(2) << " " << state_updated_state(3) << " " << state_updated_state(4));

    ColumnVector predicted_delta_pose_in_rrbf = this->_meas_MODEL->PredictionGet(empty, state_updated_state);

    this->_predicted_delta_pose_in_rrbf = Twistd( predicted_delta_pose_in_rrbf(4), predicted_delta_pose_in_rrbf(5),
                                                         predicted_delta_pose_in_rrbf(6), predicted_delta_pose_in_rrbf(1),
                                                         predicted_delta_pose_in_rrbf(2), predicted_delta_pose_in_rrbf(3));

    Eigen::Isometry3d predicted_delta = se3Exp(this->_predicted_delta_pose_in_rrbf, 1e-20);
    Eigen::Isometry3d T_rrbf_srbf_t0 = se3Exp(this->_srb_initial_pose_in_rrbf, 1.0e-20);
    Eigen::Isometry3d T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = se3Log(T_rrbf_srbf_t_next, 1.0e-20);
}

void PrismaticJointFilter::correctState()
{
    // New 26.8.2016 -> There is small rotations in the reference frame that cause the prismatic joint to rotate
    // We eliminate this: we search for the closest motion without rotation that resembles the relative motion
    Eigen::Matrix4d  _srb_current_pose_in_rrbf_matrix;
    Twist2TransformMatrix( _srb_current_pose_in_rrbf, _srb_current_pose_in_rrbf_matrix);

    //Then set the rotation to the identity
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
                _srb_current_pose_in_rrbf_matrix(i,j) = 1;
            }else{
                _srb_current_pose_in_rrbf_matrix(i,j) = 0;
            }
        }
    }

    Twistd  _srb_current_pose_in_rrbf_no_rot;
    TransformMatrix2Twist(_srb_current_pose_in_rrbf_matrix, _srb_current_pose_in_rrbf_no_rot);

    Twistd _current_delta_pose_in_rrbf_no_rot = se3Log(se3Exp(_srb_current_pose_in_rrbf_no_rot, 1e-12)*se3Exp(_srb_initial_pose_in_rrbf, 1e-12).inverse(), 1e-12);

    ColumnVector rb2_measured_delta_relative_pose_cv(MEAS_DIM);
    rb2_measured_delta_relative_pose_cv = 0.;
    rb2_measured_delta_relative_pose_cv(1) = _current_delta_pose_in_rrbf_no_rot.vx();
    rb2_measured_delta_relative_pose_cv(2) = _current_delta_pose_in_rrbf_no_rot.vy();
    rb2_measured_delta_relative_pose_cv(3) = _current_delta_pose_in_rrbf_no_rot.vz();
    rb2_measured_delta_relative_pose_cv(4) = _current_delta_pose_in_rrbf_no_rot.rx();
    rb2_measured_delta_relative_pose_cv(5) = _current_delta_pose_in_rrbf_no_rot.ry();
    rb2_measured_delta_relative_pose_cv(6) = _current_delta_pose_in_rrbf_no_rot.rz();

    // Update the uncertainty on the measurement
    // NEW: The uncertainty on the measurement (the delta motion of the second rigid body wrt the reference rigid body) will be large if the measurement
    // is small and small if the measurement is large
    double meas_uncertainty_factor = 1.0 / (1.0 - exp(-this->_current_delta_pose_in_rrbf.norm()/this->_sigma_delta_meas_uncertainty_linear));

    // Truncate the factor
    meas_uncertainty_factor = std::min(meas_uncertainty_factor, 1e6);

    SymmetricMatrix current_delta_pose_cov_in_rrbf(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            current_delta_pose_cov_in_rrbf(i + 1, j + 1) = _current_delta_pose_cov_in_rrbf(i, j);
        }
    }
    this->_meas_PDF->AdditiveNoiseSigmaSet(current_delta_pose_cov_in_rrbf * meas_uncertainty_factor);

    this->_ekf->Update(this->_meas_MODEL, rb2_measured_delta_relative_pose_cv);

    ColumnVector updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    // The joint is defined in the space of the relative motion
    this->_joint_orientation(0) = sin(updated_state(2)) * cos(updated_state(1));
    this->_joint_orientation(1) = sin(updated_state(2)) * sin(updated_state(1));
    this->_joint_orientation(2) = cos(updated_state(2));

    SymmetricMatrix updated_uncertainty = this->_ekf->PostGet()->CovarianceGet();

    this->_joint_state = updated_state(3);
    this->_uncertainty_joint_state = updated_uncertainty(3,3);
    this->_joint_velocity = updated_state(4);
    this->_uncertainty_joint_velocity = updated_uncertainty(4,4);

    this->_joint_orientation_phi = updated_state(1);
    this->_joint_orientation_theta = updated_state(2);
    this->_uncertainty_joint_orientation_phitheta(0,0) = updated_uncertainty(1, 1);
    this->_uncertainty_joint_orientation_phitheta(0,1) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,0) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,1) = updated_uncertainty(2, 2);
}

void PrismaticJointFilter::estimateMeasurementHistoryLikelihood()
{
    double accumulated_error = 0.;

    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    double phi = state_updated_state(1);
    double theta = state_updated_state(2);

    this->_joint_states_all.push_back(state_updated_state(3));

    // This vector is in the ref RB with the initial relative transformation to the second RB
    Eigen::Vector3d prism_joint_translation_unitary = Eigen::Vector3d(cos(phi) * sin(theta), sin(phi) * sin(theta), cos(theta));
    prism_joint_translation_unitary.normalize();

    double frame_counter = 0.;
    size_t trajectory_length = this->_delta_poses_in_rrbf.size();
    size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);
    double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);
    size_t current_idx = 0;

    double max_norm_of_deltas = 0;


    // Estimation of the quality of the parameters of the prismatic joint
    // If the joint is prismatic and the parameters are accurate, the joint axis orientation should not change over time
    // That means that the current orientation, multiplied by the amount of prismatic displacement at each time step, should provide the delta in the relative
    // pose between ref and second rb at each time step
    // We test amount_samples of the relative trajectory
    for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
    {
        current_idx = boost::math::round(sample_idx*delta_idx_samples);
        Eigen::Isometry3d rb2_last_delta_relative_displ = se3Exp(this->_delta_poses_in_rrbf.at(current_idx), 1e-12);

        max_norm_of_deltas = std::max(this->_delta_poses_in_rrbf.at(current_idx).norm(), max_norm_of_deltas);

        Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.translation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.linear());

        Eigen::Vector3d prism_joint_translation = this->_joint_states_all.at(current_idx) * prism_joint_translation_unitary;
        Eigen::Isometry3d rb2_last_delta_relative_displ_prism_hyp = se3Exp(Twistd(0.,
                                                                                     0.,
                                                                                     0.,
                                                                                     prism_joint_translation.x(),
                                                                                     prism_joint_translation.y(),
                                                                                     prism_joint_translation.z()), 1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation_prism_hyp = rb2_last_delta_relative_displ_prism_hyp.translation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation_prism_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_prism_hyp.linear());

        // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
        double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_prism_hyp).norm();
        // Actually both rotations should be zero because a prismatic joint constraints the rotation
        Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_prism_hyp;
        double rotation_error_angle = se3Log(Eigen::Isometry3d(rotation_error), 1e-12).norm();


        accumulated_error += translation_error + fabs(rotation_error_angle);

        p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(translation_error/sigma_translation, 2)) *
                (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

        p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

        frame_counter++;
    }

    if(frame_counter != 0)
    {
        this->_measurements_likelihood = p_all_meas_given_model_params;
    }else{
        this->_measurements_likelihood = 1e-5;
    }
}

void PrismaticJointFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood;
}

TwistWithCovariance PrismaticJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
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

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    Eigen::Matrix<double,6,6> measurement_cov_eigen;
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            measurement_cov_eigen(i,j) = measurement_cov(i+1,j+1);
        }
    }
    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> tranformed_cov;
    adjointXcovXadjointT(_rrb_current_pose_in_sf, measurement_cov_eigen, tranformed_cov);
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf + tranformed_cov;
    hypothesis.covariance = new_pose_covariance;

    return hypothesis;
}

std::vector<DebugGeometry> PrismaticJointFilter::getJointDebugGeometryInRRBFrame() const
{
    // The class variable _prism_joint_orientation (also _uncertainty_o_phi and _uncertainty_o_theta) are defined in the frame of the
    // ref RB with the initial relative transformation to the second RB
    // We want the variables to be in the ref RB frame, without the initial relative transformation to the second RB
    Eigen::Vector3d prism_joint_ori_in_ref_rb = this->_joint_orientation;

    std::vector<DebugGeometry> prismatic_markers;
    // AXIS MARKER 1 -> The axis ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    DebugGeometry axis_orientation_marker;
    axis_orientation_marker.ns = "kinematic_structure";
    axis_orientation_marker.action = DebugGeometryAction::Add;
    axis_orientation_marker.type = DebugGeometryType::Arrow;
    axis_orientation_marker.id = 3 * this->_joint_id;
    axis_orientation_marker.scale = Eigen::Vector3d(JOINT_AXIS_AND_VARIABLE_MARKER_RADIUS, 0.f, 0.f);
    axis_orientation_marker.color = Eigen::Vector4d(0.f, 1.f, 0.f, 1.f);
    // Estimate position from supporting features:
    Eigen::Vector3d second_centroid_relative_to_ref_body = this->getJointPositionInRRBFrame();
    Eigen::Vector3d position1 = second_centroid_relative_to_ref_body - this->_joint_state * prism_joint_ori_in_ref_rb;
    axis_orientation_marker.points.push_back(position1);
    Eigen::Vector3d position2 = second_centroid_relative_to_ref_body + this->_joint_state * prism_joint_ori_in_ref_rb;
    axis_orientation_marker.points.push_back(position2);

    prismatic_markers.push_back(axis_orientation_marker);

    // AXIS MARKER 2 -> Proportional to joint state ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    axis_orientation_marker.points.clear();
    axis_orientation_marker.id = 3 * this->_joint_id + 1;
    axis_orientation_marker.scale.x() = JOINT_AXIS_MARKER_RADIUS;
    // Estimate position from supporting features:
    Eigen::Vector3d position12 = second_centroid_relative_to_ref_body - 100 * prism_joint_ori_in_ref_rb;
    axis_orientation_marker.points.push_back(position12);
    Eigen::Vector3d position22 = second_centroid_relative_to_ref_body + 100 * prism_joint_ori_in_ref_rb;
    axis_orientation_marker.points.push_back(position22);

    prismatic_markers.push_back(axis_orientation_marker);

    // AXIS MARKER 3 -> Text with the joint state ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    axis_orientation_marker.points.clear();
    axis_orientation_marker.id = 3 * this->_joint_id + 2;
    axis_orientation_marker.type = DebugGeometryType::TextViewFacing;
    axis_orientation_marker.scale.z() = JOINT_VALUE_TEXT_SIZE;
    std::ostringstream oss_joint_value;
    oss_joint_value << std::fixed<< std::setprecision(1) << 100*this->_joint_state;
    axis_orientation_marker.text = oss_joint_value.str() + std::string(" cm");
    axis_orientation_marker.position = second_centroid_relative_to_ref_body;
    axis_orientation_marker.orientation = Eigen::Quaterniond::Identity();

    prismatic_markers.push_back(axis_orientation_marker);

    // UNCERTAINTY MARKERS ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // This first marker is just to match the number of markers of the revolute joint, but prismatic joint has no position
    DebugGeometry axis_position_uncertainty_marker;
    axis_position_uncertainty_marker.position = second_centroid_relative_to_ref_body;
    axis_position_uncertainty_marker.ns = "kinematic_structure_uncertainty";
    axis_position_uncertainty_marker.type = DebugGeometryType::Sphere;
    axis_position_uncertainty_marker.action = DebugGeometryAction::Delete;
    axis_position_uncertainty_marker.id = 3 * this->_joint_id ;

    prismatic_markers.push_back(axis_position_uncertainty_marker);

    DebugGeometry prism_axis_unc_cone1;
    prism_axis_unc_cone1.type = DebugGeometryType::MeshResource;
    prism_axis_unc_cone1.action = DebugGeometryAction::Add;
    prism_axis_unc_cone1.mesh_resource = "package://joint_tracker/meshes/cone.stl";
    prism_axis_unc_cone1.position = second_centroid_relative_to_ref_body;

    // NOTE:
    // Estimation of the uncertainty cones -----------------------------------------------
    // We estimate the orientation of the prismatic axis in spherical coordinates (r=1 always)
    // We estimate phi: angle from the x axis to the projection of the prismatic joint axis to the xy plane
    // We estimate theta: angle from the z axis to the prismatic joint axis
    // [TODO: phi and theta are in the reference rigid body. Do we need to transform it (adding uncertainty) to the reference frame?]
    // The covariance of phi and theta (a 2x2 matrix) gives us the uncertainty of the orientation of the joint
    // If we look from the joint axis, we would see an ellipse given by this covariance matrix [http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/]
    // But in RVIZ we can only set the scale of our cone mesh in x and y, not in a different axis
    // The first thing is then to estimate the direction of the major and minor axis of the ellipse and their size
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(this->_uncertainty_joint_orientation_phitheta);

    // The sizes of the major and minor axes of the ellipse are given by the eigenvalues and the chi square distribution P(x<critical_value) = confidence_value
    // For 50% of confidence on the cones shown
    double confidence_value = 0.5;
    boost::math::chi_squared chi_sq_dist(2);
    double critical_value = boost::math::quantile(chi_sq_dist, confidence_value);
    double major_axis_length = 2*eigensolver.eigenvalues()[1]*std::sqrt(critical_value);
    double minor_axis_length = 2*eigensolver.eigenvalues()[0]*std::sqrt(critical_value);

    // If z is pointing in the direction of the joint, the angle between the x axis and the largest axis of the ellipse is arctg(v1_y/v1_x) where v1 is the eigenvector of
    // largest eigenvalue (the last column in the matrix returned by eigenvectors() in eigen library):
    double alpha = atan2(eigensolver.eigenvectors().col(1)[1],eigensolver.eigenvectors().col(1)[0]);
    // We create a rotation around the z axis to align the ellipse to have the major axis aligned to the x-axis (we UNDO the rotation of the ellipse):
    Eigen::AngleAxisd init_rot(-alpha, Eigen::Vector3d::UnitZ());

    // Now I need to rotate the mesh so that:
    // 1) The z axis of the mesh points in the direction of the joint
    // 2) The x axis of the mesh is contained in the x-y plane of the reference frame

    // To get the z axis of the mesh to point in the direction of the joint
    Eigen::Quaterniond ori_quat;
    ori_quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), prism_joint_ori_in_ref_rb);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori = prism_joint_ori_in_ref_rb.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_pos = prism_joint_ori_in_ref_rb.cross(coplanar_xy_orthogonal_to_joint_ori);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_pos;
    rotation_pos << coplanar_xy_orthogonal_to_joint_ori.x(),y_pos.x(),prism_joint_ori_in_ref_rb.x(),
            coplanar_xy_orthogonal_to_joint_ori.y(),y_pos.y(),prism_joint_ori_in_ref_rb.y(),
            coplanar_xy_orthogonal_to_joint_ori.z(),y_pos.z(),prism_joint_ori_in_ref_rb.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_final(rotation_pos);

    Eigen::Quaterniond ori_quat_final_ellipse(ori_quat_final.toRotationMatrix()*init_rot.toRotationMatrix());

    prism_axis_unc_cone1.orientation = ori_quat_final_ellipse;
    prism_axis_unc_cone1.ns = "kinematic_structure_uncertainty";
    prism_axis_unc_cone1.id = 3 * this->_joint_id + 1;
    prism_axis_unc_cone1.color = Eigen::Vector4d(0.0, 1.0, 0.0, 0.4);

    // If the uncertainty is pi/6 (30 degrees) the scale in this direction should be 1
    // If the uncertainty is pi/12 (15 degrees) the scale in this direction should be 0.5
    // If the uncertainty is close to 0 the scale in this direction should be 0
    // If the uncertainty is close to pi the scale in this direction should be inf

    prism_axis_unc_cone1.scale = Eigen::Vector3d(major_axis_length / (M_PI / 6.0), minor_axis_length / (M_PI / 6.0), 1.);

    prismatic_markers.push_back(prism_axis_unc_cone1);

    // We repeat the process for the cone in the other direction
    // To get the z axis of the mesh to point in the direction of the joint (negative)
    Eigen::Vector3d prism_joint_ori_in_ref_rb_neg = -prism_joint_ori_in_ref_rb;
    Eigen::Quaterniond ori_quat_neg;
    ori_quat_neg.setFromTwoVectors(Eigen::Vector3d::UnitZ(), prism_joint_ori_in_ref_rb_neg);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori_neg = prism_joint_ori_in_ref_rb_neg.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori_neg.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_neg = prism_joint_ori_in_ref_rb_neg.cross(coplanar_xy_orthogonal_to_joint_ori_neg);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_neg;
    rotation_neg << coplanar_xy_orthogonal_to_joint_ori_neg.x(),y_neg.x(),prism_joint_ori_in_ref_rb_neg.x(),
            coplanar_xy_orthogonal_to_joint_ori_neg.y(),y_neg.y(),prism_joint_ori_in_ref_rb_neg.y(),
            coplanar_xy_orthogonal_to_joint_ori_neg.z(),y_neg.z(),prism_joint_ori_in_ref_rb_neg.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_neg_final(rotation_neg);

    // We undo the rotation of the ellipse (but negative!):
    Eigen::AngleAxisd init_rot_neg(alpha, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond ori_quat_neg_final_ellipse(ori_quat_neg_final.toRotationMatrix()*init_rot_neg.toRotationMatrix());

    prism_axis_unc_cone1.orientation = ori_quat_neg_final_ellipse;
    prism_axis_unc_cone1.scale = Eigen::Vector3d(major_axis_length / (M_PI / 6.0), minor_axis_length / (M_PI / 6.0), 1.);
    prism_axis_unc_cone1.id =3 * this->_joint_id + 2;

    prismatic_markers.push_back(prism_axis_unc_cone1);

    return prismatic_markers;
}

JointFilterType PrismaticJointFilter::getJointFilterType() const
{
    return PRISMATIC_JOINT;
}

std::string PrismaticJointFilter::getJointFilterTypeStr() const
{
    return std::string("PrismaticJointFilter");
}
