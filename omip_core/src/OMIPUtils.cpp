#include "omip_core/OMIPUtils.h"
#include "omip_core/Log.h"

#include <math.h>

#include <ctime>
#include <random>

#include <iostream>
#include <fstream>

using namespace omip;

void omip::EigenAffine2TranslationAndEulerAngles(const Eigen::Affine3d& t,
                                                 double& x, double& y, double& z,
                                                 double& roll, double& pitch,
                                                 double& yaw)
{
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    roll = atan2(t(2, 1), t(2, 2));
    pitch = asin(-t(2, 0));
    yaw = atan2(t(1, 0), t(0, 0));
}

void omip::TranslationAndEulerAngles2EigenAffine(
        const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw,
        Eigen::Transform<double, 3, Eigen::Affine> &t)
{
    double A = cos(yaw), B = sin(yaw), C = cos(pitch), D = sin(pitch), E = cos(roll), F = sin(roll), DE = D * E, DF = D * F;

    t(0, 0) = A * C;
    t(0, 1) = A * DF - B * E;
    t(0, 2) = B * F + A * DE;
    t(0, 3) = x;
    t(1, 0) = B * C;
    t(1, 1) = A * E + B * DF;
    t(1, 2) = B * DE - A * F;
    t(1, 3) = y;
    t(2, 0) = -D;
    t(2, 1) = C * F;
    t(2, 2) = C * E;
    t(2, 3) = z;
    t(3, 0) = 0;
    t(3, 1) = 0;
    t(3, 2) = 0;
    t(3, 3) = 1;
}

double omip::sampleNormal(double mean, double std_dev)
{
    // Ported from std::tr1::{mt19937,normal_distribution,variate_generator}
    // (pre-C++11 TR1, unavailable/removed in modern libc++) to the
    // equivalent standard <random> facilities. Same generator (Mersenne
    // Twister) and distribution, same wall-clock seed — no numerical
    // behavior change, just a namespace/API modernization forced by the
    // toolchain (see PORTING_NOTES.md Phase 2 section).
    static std::mt19937 rng(static_cast<unsigned>(std::time(0)));

    std::normal_distribution<double> norm_dist(mean, std_dev);

    return norm_dist(rng);
}

std::ostream& omip::operator <<(std::ostream& os,
                                std::vector<Feature::Id> vector_ids)
{
    for (size_t idx = 0; idx < vector_ids.size(); idx++)
    {
        os << vector_ids.at(idx) << " ";
    }
    return (os);
}

std::ostream& omip::operator <<(std::ostream& os, Feature::Location location)
{
    os << "(" << boost::tuples::get<0>(location) << ","
       << boost::tuples::get<1>(location) << ","
       << boost::tuples::get<2>(location) << ")";

    return (os);
}

std::ostream& omip::operator <<(std::ostream& os, Twistd twistd)
{
    os << "(" << twistd.vx() << ","
       << twistd.vy() << ","
       << twistd.vz() << ","
       << twistd.rx() << ","
       << twistd.ry() << ","
       << twistd.rz() << ")";

    return (os);
}

double omip::L2Distance(const Feature::Location &first, const Feature::Location &second)
{
    double error_value = sqrt( pow(boost::tuples::get<0>(first) - boost::tuples::get<0>(second), 2)
                               + pow(boost::tuples::get<1>(first) - boost::tuples::get<1>(second), 2)
                               + pow(boost::tuples::get<2>(first) - boost::tuples::get<2>(second), 2));
    return error_value;
}

void omip::TransformMatrix2Twist(const Eigen::Matrix4d& transformation_matrix, Twistd &twist)
{
    Eigen::Isometry3d displ_from_transf(transformation_matrix);
    twist = se3Log(displ_from_transf, 1e-12);
}

void omip::TransformMatrix2TwistUnwrapped(const Eigen::Matrix4d& transformation_matrix, Twistd &twist, const Twistd &twist_previous)
{
    Eigen::Isometry3d displ_from_transf(transformation_matrix);
    twist = se3Log(displ_from_transf, 1e-12);
}

void omip::Twist2TransformMatrix(const Twistd& transformation_twist, Eigen::Matrix4d &matrix)
{
    Eigen::Isometry3d displ_from_twist = se3Exp(transformation_twist, 1e-12);
    matrix = displ_from_twist.matrix();
}

void omip::TransformLocation(const Feature::Location& origin, const Eigen::Matrix4d& transformation, Feature::Location& new_location)
{
    Eigen::Affine3d motion_matrix(transformation);
    Eigen::Vector3d origin_vector = Eigen::Vector3d(
                boost::tuples::get<0>(origin), boost::tuples::get<1>(origin),
                boost::tuples::get<2>(origin));
    Eigen::Vector3d destination_vector = motion_matrix * origin_vector;
    boost::tuples::get<0>(new_location) = destination_vector[0];
    boost::tuples::get<1>(new_location)= destination_vector[1];
    boost::tuples::get<2>(new_location) = destination_vector[2];
}

void omip::TransformLocation(const Feature::Location& origin, const Twistd& twist, Feature::Location& new_location, int feat_id)
{
    Eigen::Matrix4d eigen_transform;
    omip::Twist2TransformMatrix(twist, eigen_transform);

    omip::TransformLocation(origin, eigen_transform, new_location);
}

void omip::LocationOfFeature2ColumnVector(const Feature::Location &lof, MatrixWrapper::ColumnVector &col_vec)
{
    col_vec(1) = boost::tuples::get<0>(lof);
    col_vec(2) = boost::tuples::get<1>(lof);
    col_vec(3) = boost::tuples::get<2>(lof);
}

void omip::LocationOfFeature2ColumnVectorHomogeneous(const Feature::Location& lof, MatrixWrapper::ColumnVector& col_vec)
{
    col_vec(1) = boost::tuples::get<0>(lof);
    col_vec(2) = boost::tuples::get<1>(lof);
    col_vec(3) = boost::tuples::get<2>(lof);
    col_vec(4) = 1;
}

void omip::LocationOfFeature2EigenVectorHomogeneous(const Feature::Location& lof, Eigen::Vector4d& eig_vec)
{
    eig_vec(0) = boost::tuples::get<0>(lof);
    eig_vec(1) = boost::tuples::get<1>(lof);
    eig_vec(2) = boost::tuples::get<2>(lof);
    eig_vec(3) = 1;
}

Feature::Location omip::operator-(const Feature::Location& location1,
                                  const Feature::Location& location2)
{
    return Feature::Location(
                boost::tuples::get<0>(location1) - boost::tuples::get<0>(location2),
                boost::tuples::get<1>(location1) - boost::tuples::get<1>(location2),
                boost::tuples::get<2>(location1) - boost::tuples::get<2>(location2));
}

Feature::Location omip::operator+(const Feature::Location& location1,
                                  const Feature::Location& location2)
{
    return Feature::Location(
                boost::tuples::get<0>(location1) + boost::tuples::get<0>(location2),
                boost::tuples::get<1>(location1) + boost::tuples::get<1>(location2),
                boost::tuples::get<2>(location1) + boost::tuples::get<2>(location2));
}

bool omip::isFinite(const Eigen::Matrix4d& transformation)
{
    bool ret_val = true;
    for (int col = 0; col < transformation.cols(); ++col)
    {
        for (int row = 0; row < transformation.rows(); ++row)
        {
            if (isnan(transformation(row, col)))
            {
                ret_val = false;
                break;
            }
        }
    }
    return ret_val;
}

void omip::Location2PointPCL(const Feature::Location &point_location,pcl::PointXYZ& point_pcl)
{
    point_pcl.x = boost::tuples::get<0>(point_location);
    point_pcl.y = boost::tuples::get<1>(point_location);
    point_pcl.z = boost::tuples::get<2>(point_location);
}

void omip::LocationAndId2FeaturePCL(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCL &feature_pcl)
{
    feature_pcl.x = boost::tuples::get<0>(feature_location);
    feature_pcl.y = boost::tuples::get<1>(feature_location);
    feature_pcl.z = boost::tuples::get<2>(feature_location);
    feature_pcl.label = feature_id;
}

void omip::LocationAndId2FeaturePCLwc(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCLwc &feature_pcl)
{
    feature_pcl.x = boost::tuples::get<0>(feature_location);
    feature_pcl.y = boost::tuples::get<1>(feature_location);
    feature_pcl.z = boost::tuples::get<2>(feature_location);
    feature_pcl.label = feature_id;
}

//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),2*M_PI);
}
inline double angleDiff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}
inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}

Twistd omip::unwrapTwist(Twistd& current_twist, Eigen::Isometry3d& current_displacement, Twistd& previous_twist, bool& changed)
{
    Eigen::Vector3d current_angular_component = Eigen::Vector3d(current_twist.rx(), current_twist.ry(), current_twist.rz());
    double current_rotation_angle = current_angular_component.norm();
    Eigen::Vector3d current_angular_direction = current_angular_component / current_rotation_angle;
    Eigen::Vector3d previous_angular_component = Eigen::Vector3d(previous_twist.rx(), previous_twist.ry(), previous_twist.rz());
    double previous_rotation_angle = previous_angular_component.norm();
    Eigen::Vector3d previous_angular_direction = previous_angular_component / previous_rotation_angle;

    // The difference should be around 2PI (a little bit less)
    if(Eigen::Vector3d(current_twist.rx()-previous_twist.rx(), current_twist.ry()-previous_twist.ry(), current_twist.rz()-previous_twist.rz()).norm() > M_PI
            || current_angular_direction.dot(previous_angular_direction) < -0.8)
    {

        Eigen::Vector3d new_angular_component;

        Twistd unwrapped_twist;
        // Two cases:
        // 1) Jump from PI to -PI or vice versa
        // 1) Jump from 2*PI to 0 or from -2*PI to zero
        if(previous_rotation_angle + 0.3 < 2*M_PI)
        {
            // If previous and current are looking in opposite directions -> scalar product will be close to -1
            if(current_angular_direction.dot(previous_angular_direction) < 0)
            {
                new_angular_component= M_PI*previous_angular_direction -(M_PI - current_rotation_angle)*current_angular_direction;
            }// If both are looking in the same direction -> scalar product is close to +1
            else{
                new_angular_component= M_PI*previous_angular_direction +(M_PI - current_rotation_angle)*current_angular_direction;
            }
        }else{
            OMIP_ERROR_STREAM("Numeric instability in the logarithm of a homogeneous transformation!");
            // If previous and current are looking in opposite directions -> scalar product will be close to -1
            if(current_angular_direction.dot(previous_angular_direction) < 0)
            {
                new_angular_component= std::min(2*M_PI, previous_rotation_angle)*previous_angular_direction +current_rotation_angle*current_angular_direction;
            }// If both are looking in the same direction -> scalar product is close to +1
            else{
                new_angular_component= std::min(2*M_PI, previous_rotation_angle)*previous_angular_direction +current_rotation_angle*current_angular_direction;
            }
        }

        double n2 = new_angular_component.squaredNorm();  // ||wtilde||^2
        double n = sqrt(n2);
        double sn = sin(n);
        double val = (double(2.0) * sn - n * (double(1.0) + cos(n))) / (double(2.0) *n2 * sn);

        Eigen::Vector3d RE3Element = current_displacement.translation();
        Eigen::Vector3d lin = -0.5*new_angular_component.cross(RE3Element);
        lin += (double(1.0) - val * n2 ) * RE3Element;
        lin += val * new_angular_component.dot(RE3Element) * new_angular_component;

        unwrapped_twist = Twistd(new_angular_component.x(), new_angular_component.y(), new_angular_component.z(), lin.x(), lin.y(), lin.z());

        if(std::fabs(val) > 10)
        {
            OMIP_ERROR_STREAM("Numeric instability in the logarithm of a homogeneous transformation. Val: " << val);
        }

        changed = true;
        return unwrapped_twist;
    }else{
        changed = false;
        return current_twist;
    }
}

Twistd omip::invertTwist(Twistd& current_twist, Twistd& previous_twist, bool& inverted)
{
    Eigen::Vector3d current_angular_component = Eigen::Vector3d(current_twist.rx(), current_twist.ry(), current_twist.rz());
    double current_rotation_angle = current_angular_component.norm();
    Eigen::Vector3d current_angular_direction = current_angular_component / current_rotation_angle;
    Eigen::Vector3d previous_angular_component = Eigen::Vector3d(previous_twist.rx(), previous_twist.ry(), previous_twist.rz());
    double previous_rotation_angle = previous_angular_component.norm();
    Eigen::Vector3d previous_angular_direction = previous_angular_component / previous_rotation_angle;
    // The difference should be around 2PI (a little bit less)
    if(Eigen::Vector3d(current_twist.rx()-previous_twist.rx(), current_twist.ry()-previous_twist.ry(), current_twist.rz()-previous_twist.rz()).norm() > M_PI
            || current_angular_direction.dot(previous_angular_direction) < -0.8)
    {
        inverted = true;
        return -current_twist;
    }else{
        inverted = false;
        return current_twist;
    }
}

void omip::invert3x3Matrix(const MatrixWrapper::Matrix& to_inv, MatrixWrapper::Matrix& inverse)
{
    double determinant =    +to_inv(1,1)*(to_inv(2,2)*to_inv(3,3)-to_inv(3,2)*to_inv(2,3))
            -to_inv(1,2)*(to_inv(2,1)*to_inv(3,3)-to_inv(2,3)*to_inv(3,1))
            +to_inv(1,3)*(to_inv(2,1)*to_inv(3,2)-to_inv(2,2)*to_inv(3,1));
    double invdet = 1.0/determinant;
    inverse(1,1) =  (to_inv(2,2)*to_inv(3,3)-to_inv(3,2)*to_inv(2,3))*invdet;
    inverse(2,1) = -(to_inv(1,2)*to_inv(3,3)-to_inv(1,3)*to_inv(3,2))*invdet;
    inverse(3,1) =  (to_inv(1,2)*to_inv(2,3)-to_inv(1,3)*to_inv(2,2))*invdet;
    inverse(1,2) = -(to_inv(2,1)*to_inv(3,3)-to_inv(2,3)*to_inv(3,1))*invdet;
    inverse(2,2) =  (to_inv(1,1)*to_inv(3,3)-to_inv(1,3)*to_inv(3,1))*invdet;
    inverse(3,2) = -(to_inv(1,1)*to_inv(2,3)-to_inv(2,1)*to_inv(1,3))*invdet;
    inverse(1,3) =  (to_inv(2,1)*to_inv(3,2)-to_inv(3,1)*to_inv(2,2))*invdet;
    inverse(2,3) = -(to_inv(1,1)*to_inv(3,2)-to_inv(3,1)*to_inv(1,2))*invdet;
    inverse(3,3) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
}

void omip::invert3x3MatrixEigen(const Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& to_inv,
                                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& inverse)
{
    double determinant =    +to_inv(0,0)*(to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))
            -to_inv(0,1)*(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))
            +to_inv(0,2)*(to_inv(1,0)*to_inv(2,1)-to_inv(1,1)*to_inv(2,0));
    double invdet = 1.0/determinant;
    inverse(0,0) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
    inverse(1,0) = -(to_inv(0,1)*to_inv(2,2)-to_inv(0,2)*to_inv(2,1))*invdet;
    inverse(2,0) =  (to_inv(0,1)*to_inv(1,2)-to_inv(0,2)*to_inv(1,1))*invdet;
    inverse(0,1) = -(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))*invdet;
    inverse(1,1) =  (to_inv(0,0)*to_inv(2,2)-to_inv(0,2)*to_inv(2,0))*invdet;
    inverse(2,1) = -(to_inv(0,0)*to_inv(1,2)-to_inv(1,0)*to_inv(0,2))*invdet;
    inverse(0,2) =  (to_inv(1,0)*to_inv(2,1)-to_inv(2,0)*to_inv(1,1))*invdet;
    inverse(1,2) = -(to_inv(0,0)*to_inv(2,1)-to_inv(2,0)*to_inv(0,1))*invdet;
    inverse(2,2) =  (to_inv(0,0)*to_inv(1,1)-to_inv(1,0)*to_inv(0,1))*invdet;
}

void omip::invert3x3MatrixEigen2(const Eigen::Matrix3d& to_inv,
                                 Eigen::Matrix3d& inverse)
{
    double determinant =    +to_inv(0,0)*(to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))
            -to_inv(0,1)*(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))
            +to_inv(0,2)*(to_inv(1,0)*to_inv(2,1)-to_inv(1,1)*to_inv(2,0));
    double invdet = 1.0/determinant;
    inverse(0,0) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
    inverse(1,0) = -(to_inv(0,1)*to_inv(2,2)-to_inv(0,2)*to_inv(2,1))*invdet;
    inverse(2,0) =  (to_inv(0,1)*to_inv(1,2)-to_inv(0,2)*to_inv(1,1))*invdet;
    inverse(0,1) = -(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))*invdet;
    inverse(1,1) =  (to_inv(0,0)*to_inv(2,2)-to_inv(0,2)*to_inv(2,0))*invdet;
    inverse(2,1) = -(to_inv(0,0)*to_inv(1,2)-to_inv(1,0)*to_inv(0,2))*invdet;
    inverse(0,2) =  (to_inv(1,0)*to_inv(2,1)-to_inv(2,0)*to_inv(1,1))*invdet;
    inverse(1,2) = -(to_inv(0,0)*to_inv(2,1)-to_inv(2,0)*to_inv(0,1))*invdet;
    inverse(2,2) =  (to_inv(0,0)*to_inv(1,1)-to_inv(1,0)*to_inv(0,1))*invdet;
}

void omip::computeAdjoint(const Twistd &pose_ec, Eigen::Matrix<double, 6, 6> &adjoint_out)
{
    omip::computeAdjoint(se3Exp(pose_ec, 1e-12), adjoint_out);
}

void omip::adjointXcovXadjointT(const Twistd& pose_ec, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    Eigen::Matrix<double, 6, 6> adjoint;
    omip::computeAdjoint(pose_ec, adjoint);

    transformed_cov_out = adjoint*cov*adjoint.transpose();
}

//LGSM adjoint:
// (   R    0 )
// ((t x R) R )
//Desired adjoint:
// ( R (t x R))
// ( 0    R   )
void omip::computeAdjoint(Eigen::Isometry3d pose_disp, Eigen::Matrix<double, 6, 6> &adjoint_out)
{
    Eigen::Matrix<double, 6, 6> adjoint_rot_trans = se3Adjoint(pose_disp);

    adjoint_out = adjoint_rot_trans;
    adjoint_out.block<3,3>(0,3) = adjoint_rot_trans.block<3,3>(3,0);
    adjoint_out.block<3,3>(3,0) = adjoint_rot_trans.block<3,3>(0,3);
}

void omip::adjointXcovXadjointT(const Eigen::Isometry3d& pose_disp,
                                const Eigen::Matrix<double, 6, 6>& cov,
                                Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    Eigen::Matrix<double, 6, 6> adjoint;
    omip::computeAdjoint(pose_disp, adjoint);

    transformed_cov_out = adjoint*cov*adjoint.transpose();
}

void omip::adjointXinvAdjointXcovXinvAdjointTXadjointT(const Eigen::Isometry3d& pose_disp1,
                                                       const Eigen::Isometry3d& pose_disp2,
                                                       const Eigen::Matrix<double, 6, 6>& cov,
                                                       Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    //T_rrbf_srbf.adjoint()*T_rrbf_srbf_t0.inverse().adjoint()*_srb_initial_pose_cov_in_rrbf*T_rrbf_srbf_t0.inverse().adjoint().transpose()*T_rrbf_srbf.adjoint().transpose()
    Eigen::Matrix<double, 6, 6> adjoint1;
    omip::computeAdjoint(pose_disp1, adjoint1);

    Eigen::Matrix<double, 6, 6> adjoint2;
    omip::computeAdjoint(pose_disp2.inverse(), adjoint2);

    transformed_cov_out = adjoint1*adjoint2*cov*adjoint2.transpose()*adjoint1.transpose();
}
