/*
 * OMIPUtils.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014).
 * This implementation can be used to reproduce the results of the paper and to be applied to new research.
 * The implementation allows also to be extended to perceive different information/models or to use additional sources of information.
 * A detail explanation of the method and the system can be found in our paper.
 *
 * If you are using this implementation in your research, please consider citing our work:
 *
@inproceedings{martinmartin_ip_iros_2014,
Title = {Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors},
Author = {Roberto {Mart\'in-Mart\'in} and Oliver Brock},
Booktitle = {Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems},
Pages = {2494-2501},
Year = {2014},
Location = {Chicago, Illinois, USA},
Note = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Url = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Projectname = {Interactive Perception}
}
 * If you have questions or suggestions, contact us:
 * roberto.martinmartin@tu-berlin.de
 *
 * Enjoy!
 *
 * Ported to a ROS-free library. Dropped per PORTING_NOTES.md section 0.3
 * ("geometry_msgs::Twist" row): the geometry_msgs::Twist-typed overloads of
 * TransformLocation, plus GeometryMsgsTwist2EigenTwist/
 * EigenTwist2GeometryMsgsTwist/ROSTwist2EigenTwist, since Twistd-only
 * equivalents already exist in this same file (pure overload removal, no
 * gap-filling needed). Also dropped the unused `<ros/ros.h>`/
 * `<tf/transform_broadcaster.h>` includes (see PORTING_NOTES.md: verified
 * vestigial, no tf:: symbol used in OMIPUtils.cpp; the two ROS_ERROR_STREAM
 * call sites are replaced with the Log.h shim).
 *
 * lgsm (Twistd/Eigen::Isometry3d) replaced with omip::Twistd and
 * Eigen::Isometry3d per PORTING_NOTES.md Phase 2 (lgsm proved incompatible
 * with every modern Eigen version) — see omip_core/LieGroup.hpp for the
 * verbatim-ported log/exp/adjoint math.
 */

#ifndef OMIP_UTILS_H_
#define OMIP_UTILS_H_

#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include "omip_core/Feature.h"
#include "omip_core/LieGroup.hpp"

#include "wrappers/matrix/matrix_wrapper.h"

#include "omip_core/OMIPTypeDefs.h"

namespace omip
{

/**
 * Convert an Eigen::Affine3d matrix into translation and rotation (Euler angles)
 * @param t - Eigen Affine3d (input)
 * @param x - Translation in x (output)
 * @param y - Translation in y (output)
 * @param z - Translation in z (output)
 * @param roll - Rotation roll (output)
 * @param pitch - Rotation pitch (output)
 * @param yaw - Rotation yaw (output)
 */
void EigenAffine2TranslationAndEulerAngles(const Eigen::Affine3d& t, double& x,
                                  double& y, double& z, double& roll,
                                  double& pitch, double& yaw);

/**
 * Convert translation and rotation (Euler angles) into an Eigen::Affine3d matrix
 * @param x - Translation in x (input)
 * @param y - Translation in x (input)
 * @param z - Translation in x (input)
 * @param roll - Rotation roll (input)
 * @param pitch - Rotation pitch (input)
 * @param yaw - Rotation yaw (input)
 * @param t - Eigen Affine3d (output)
 */
void TranslationAndEulerAngles2EigenAffine(const double& x, const double& y, const double& z, const double& roll, const double& pitch,
                       const double& yaw,
                       Eigen::Transform<double, 3, Eigen::Affine> &t);

/**
 * Retrieve one sample of a normal (Gaussian) distribution, given its mean and standard deviation
 * @param mean - Mean value of the Gaussian
 * @param std_dev - Std deviation of the Gaussian
 * @return - One sample of the Gaussian distribution
 */
double sampleNormal(double mean, double std_dev);

/**
 * Operator << for ostream to add a vector of Feature Ids
 * @param os - Input ostream
 * @param vector_ids - Vector of Feature Ids to add to the ostream
 * @return - ostream with the added vector
 */
std::ostream& operator <<(std::ostream& os,
                          std::vector<Feature::Id> vector_ids);

/**
 * Operator << for ostream to add the Location of a Feature
 * @param os - Input ostream
 * @param location - Location of a Feature to add to the ostream
 * @return - ostream with the added Location
 */
std::ostream& operator <<(std::ostream& os, Feature::Location location);


std::ostream& operator <<(std::ostream& os, Twistd twistd);

/**
 * Compute the L2 distance between two Feature Locations. The Locations can represent
 * the same Feature in two time steps or two different Features
 * @param first - First Feature location
 * @param second - Second Feature location
 * @return - L2 distance between first and second Feature Locations
 */
double L2Distance(const Feature::Location& first, const Feature::Location& second);

/**
 * DEPRECATED!
 * This function is dangerous because it uses internally the function log to convert the transformation matrix
 * into an element of the Lie algebra and this function is discontinuous!
 * DO NOT USE THIS FUNCTION!
 * Convert an Eigen Matrix (4x4) of a homogeneous transformation to an Eigen Twist
 * @param transformation_matrix - Eigen matrix of the transformation
 * @return - Twist of the transformation
 */
void TransformMatrix2Twist(const Eigen::Matrix4d& transformation_matrix, Twistd& twist);

/**
 * Convert an Eigen Matrix (4x4) of a homogeneous transformation to an Eigen Twist
 * @param transformation_matrix - Eigen matrix of the transformation
 * @return - Twist of the transformation
 */
void TransformMatrix2TwistUnwrapped(const Eigen::Matrix4d& transformation_matrix, Twistd& twist, const Twistd& twist_previous);

/**
 * Convert an Eigen Twist to an Eigen Matrix (4x4) homogeneous transformation
 * @param transformation_twist - Eigen twist of the transformation
 * @return - Eigen Matrix (4x4) homogeneous transformation
 */
void Twist2TransformMatrix( const Twistd& transformation_twist, Eigen::Matrix4d& matrix);

/**
 * Transform the location of a Feature using a rigid body transformation (matrix)
 * @param origin - Original location of the Feature
 * @param transformation - Rigid body transformation (pose)
 * @return - New location of the Feature
 */
void TransformLocation(const Feature::Location& origin, const Eigen::Matrix4d& transformation, Feature::Location& new_location);

/**
 * Transform the location of a Feature using a rigid body transformation (twist)
 * @param origin - Original location of the Feature
 * @param twist - Rigid body transformation (pose)
 * @return - New location of the Feature
 */
void TransformLocation(const Feature::Location& origin, const Twistd& twist, Feature::Location &new_location, int feat_id=0);

/**
 * Convert the Location of a Feature to a ColumnVector (3x1)
 * @param lof - Location of a Feature
 * @return - Column vector (3x1)
 */
void LocationOfFeature2ColumnVector(const Feature::Location& lof, MatrixWrapper::ColumnVector& col_vec);

/**
 * Convert the Location of a Feature to a ColumnVector in homogeneous coordinates (4x1)
 * @param lof - Location of a Feature
 * @return - Column vector homogeneous (4x1)
 */
void LocationOfFeature2ColumnVectorHomogeneous(const Feature::Location &lof, MatrixWrapper::ColumnVector &col_vec);

void LocationOfFeature2EigenVectorHomogeneous(const Feature::Location& lof, Eigen::Vector4d& eig_vec);

/**
 * Operator - for two Feature Locations
 * @param location1 - First Feature Location
 * @param location2 - Second Feature Location
 * @return - Subtraction of the first Feature Location minus the second Feature Location (it should be a
 * vector but for coherence we return a Feature Location)
 */
Feature::Location operator-(const Feature::Location& location1,
                            const Feature::Location& location2);

Feature::Location operator+(const Feature::Location& location1,
                            const Feature::Location& location2);

/**
 * Checks if all the elements of the Eigen matrix are finite
 * @param transformation - Eigen matrix to test
 * @return - TRUE if all elements of the matrix are finite
 */
bool isFinite(const Eigen::Matrix4d& transformation);

void Location2PointPCL(const Feature::Location &point_location, pcl::PointXYZ& point_pcl);

void LocationAndId2FeaturePCL(const Feature::Location &feature_location, const Feature::Id &feature_id, pcl::PointXYZL& feature_pcl);

void LocationAndId2FeaturePCLwc(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCLwc& feature_pcl);

Twistd unwrapTwist(Twistd& current_twist, Eigen::Isometry3d& current_displacement, Twistd& previous_twist, bool &changed);

Twistd invertTwist(Twistd& current_twist, Twistd& previous_twist, bool& inverted);

void invert3x3Matrix(const MatrixWrapper::Matrix& to_inv, MatrixWrapper::Matrix& inverse);

void invert3x3MatrixEigen(const Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& to_inv,
                          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& inverse);

void invert3x3MatrixEigen2(const Eigen::Matrix3d& to_inv, Eigen::Matrix3d& inverse);

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

/**
 * @brief computeAdjoint Returns the adjoint matrix of the given pose. Necessary because the LGSM library represents first rotation
 * and then translation while we represent first translation and then rotation
 * @param pose_ec Pose (in exponential coordinates) to obtain the adjoint for
 * @param adjoint Matrix containing the adjoint. 3 first rows are translation and 3 last rows are rotation
 */
void computeAdjoint(const Twistd& pose_ec, Eigen::Matrix<double, 6, 6>& adjoint_out);

/**
 * @brief adjointXcovXadjointT Transforms a covariance from one reference frame to another
 * @param pose_ec Pose (in exponential coordinates) where we want to have the covariance expressed
 * @param cov Original covariance
 * @param transformed_cov Transformed covariance
 */
void adjointXcovXadjointT(const Twistd& pose_ec, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out);

/**
 * Same as the functions above but the pose is passed as a displacement
 */
void computeAdjoint(const Eigen::Isometry3d pose_disp, Eigen::Matrix<double, 6, 6> &adjoint_out);
void adjointXcovXadjointT(const Eigen::Isometry3d &pose_disp, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out);

void adjointXinvAdjointXcovXinvAdjointTXadjointT(const Eigen::Isometry3d& pose_disp1,
                                                       const Eigen::Isometry3d& pose_disp2,
                                                       const Eigen::Matrix<double, 6, 6>& cov,
                                                       Eigen::Matrix<double, 6, 6>& transformed_cov_out);

}

#endif /* OMIP_UTILS_H_ */
