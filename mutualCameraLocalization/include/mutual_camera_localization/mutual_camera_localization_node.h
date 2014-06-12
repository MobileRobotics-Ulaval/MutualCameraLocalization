/*
 * mutual_camera_localizator_node.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Matthias Fässler
 *  Edited on:  May 14 2014
 *      Author: Philippe Bbin
 */

/** \file mutual_camera_localizator_node.h
 * \brief File containing the definition of the Mutual Camera Localizator Node class
 *
 */

#ifndef MUTUAL_CAMERA_LOCALIZATOR_NODE_H_
#define MUTUAL_CAMERA_LOCALIZATOR_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <dynamic_reconfigure/server.h>
#include <mutual_camera_localization/MutualCameraLocalizationConfig.h>
#include <mutual_camera_localization/led_detector.h>
#include <mutual_camera_localization/visualization.h>

//#include "mutual_camera_localizator/pose_estimator.h"

namespace mutual_camera_localizator
{
  /*
typedef Eigen::Matrix<double, 6, 6> Matrix6d; //!< A 6x6 matrix of doubles
typedef Eigen::Matrix<double, 2, 6> Matrix2x6d; //!< A 2x6 matrix of doubles
typedef Eigen::Matrix<double, 3, 4> Eigen::Matrix<double, 3, 4>; //!< A 3x4 matrix of doubles
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXYd; //!< A matrix of doubles containing dynamic rows and columns
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> MatrixXYu; //!< A matrix of unsigned integers containing dynamic rows and columns
typedef Eigen::Matrix<double, 6, 1> Vector6d; //!< A column vector of 6 elements containing doubles
typedef Eigen::Matrix<unsigned, 3, 1> Vector3u; //!< A column vector of 3 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, 4, 1> Vector4u; //!< A column vector of 4 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> VectorXu; //!< A dynamic column vector containing unsigened integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 2> VectorXuPairs; //!< A matrix with a dynamic number of rows and 2 columns containing unsigned integers
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowXd; //!< A dynamic row vector containing doubles
typedef Eigen::Matrix<unsigned, 1, Eigen::Dynamic> RowXu; //!< A dynamic row vector containing unsigned integers
typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d
typedef Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1> List3DPoints; //!< A dynamic column vector containing Vector3D elements. \see Vector3d
typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints; //!< A dynamic column vector containing Vector4D elements. \see Vector4d
*/
class MCLNode{
private:
  const ros::Duration diffMax;
  tf::TransformBroadcaster br;
  ros::NodeHandle nh_;

  image_transport::Publisher image_pubA_; //!< The ROS image publisher that publishes the visualisation image
  image_transport::Publisher image_pubB_; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pose_pub_; //!< The ROS publisher that publishes the estimated pose.
  ros::Publisher pose_pub_point_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber image_subA_; //!< The ROS subscriber to the raw camera image A
  ros::Subscriber image_subB_; //!< The ROS subscriber to the raw camera image B
  ros::Subscriber camera_info_sub_; //!< The ROS subscriber to the camera info

  dynamic_reconfigure::Server<mutual_camera_localization::MutualCameraLocalizationConfig> dr_server_; //!< The dynamic reconfigure server
  dynamic_reconfigure::Server<mutual_camera_localization::MutualCameraLocalizationConfig>::CallbackType cb_; //!< The dynamic reconfigure callback type

  //geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool have_camera_info_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  ros::Time camB_time_;
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

  //PoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated
  //geometry_msgs::PoseWithCovarianceStamped predicted_pose_;
  //geometry_msgs::PoseStamped predicted_pose_;
  visualization_msgs::Marker marker_pose_;
  visualization_msgs::Marker marker_arrow_;
  cv::Rect region_of_interest_;
  cv::Mat camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
  cv::Mat camera_matrix_P_; //!< Variable to store the projection matrix (as an OpenCV matrix) that projects points onto the rectified image plane.
  std::vector<double> camera_distortion_coeffs_; //!< Variable to store the camera distortion parameters
  std::vector<cv::Point2f> distorted_detection_centers_;
  std::vector<cv::Point2f> undistorted_detection_centers_;
  Eigen::Matrix<double, 3, 4> camera_projection_matrix_; //!< Stores the camera calibration matrix. This is the 3x4 projection matrix that projects the world points to the image coordinates stored in #image_points_.
  Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> image_vectorsA_;
  Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> image_vectorsB_;
  //Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> image_vectors_2;

  double gaussian_sigma_;
  double min_blob_area_;
  double max_blob_area_;
  double max_circular_distortion_;
  double ratio_ellipse_min_;
  double ratio_ellipse_max_;
  double pos_ratio_;
  double pos_ratio_tolerance_;
  double line_angle_tolerance_;
  double hor_line_angle_; 
  double radius_ratio_tolerance_;
  double ratio_int_tolerance_; 
  double rdA_, ldA_, rdB_, ldB_;
  //double max_width_height_distortion_;
  //double radius_ratio_tolerance_;
  //double ratio_int_tolerance_;
public:

  MCLNode();
  ~MCLNode();
  void imageCallbackA(const sensor_msgs::Image::ConstPtr& image_msg);
  void imageCallbackB(const sensor_msgs::Image::ConstPtr& image_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg, const bool camA);
  bool callDetectLed(cv::Mat image, const bool camA);

  void initMarker();


  static Eigen::Vector2d ComputePositionMutual(double alpha, double beta, double d);
  static Eigen::MatrixXd vrrotvec2mat(double p, Eigen::Vector3d r);
  static Eigen::MatrixXd Compute3DMutualLocalisation(Eigen::Vector2d ImageA1, Eigen::Vector2d ImageA2,
                                            Eigen::Vector2d ImageB1, Eigen::Vector2d ImageB2,
                                            Eigen::Vector2d ppA, Eigen::Vector2d ppB,
                                            Eigen::Vector2d fCamA, Eigen::Vector2d fCamB,
                                            double rdA, double ldA, double rdB, double ldB,
                                            Eigen::Vector3d* pos, double* dist);

  void dynamicParametersCallback(mutual_camera_localization::MutualCameraLocalizationConfig &config, uint32_t level);

  //void calculateImageVectors(Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> image_points);
};

} // mutual_camera_localizator_node namespace

#endif /* MUTAL_CAMERA_LOCALIZATOR_NODE_H_ */
