/*
 * monocular_pose_estimator.cpp
 *
 * Created on: Jul 29, 2013
 * Author: Karl Schwabe
 */

/** \file monocular_pose_estimator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */
using namespace std;
#include "mutualCameraLocalization/monocular_pose_estimator_node.h"

namespace monocular_pose_estimator
{

/**
 * Constructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::MPENode()
{
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  //dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_;
  //cb_ = boost::bind(&MPENode::dynamicParametersCallback, this, _1, _2);
  //dr_server_.setCallback(cb_);

  // Initialize subscribers
  image_sub_ = nh_.subscribe("/camera/image_raw", 1, &MPENode::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("/camera/camera_info", 1, &MPENode::cameraInfoCallback, this);

  // Initialize pose publisher
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);

  // Create the marker positions from the test points
   /*List4DPoints positions_of_markers_on_object;

  // Read in the marker positions from the YAML parameter file
 XmlRpc::XmlRpcValue points_list;
  if (!nh_.getParam(ros::this_node::getName() + "/marker_positions", points_list))
  {
    ROS_ERROR(
        "%s: No reference file containing the marker positions, or the file is improperly formatted. Use the 'marker_positions_file' parameter in the launch file.",
        ros::this_node::getName().c_str());
    ros::shutdown();
  }
  else
  {
    positions_of_markers_on_object.resize(points_list.size());
    for (int i = 0; i < points_list.size(); i++)
    {
      Eigen::Matrix<double, 4, 1> temp_point;
      temp_point(0) = points_list[i]["x"];
      temp_point(1) = points_list[i]["y"];
      temp_point(2) = points_list[i]["z"];
      temp_point(3) = 1;
      positions_of_markers_on_object(i) = temp_point;
    }
  }
  //trackable_object_.setMarkerPositions(positions_of_markers_on_object);
  ROS_INFO("The number of markers on the object are: %d", (int )positions_of_markers_on_object.size());*/
}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::~MPENode()
{

}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MPENode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (!have_camera_info_)
  {
    cam_info_ = *msg;

    // Calibrated camera
    Matrix3x4d camera_matrix;
    camera_matrix(0, 0) = cam_info_.P[0];
    camera_matrix(0, 2) = cam_info_.P[2];
    camera_matrix(1, 1) = cam_info_.P[5];
    camera_matrix(1, 2) = cam_info_.P[6];
    camera_matrix(2, 2) = 1.0;
    
    camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
    camera_matrix_P_ = cv::Mat(3, 4, CV_64F);

    camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
    camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
    camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
    camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
    camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
    camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
    camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
    camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
    camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
    camera_distortion_coeffs_ = cam_info_.D;
    camera_matrix_P_.at<double>(0, 0) = cam_info_.P[0];
    camera_matrix_P_.at<double>(0, 1) = cam_info_.P[1];
    camera_matrix_P_.at<double>(0, 2) = cam_info_.P[2];
    camera_matrix_P_.at<double>(0, 3) = cam_info_.P[3];
    camera_matrix_P_.at<double>(1, 0) = cam_info_.P[4];
    camera_matrix_P_.at<double>(1, 1) = cam_info_.P[5];
    camera_matrix_P_.at<double>(1, 2) = cam_info_.P[6];
    camera_matrix_P_.at<double>(1, 3) = cam_info_.P[7];
    camera_matrix_P_.at<double>(2, 0) = cam_info_.P[8];
    camera_matrix_P_.at<double>(2, 1) = cam_info_.P[9];
    camera_matrix_P_.at<double>(2, 2) = cam_info_.P[10];
    camera_matrix_P_.at<double>(2, 3) = cam_info_.P[11];

    camera_projection_matrix_ = camera_matrix;
    have_camera_info_ = true;
    ROS_INFO("Camera calibration information obtained.");
    ROS_DEBUG_STREAM("Camera calibration matrix: \n" << camera_matrix);
  }

}

void MPENode::calculateImageVectors(List2DPoints image_points){
  unsigned num_image_points = image_points.size();
  image_vectors_.resize(num_image_points);
  image_vectors_2.resize(num_image_points);
  Eigen::Vector3d single_vector;
  Eigen::Vector2d single_vector2;

  for (unsigned i = 0; i < num_image_points; ++i){
    single_vector(0) = (image_points(i)(0) - camera_projection_matrix_(0, 2)) / camera_projection_matrix_(0, 0);
    single_vector(1) = (image_points(i)(1) - camera_projection_matrix_(1, 2)) / camera_projection_matrix_(1, 1);
    single_vector(2) = 1;
    single_vector2(0) = single_vector[0];
    single_vector2(1) = single_vector[1];

    image_vectors_(i) = single_vector / single_vector.norm();
    image_vectors_2(i) = single_vector2 / single_vector2.norm();
  }
}

bool MPENode::callDetectLed(cv::Mat image){
  region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

    // Do detection of LEDs in image
  List2DPoints detected_led_positions;
  distorted_detection_centers_;

  LEDDetector::findLeds(image, region_of_interest_, 140, 0.6, 100,
                        40000, 0.5, 0.5,
                        detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
                        camera_distortion_coeffs_, camera_matrix_P_);
  detected_led_positions.size();
  printf("Nbr of leds: %i\n",(int)detected_led_positions.size());
  //ROS_WARN("Nobody like sarcasm");

  if (detected_led_positions.size() >= 2) // If found enough LEDs, Reinitialise
  {
    //printf("P1: (%f, %f)\n",(float)detected_led_positions(0)(0),(float)detected_led_positions(0)(1));
    //printf("P2: (%f, %f)\n",(float)detected_led_positions(1)(0),(float)detected_led_positions(1)(1));
    calculateImageVectors(detected_led_positions);
  }
  else
  { // Too few LEDs found
    return false;
  }
  return true;
}


/**
 * The callback function that is executed every time an image is received. It runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void MPENode::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{

  // Check whether already received the camera calibration data
  if (!have_camera_info_)
  {
    ROS_WARN("No camera info yet...");
    return;
  }

  // Import the image from ROS message to OpenCV mat
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr->image;

  // Get time at which the image was taken. This time is used to stamp the estimated pose and also calculate the position of where to search for the makers in the image
  double time_to_predict = image_msg->header.stamp.toSec();

  if (callDetectLed(image)) // Only output the pose, if the pose was updated (i.e. a valid pose was found).
  {
    //Eigen::Matrix4d transform = trackable_object.getPredictedPose();
    /*
    Matrix6d cov = trackable_object_.getPoseCovariance();
    Eigen::Matrix4d transform = trackable_object_.getPredictedPose();

    ROS_DEBUG_STREAM("The transform: \n" << transform);
    ROS_DEBUG_STREAM("The covariance: \n" << cov);

    // Convert transform to PoseWithCovarianceStamped message
    predicted_pose_.header.stamp = image_msg->header.stamp;
    predicted_pose_.pose.pose.position.x = transform(0, 3);
    predicted_pose_.pose.pose.position.y = transform(1, 3);
    predicted_pose_.pose.pose.position.z = transform(2, 3);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.block<3, 3>(0, 0));
    predicted_pose_.pose.pose.orientation.x = orientation.x();
    predicted_pose_.pose.pose.orientation.y = orientation.y();
    predicted_pose_.pose.pose.orientation.z = orientation.z();
    predicted_pose_.pose.pose.orientation.w = orientation.w();

    // Add covariance to PoseWithCovarianceStamped message
    for (unsigned i = 0; i < 6; ++i)
    {
      for (unsigned j = 0; j < 6; ++j)
      {
        predicted_pose_.pose.covariance.elems[j + 6 * i] = cov(i, j);
      }
    }

    // Publish the pose
    pose_pub_.publish(predicted_pose_);*/

    
    cv::Mat visualized_image = image.clone();
    cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);
/*(cv::Mat &image, List2DPoints image_vectors_, const cv::Mat camera_matrix_K,
                                             const std::vector<double> camera_distortion_coeffs,
                                             cv::Rect region_of_interest,
                                             std::vector<cv::Point2f> distorted_detection_centers)*/
    Visualization::createVisualizationImage(visualized_image, image_vectors_2, camera_matrix_K_, camera_distortion_coeffs_,
                                          region_of_interest_, distorted_detection_centers_);
   
    //trackable_object_.augmentImage(visualized_image);

    // Publish image for visualization
    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.header = image_msg->header;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = visualized_image;

    image_pub_.publish(visualized_image_msg.toImageMsg());
    //}
  }
  else
  { // If pose was not updated
    ROS_WARN("Unable to resolve a pose.");
  }
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
/*void MPENode::dynamicParametersCallback(monocular_pose_estimator::MonocularPoseEstimatorConfig &config, uint32_t level)
{
  
  trackable_object_.detection_threshold_value_ = config.threshold_value;
  trackable_object_.gaussian_sigma_ = config.gaussian_sigma;
  trackable_object_.min_blob_area_ = config.min_blob_area;
  trackable_object_.max_blob_area_ = config.max_blob_area;
  trackable_object_.max_width_height_distortion_ = config.max_width_height_distortion;
  trackable_object_.max_circular_distortion_ = config.max_circular_distortion;
  trackable_object_.roi_border_thickness_ = config.roi_border_thickness;

  trackable_object_.setBackProjectionPixelTolerance(config.back_projection_pixel_tolerance);
  trackable_object_.setNearestNeighbourPixelTolerance(config.nearest_neighbour_pixel_tolerance);
  trackable_object_.setCertaintyThreshold(config.certainty_threshold);
  trackable_object_.setValidCorrespondenceThreshold(config.valid_correspondence_threshold);

  ROS_INFO("Parameters changed");
  
}*/

} // namespace monocular_pose_estimator


Eigen::Vector2d ComputePositionMutual(double alpha, double beta, double d){
  double r = 0.5*d/sin(alpha);
  
  // Position of the center
  double Cx = 0.0;
  double Cy = 0.5*d/tan(alpha);

  // From http://mathworld.wolfram.com/Circle-LineIntersection.html
  // Intersection of a line and of a circle, with new beta
  double OtherAngle = 0.5 * M_PI - beta;
  double x1 = 0; 
  double x2 = 50 * cos(OtherAngle);
  double y1 = -Cy; 
  double y2 = 50 * sin(OtherAngle) - Cy;

  double dx = x2-x1;
  double dy = y2-y1;

  double dr = sqrt(dx*dx + dy*dy);
  double D  = x1*y2 - x2*y1;
    
  double X = (D*dy+abs(dy)/dy*dx*sqrt(r*r*dr*dr-D*D))/dr/dr;
  double Y = (-D*dx + abs(dy)*sqrt(r*r*dr*dr-D*D))/dr/dr;
    
  Y = Y + Cy;

  Eigen::Vector2d p(X, Y);
  return  p;
}

Eigen::MatrixXd vrrotvec2mat(double p, Eigen::Vector3d r){
  float s = sin(p);
  float c = cos(p);
  float t = 1 - c;
  r.normalize();
  Eigen::Vector3d n = r;

  float x = n[0];
  float y = n[1];
  float z = n[2];
  Eigen::MatrixXd m(3,3);
  m(0,0) = t*x*x + c; m(0,1) = t*x*y - s*z; m(0,2) = t*x*z + s*y;
  m(1,0) = t*x*y + s*z; m(1,1) = t*y*y + c; m(1,2) = t*y*z - s*x;
  m(2,0) = t*x*z - s*y; m(2,1) = t*y*z + s*x; m(2,2) = t*z*z + c;
  return m;
}

/**
    ImageA1 = point gauche robot A
    ppA = centre de la caméra A ([0,0])
    fCamA = focal caméra A
    rdA = distance de la caméra du point à droite sur le robot A (POSITIF)
*/
Eigen::MatrixXd Compute3DMutualLocalisation(Eigen::Vector2d ImageA1, Eigen::Vector2d ImageA2,
                                            Eigen::Vector2d ImageB1, Eigen::Vector2d ImageB2,
                                            Eigen::Vector2d ppA, Eigen::Vector2d ppB,
                                            Eigen::Vector2d fCamA, Eigen::Vector2d fCamB,
                                            double rdA, double ldA, double rdB, double ldB,
                                            Eigen::Vector3d* pos, double* dist){
  Eigen::Vector3d PAM1((ImageB1[0]-ppB[0])/fCamB[0], (ImageB1[1]-ppB[1])/fCamB[1], 1);
  Eigen::Vector3d PAM2((ImageB2[0]-ppB[0])/fCamB[0], (ImageB2[1]-ppB[1])/fCamB[1], 1);
  PAM1.normalize();
  PAM2.normalize();
  double alpha = acos(PAM1.dot(PAM2));

  double d = rdA + ldA;

  //Eigen::Vector2d BLeftMarker(ImageA1[0], ImageA2[0]);
  //Eigen::Vector2d BRightMarker(ImageA1[1], ImageA2[1
  Eigen::Vector2d BLeftMarker = ImageA2;
  Eigen::Vector2d BRightMarker = ImageA1;
  
  Eigen::Vector2d PB1(BLeftMarker[0] + (ldB/(rdB+ldB)) * (BRightMarker[0] - BLeftMarker[0]),
                      BLeftMarker[1] + (ldB/(rdB+ldB)) * (BRightMarker[1] - BLeftMarker[1]));
  //cout << "BLeftMarker: \n" << BLeftMarker << endl;
  Eigen::Vector3d PB12((PB1[0]-ppA[0])/fCamA[0], (PB1[1]-ppA[1])/fCamA[1], 1);
  PB12.normalize();
  double phi = acos(PB12[0]);
  double beta = 0.5f * M_PI - phi;

  Eigen::Vector2d plane = ComputePositionMutual(alpha, beta, d);

  double EstimatedDistance = plane.norm();

  *pos =  PB12 * EstimatedDistance; 
  *dist = EstimatedDistance;
    //====================================================================
    //=========================Axis Angle Part============================
    //Create the two plans
    //Plan in B Refs
    /*[~,i] = min(BMarkersCOM(1,:));*/ 
  Eigen::Vector2d ALeftMarker = ImageB2; 
    /*[~,i] = max(BMarkersCOM(1,:));*/ 
  Eigen::Vector2d ARightMarker = ImageB1; 

  Eigen::Vector3d ALM((ALeftMarker[0]-ppB[0])/fCamB[0], (ALeftMarker[1]-ppB[1])/fCamB[1], 1);
  ALM.normalize();

  Eigen::Vector3d ARM((ARightMarker[0]-ppB[0])/fCamB[0], (ARightMarker[1]-ppB[1])/fCamB[1], 1);
  ARM.normalize();
  //Plan in A Refs
  Eigen::Vector3d AToB = PB12;
  //LeftMarker = NormalizeV([ldA;0;0]); WTF???
  Eigen::Vector3d LeftMarker(1, 0, 0);

  //Align the two plans
  Eigen::Vector3d NormalPlanInB = ALM.cross(ARM);
  Eigen::Vector3d NormalPlanInA = AToB.cross(LeftMarker);
  Eigen::Vector3d AxisAlignPlans = NormalPlanInB.cross(NormalPlanInA);
  NormalPlanInB.normalize();
  NormalPlanInA.normalize();
  AxisAlignPlans.normalize();
  double AngleAlignPlans = acos(NormalPlanInB.dot(NormalPlanInA));

  Eigen::MatrixXd AlignPlans = vrrotvec2mat(AngleAlignPlans, AxisAlignPlans);

  //Align the vector of the cameraA seen from B with the plan
  Eigen::Vector3d CameraASeenFromB(
                      ((ALeftMarker[0] + (ldA/(rdA+ldA))*(ARightMarker[0] - ALeftMarker[0]))-ppB[0])/fCamB[0],
                      ((ALeftMarker[1] + (ldA/(rdA+ldA))*(ARightMarker[1] - ALeftMarker[1]))-ppB[1])/fCamB[1],
                      1);
  CameraASeenFromB.normalize();
  Eigen::Vector3d alignedBToA = AlignPlans * CameraASeenFromB;
  //Turn the vector BToA to make it align with AToB
  Eigen::Vector3d AxisAlignABwBA = alignedBToA.cross(AToB);
  AxisAlignABwBA.normalize();
  //Since we know that cameras are facing each other, we rotate pi
  double AngleAlignABwBA = acos(alignedBToA.dot(AToB)) - M_PI;
  
  Eigen::MatrixXd AlignVectors = vrrotvec2mat(AngleAlignABwBA, AxisAlignABwBA);

  return AlignVectors * AlignPlans;
  //cout << "Rotation:\n" << Rotation << endl;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "monocular_pose_tracker");

  monocular_pose_estimator::MPENode mpe_node;
  ros::spin();

  return 0;
  //Eigen::Matrix<double, 3, 4> m;
  Eigen::Vector2d ImageA1(43.2675,  -230.8107); // En colonne
  Eigen::Vector2d ImageA2(-117.6621,  -184.6691); 

  Eigen::Vector2d ImageB1( 66.7481,  72.4816); // En colonne
  Eigen::Vector2d ImageB2(-221.8502,  153.4686); 
 /* Eigen::Vector2d ImageA1(595.8564,  27.7171); // En colonne
  Eigen::Vector2d ImageA2(516.2996,  -26.9985); 

  Eigen::Vector2d ImageB1(144.2211,  56.6879); // En colonne
  Eigen::Vector2d ImageB2(46.9843,  -19.3582); */
  int f = 1000;
  double d = 80;

  Eigen::Vector2d fCam(f,  f); 
  Eigen::Vector2d pp(0,  0); 
  double rdA, ldA, rdB, ldB;
  rdA = 0.9f * d;
  ldA = rdA;
  rdB = 0.5f * d;
  ldB = rdB;
  Eigen::Vector3d pos(0,0,0);
  double dist;

  Eigen::MatrixXd rotation = Compute3DMutualLocalisation(ImageA1, ImageA2, ImageB1, ImageB2, pp, pp, fCam, fCam, rdA, ldA, rdB, ldB, &pos, &dist);
  
  cout<<"Position: "<<pos<<endl;
  cout<<"Distance: "<<dist<<endl;
  cout<<"Rotation: "<<rotation<<endl;
}
