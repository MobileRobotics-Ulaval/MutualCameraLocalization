/*
 * mutual_camera_localizator_node.cpp
 *
 * Created on: Jul 29, 2013
 *      Author: Karl Schwabe
 *  Edited on:  May 14 2014
 *      Author: Philippe Bbin
 */

/** \file mutual_camera_localizator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */
using namespace std;
#include "mutual_camera_localization/mutual_camera_localization_node.h"

namespace mutual_camera_localizator
{

/**
 * Constructor of the Monocular Pose Estimation Node class
 *
 */
MCLNode::MCLNode(): diffMax(ros::Duration(1000000))
{
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<mutual_camera_localization::MutualCameraLocalizationConfig>::CallbackType cb_;
  cb_ = boost::bind(&MCLNode::dynamicParametersCallback, this, _1, _2);
  dr_server_.setCallback(cb_);

  // Initialize subscribers
  image_subA_ = nh_.subscribe("/cameraA/image_raw", 1, &MCLNode::imageCallbackA, this);
  image_subB_ = nh_.subscribe("/cameraB/image_raw", 1, &MCLNode::imageCallbackB, this);
  camera_info_sub_ = nh_.subscribe("/cameraA/camera_info", 1, &MCLNode::cameraInfoCallback, this);

  // Initialize pose publisher
  //pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose", 1);
  pose_pub_ = nh_.advertise<visualization_msgs::Marker>("estimated_pose", 1);
  pose_pub_point_ = nh_.advertise<geometry_msgs::PoseStamped>("estimated_pose_arrow", 1);
  //pose_pub_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);
  initMarker();


  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pubA_ = image_transport.advertise("image_with_detectionsA", 1);
  image_pubB_ = image_transport.advertise("image_with_detectionsB", 1);
}

/**
 * Destructor of the Mutual camera localization Node class
 *
 */
MCLNode::~MCLNode()
{

}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MCLNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  if (!have_camera_info_)
  {
    cam_info_ = *msg;

    // Calibrated camera
    Eigen::Matrix<double, 3, 4> camera_matrix;
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

//void MCLNode::calculateImageVectors(Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> image_points){}

bool MCLNode::callDetectLed(cv::Mat image, const bool camA){
  region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

    // Do detection of LEDs in image
  Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> detected_led_positions;
  //distorted_detection_centers_;
  /*
  LEDDetector::findLeds(image, region_of_interest_, 0, gaussian_sigma_, min_blob_area_,
                        max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
                        detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
                        camera_distortion_coeffs_, camera_matrix_P_);
                        */
  LEDDetector::LedFilteringTrypon(image, min_blob_area_, max_blob_area_, max_circular_distortion_,
               radius_ratio_tolerance_, ratio_int_tolerance_, hor_line_angle_,
               ratio_ellipse_min_, ratio_ellipse_max_,
               pos_ratio_, pos_ratio_tolerance_,
               line_angle_tolerance_, 0,
                           distorted_detection_centers_, undistorted_detection_centers_,
                           camera_matrix_K_,
                        camera_distortion_coeffs_, camera_matrix_P_);


  if (distorted_detection_centers_.size() >= 2) // If found enough LEDs, Reinitialise
  {

    //printf("P1: (%f, %f)\n",(float)detected_led_positions(0)(0),(float)detected_led_positions(0)(1));
    //printf("P2: (%f, %f)\n",(float)detected_led_positions(1)(0),(float)detected_led_positions(1)(1));
    Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1>* image_vectors;
    if(camA)
      image_vectors = &image_vectorsA_;
    else
      image_vectors = &image_vectorsB_;
    unsigned num_image_points = distorted_detection_centers_.size();
    image_vectors->resize(num_image_points);
    Eigen::Vector2d single_vector;
   
    for (unsigned i = 0; i < num_image_points; ++i){
      //single_vector(0) = (undistorted_detection_centers_[i].x - camera_projection_matrix_(0, 2)) / camera_projection_matrix_(0, 0);
      //single_vector(1) = (undistorted_detection_centers_[i].y - camera_projection_matrix_(1, 2)) / camera_projection_matrix_(1, 1);
      
      (*image_vectors)(i)(0) = undistorted_detection_centers_[i].x;//single_vector;// / single_vector.norm();
      (*image_vectors)(i)(1) = undistorted_detection_centers_[i].y;
    }
  }
  else
  { // Too few LEDs found
    return false;
  }
  return true;
}


void MCLNode::imageCallbackA(const sensor_msgs::Image::ConstPtr& image_msg){
  imageCallback(image_msg, true);
}
void MCLNode::imageCallbackB(const sensor_msgs::Image::ConstPtr& image_msg){
  imageCallback(image_msg, false);
}

/**
 * The callback function that is executed every time an image is received. It runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void MCLNode::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg, const bool camA)
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
  //double time_to_predict = image_msg->header.stamp.toSec();

  if (callDetectLed(image, camA)){ // Only output the pose, if the pose was updated (i.e. a valid pose was found).
    bool camB_ready = image_msg->header.stamp - camB_time_ <= diffMax;
    //bool camB_ready = true;
    //If we are the camB we declare that we are ready
    if(!camA)
        camB_time_ = image_msg->header.stamp;


    if(camA && !camB_ready)
        ROS_WARN("CamB delay too long: %f", (image_msg->header.stamp - camB_time_).toSec());


    if(camA && camB_ready){
        //ROS_WARN("Let's shoot data!");
        Eigen::Vector2d ImageA1(image_vectorsA_(0)(0),  image_vectorsA_(0)(1)); 
        Eigen::Vector2d ImageA2(image_vectorsA_(2)(0),  image_vectorsA_(2)(1)); 

        Eigen::Vector2d ImageB1(image_vectorsB_(0)(0),  image_vectorsB_(0)(1)); 
        Eigen::Vector2d ImageB2(image_vectorsB_(2)(0),  image_vectorsB_(2)(1)); 
        //int fx = camera_matrix_K_[0][0];
        //int fy = camera_matrix_K_[1][1];
        Eigen::Vector2d fCam(camera_matrix_K_.at<double>(0, 0), camera_matrix_K_.at<double>(1, 1)); 
        Eigen::Vector2d pp(camera_matrix_K_.at<double>(0, 2), camera_matrix_K_.at<double>(1, 2)); 
        //Eigen::Vector2d pp(camera_matrix_K_.at<double>(0, 2) -  image.cols/2, camera_matrix_K_.at<double>(1, 2) - image.rows/2); 
        //Eigen::Vector2d pp(0, 0); 
        //cout<<"ImageA1: "<<ImageA1<<endl;
        //cout<<"ImageA2: "<<ImageA2<<endl;
        //double rdA, ldA, rdB, ldB;
       /* ldA = 0.13;
        rdA = 0.14;
        ldB = 0.11;
        rdB = 0.125;*/

        Eigen::Vector3d pos(0,0,0);
        double dist;

        Eigen::Matrix< double, 3, 3 > rotation = Compute3DMutualLocalisation(ImageA1, ImageA2, ImageB1, ImageB2, pp, pp, fCam, fCam,
                                                               rdA_, ldA_, rdB_, ldB_, &pos, &dist);
        //cout<<"fCam: "<<fCam<<endl;
        //cout<<"pp: "<<pp<<endl;
        cout<<"Position: "<<pos<<endl;
        cout<<"Distance: "<<dist<<endl;
        cout<<"Rotation: "<<rotation<<endl;

        //marker_pose_.header.stamp = ros::Time::now();

        marker_pose_.pose.position.x = pos[2];
        marker_pose_.pose.position.y = pos[0];
        marker_pose_.pose.position.z = -pos[1];
        //Eigen::Quaterniond orientation = Eigen::Quaterniond(rotation * Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX() * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond orientation = Eigen::Quaterniond(rotation);
        //tf frame(cube) poseStamped odometry
        //cuba parent -  cubeB child_id
        marker_pose_.pose.orientation.x = orientation.z();
        marker_pose_.pose.orientation.y = orientation.x();
        marker_pose_.pose.orientation.z = -orientation.y();
        marker_pose_.pose.orientation.w = orientation.w();

        marker_pose_.header.stamp = ros::Time::now();



        pose_pub_.publish(marker_pose_);

        geometry_msgs::PoseStamped pos_packet;

        pos_packet.header.frame_id = "/cubeB";
        pos_packet.header.stamp = ros::Time::now();

        pos_packet.pose = marker_pose_.pose;

        pose_pub_point_.publish(pos_packet);

        /*broadcaster.sendTransform(
        tf::StampedTransform(
        marker_pose_,
        ros::Time::now(),"cubeA", "cubeB"));*/
        /*tf::Transform transform;
        transform.setOrigin( tf::Vector3(marker_pose_.pose.position.x,
                                         marker_pose_.pose.position.y,
                                         marker_pose_.pose.position.z));
        tf::Quaternion q(marker_pose_.pose.orientation.x,
                         marker_pose_.pose.orientation.y,
                         marker_pose_.pose.orientation.z,
                         marker_pose_.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/cubeA", "/cubeB"));*/
        /*
        transform.setOrigin( tf::Vector3(0, 0, 0);
        transform.setRotation(tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "cubeA"));*/
    }
  }
  else
  { // If pose was not updated
    ROS_WARN("Unable to resolve a pose.");
  }

  cv::Mat visualized_image = image.clone();
  cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);

  if(camA)
    Visualization::createVisualizationImage(visualized_image, image_vectorsA_, camera_matrix_K_, camera_distortion_coeffs_,
                                        region_of_interest_, distorted_detection_centers_);
  else
    Visualization::createVisualizationImage(visualized_image, image_vectorsB_, camera_matrix_K_, camera_distortion_coeffs_,
                                        region_of_interest_, distorted_detection_centers_);

  // Publish image for visualization
  cv_bridge::CvImage visualized_image_msg;
  visualized_image_msg.header = image_msg->header;
  visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  visualized_image_msg.image = visualized_image;

  if(camA)
    image_pubA_.publish(visualized_image_msg.toImageMsg());
  else
    image_pubB_.publish(visualized_image_msg.toImageMsg());
}

void MCLNode::initMarker(){
    marker_pose_.header.frame_id = "/cubeB";
    marker_pose_.ns = "arrows";
    marker_pose_.id = 1;
    marker_pose_.type = visualization_msgs::Marker::CUBE;
    marker_pose_.lifetime = ros::Duration();

    marker_pose_.action = visualization_msgs::Marker::ADD;
    marker_pose_.scale.x = 1;
    marker_pose_.scale.y = 1;
    marker_pose_.scale.z = 1;

    marker_pose_.color.r = 0.0f;
    marker_pose_.color.g = 1.0f;
    marker_pose_.color.b = 0.0f;
    marker_pose_.color.a = 1.0;
/*
    marker_arrow_.header.frame_id = "/arrow";
    marker_arrow_.ns = "arrows";
    marker_arrow_.id = 2;
    marker_arrow_.type = visualization_msgs::Marker::ARROW;
    marker_arrow_.lifetime = ros::Duration();

    marker_arrow_.action = visualization_msgs::Marker::ADD;
    marker_arrow_.scale.x = 1;
    marker_arrow_.scale.y = 0.1;
    marker_arrow_.scale.z = 0.1;

    marker_arrow_.color.r = 1.0f;
    marker_arrow_.color.g = 0.0f;
    marker_arrow_.color.b = 0.0f;
    marker_arrow_.color.a = 1.0;

    geometry_msgs::Point p;
      p.x = 0;
      p.y = 0;
      p.z = 0;

    marker_arrow_.points.push_back(p);
    marker_arrow_.points.push_back(p);*/
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void MCLNode::dynamicParametersCallback(mutual_camera_localization::MutualCameraLocalizationConfig &config, uint32_t level){
  gaussian_sigma_ = config.gaussian_sigma;
  min_blob_area_ = config.min_blob_area;
  max_blob_area_ = config.max_blob_area;
  max_circular_distortion_ = config.max_circular_distortion;
  ratio_ellipse_min_ = config.ratio_ellipse_min;
  ratio_ellipse_max_ = config.ratio_ellipse_max;
  pos_ratio_ = config.pos_ratio;
  pos_ratio_tolerance_ = config.pos_ratio_tolerance;
  line_angle_tolerance_ = config.line_angle_tolerance;
  hor_line_angle_ = config.hor_line_angle;

  radius_ratio_tolerance_ = config.radius_ratio_tolerance;
  ratio_int_tolerance_ = config.ratio_int_tolerance;

  ldA_ = config.pos_left_led_cam_a; rdA_ = config.pos_right_led_cam_a;
  ldB_ = config.pos_left_led_cam_b; rdB_ = config.pos_right_led_cam_b;

  //max_width_height_distortion_ = config.max_width_height_distortion;
  //radius_ratio_tolerance_ = config.radius_ratio_tolerance;
  //ratio_int_tolerance_ = config.ratio_int_tolerance;

  ROS_INFO("Parameters changed");
}



Eigen::Vector2d MCLNode::ComputePositionMutual(double alpha, double beta, double d){
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

Eigen::MatrixXd MCLNode::vrrotvec2mat(double p, Eigen::Vector3d r){
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
Eigen::MatrixXd MCLNode::Compute3DMutualLocalisation(Eigen::Vector2d ImageA1, Eigen::Vector2d ImageA2,
                                            Eigen::Vector2d ImageB1, Eigen::Vector2d ImageB2,
                                            Eigen::Vector2d ppA, Eigen::Vector2d ppB,
                                            Eigen::Vector2d fCamA, Eigen::Vector2d fCamB,
                                            double rdA, double ldA, double rdB, double ldB,
                                            Eigen::Vector3d* pos, double* dist){
  /*cout<<"-Parameters-"<<endl;
  cout<<"ImageA1:"<<ImageA1<<endl;
  cout<<"ImageA2:"<<ImageA2<<endl;
  cout<<"ImageB1:"<<ImageB1<<endl;
  cout<<"ImageB2:"<<ImageB2<<endl;
  cout<<"ppA:"<<ppA<<endl;
  cout<<"ppB:"<<ppB<<endl;
  cout<<"fCamA:"<<fCamA<<endl;
  cout<<"fCamB:"<<fCamB<<endl;
  cout<<"rdA:"<<rdA<<endl;
  cout<<"ldA:"<<ldA<<endl;
  cout<<"rdB:"<<rdB<<endl;
  cout<<"ldB:"<<ldB<<endl;*/

  Eigen::Vector3d PAM1((ImageB1[0]-ppB[0])/fCamB[0], (ImageB1[1]-ppB[1])/fCamB[1], 1);
  Eigen::Vector3d PAM2((ImageB2[0]-ppB[0])/fCamB[0], (ImageB2[1]-ppB[1])/fCamB[1], 1);
  PAM1.normalize();
  PAM2.normalize();
  double alpha = acos(PAM1.dot(PAM2));
  printf("Alpha: %f\n",alpha);

  double d = rdA + ldA;

  //Eigen::Vector2d BLeftMarker(ImageA1[0], ImageA2[0]);
  //Eigen::Vector2d BRightMarker(ImageA1[1], ImageA2[1
  Eigen::Vector2d BLeftMarker = ImageA2;
  Eigen::Vector2d BRightMarker = ImageA1;
  
  Eigen::Vector2d PB1(BLeftMarker[0] + (ldB/(rdB+ldB)) * (BRightMarker[0] - BLeftMarker[0]),
                      BLeftMarker[1] + (ldB/(rdB+ldB)) * (BRightMarker[1] - BLeftMarker[1]));

  //cout<<"PB1: "<<PB1<<endl;
  //cout << "BLeftMarker: \n" << BLeftMarker << endl;
  Eigen::Vector3d PB12((PB1[0]-ppA[0])/fCamA[0], (PB1[1]-ppA[1])/fCamA[1], 1);
  PB12.normalize();
  double phi = acos(PB12[0]);
  double beta = 0.5f * M_PI - phi;
  printf("Beta: %f\n",beta);

  Eigen::Vector2d plane = ComputePositionMutual(alpha, beta, d);

  //cout<<"plane: "<<plane<<endl;
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

} // namespace mutual_camera_localizator_node

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mutual_camera_localization");

  mutual_camera_localizator::MCLNode mpe_node;
  ros::spin();
  //Eigen::Matrix<double, 3, 4> m;
  /*Eigen::Vector2d ImageA1(-4.6882,  -2.9915); // En colonne
  Eigen::Vector2d ImageA2(-4.6299,  -2.8090); 

  Eigen::Vector2d ImageB1(-3.7757, 2.147); // En colonne
  Eigen::Vector2d ImageB2(2.0148,  -1.0384); 
  double f = 1000.0;
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

  Eigen::MatrixXd rotation = mutual_camera_localizator::MCLNode::Compute3DMutualLocalisation(ImageA1*1000.0, ImageA2*1000.0, ImageB1*10000.0, ImageB2*10000000.0, pp, pp, fCam, fCam, rdA, ldA, rdB, ldB, &pos, &dist);
  
  cout<<"Position: "<<pos<<endl;
  cout<<"Distance: "<<dist<<endl;
  cout<<"Rotation: "<<rotation<<endl;*/

  return 0;
}
