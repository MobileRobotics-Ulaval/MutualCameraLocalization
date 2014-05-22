/*
 * led_detector.cpp
 *
 * Created on: July 29, 2013
 * Author: Karl Schwabe
 */

/**
 * \file led_detector.cpp
 * \brief File containing the function definitions required for detecting LEDs and visualising their detections and the pose of the tracked object.
 *
 */

#include "ros/ros.h"
#include "mutual_camera_localization/led_detector.h"

namespace mutual_camera_localizator
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d

void LEDDetector::LedFilteringTrypon(const cv::Mat &gaussian_image, const double &min_blob_area, const double &max_blob_area, const double &max_circular_distortion,
               const double &radius_ratio_tolerance, 
               const double &min_ratio_ellipse, const double &max_ratio_ellipse,
               const double &distance_ratio, const double &distance_ratio_tolerance,
               const double &acos_tolerance, int OutputFlag, const double &acos_tolerance
                           std::vector<cv::Point2f> &distorted_detection_centers) {

  distorted_detection_centers.clear();
  std::vector<cv::Point2f> detection_centers;
  bool FoundLEDs = false; // Flag to indicate if we have found the LEDs or not
  cv::Rect ROI= cv::Rect(0,0,gaussian_image.cols,gaussian_image.rows);

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Vector for containing the detected points that will be undistorted later
    // std::vector<cv::Point2f> distorted_points;
    int DataIndex = 1; // For matlab-compatible output, when chasing the parameters.
  
    std::vector<cv::Point2f> KeptContoursPosition;
    std::vector<double> KeptRadius, KeptAvgIntensity;

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4; // Average radius

    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

    if (area > 0.01) {
      double width_height_distortion = std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width));
      double circular_distortion1 = std::abs(1 - (area / (CV_PI * pow(rect.width / 2, 2.0))));
      double circular_distortion2 = std::abs(1 - (area / (CV_PI * pow(rect.height / 2, 2.0))));

      cv::RotatedRect minEllipse;
      double RatioEllipse = 1.0;
      if (contours[i].size()>4) {
        minEllipse = cv::fitEllipse(cv::Mat(contours[i]));
        RatioEllipse = float(minEllipse.boundingRect().width+1.0)/float(minEllipse.boundingRect().height+1.0);  // the 0.5 is to increase immunity to small circles.
      }
      int x, y;
      double total_intensity=0.0,avg_intensity;
      for (x = rect.x; x<rect.x+rect.width; x++) {
        for (y = rect.y; y<rect.y+rect.height; y++) {
          cv::Scalar intensity = gaussian_image.at<uchar>(y, x);
          total_intensity+= float(intensity.val[0]);
        }
      }
      avg_intensity = total_intensity/area;

      if (OutputFlag==1) {
        // We want to output some data for further analysis in matlab.
        printf("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %6.2f, %.2f, %d\n", mc.x, mc.y, area, radius, width_height_distortion,circular_distortion1,circular_distortion2,total_intensity,avg_intensity,RatioEllipse,DataIndex);
        DataIndex++;
      }
  
      // We will prune the blobs set based on appearance. These were found using matlab.
      if ((area < max_blob_area) && (circular_distortion1 < max_circular_distortion) && (circular_distortion2 < max_circular_distortion)
            && (RatioEllipse < max_ratio_ellipse) && (RatioEllipse > min_ratio_ellipse )) {
        // These will be used further down the filtering pipeline
        // Ideally we would sort in order of intensity, and do a n choose k on a sliding window of that ranked intensity.
        KeptContoursPosition.push_back(mc);
        KeptRadius.push_back(radius);
        KeptAvgIntensity.push_back(avg_intensity);
      }
    }
    }

  // Filtering step #2: doing all the permutations to find the one with the best characteristics.
  // We are basically looking at:
  //   -Three blobs in line (line angle tolerance)
  //   -With a 3-to-1 ratio between the shortest distance to the longest distance
  //   -And similar radii
  int nBlob = KeptContoursPosition.size();
  cv::Point vec1, vec2, vec3, shortArm, longArm;
  double norm1, norm2, norm3, cosArms;
  int index;
  double RatioRadii, RatioDistance, minDist, maxDist, maxIntensity=0.0;
  int l1, l2, l3, BestCombo[3];
  for (l1 = 0; l1<(nBlob-2);l1++) {
      for (l2 = (l1+1); l2<(nBlob-1); l2++) {
      for (l3 = (l2+1); l3<nBlob; l3++) {
        // This is n-choose-k permutations

        // Test 1: Radius ratio tolerance
        // Let's start with the computations that take the least amount of time
        RatioRadii = std::min(std::min(KeptRadius[l1],KeptRadius[l2]),KeptRadius[l3])/std::max(std::max(KeptRadius[l1],KeptRadius[l2]),KeptRadius[l3]);
        if (std::abs(RatioRadii-1.0)>radius_ratio_tolerance) continue;

        // Ok now we have no choice. We have to compute the 3 vectors representing the 3-choose-2 combination of leds
        vec1 = KeptContoursPosition[l1]-KeptContoursPosition[l2];
        vec2 = KeptContoursPosition[l2]-KeptContoursPosition[l3];
        vec3 = KeptContoursPosition[l3]-KeptContoursPosition[l1];
        norm1 = cv::norm(vec1);
        norm2 = cv::norm(vec2);
        norm3 = cv::norm(vec3);

        // Test 2: Led Distance tolerance
        minDist = std::min(std::min(norm1,norm2),norm3);
        maxDist = std::max(std::max(norm1,norm2),norm3);
        RatioDistance = maxDist/minDist;
        if (std::abs(RatioDistance-distance_ratio)>distance_ratio_tolerance) continue;

        // Now we have to find the actual shortest arm
        if (minDist == norm1) shortArm = vec1;
        else if (minDist == norm2) shortArm = vec2;
        else shortArm = vec3;

        // Now we have to find the actual longest arm
        if (maxDist == norm1) longArm = vec1;
        else if (maxDist == norm2) longArm = vec2;
        else longArm = vec3;

        // Test 3: tolerance on the angle between the arms
        cosArms = std::abs(shortArm.dot(longArm)/(minDist*maxDist));
        //double cosArms2 = std::abs(shortArm.dot(longArm)/(cv::norm(shortArm)*cv::norm(longArm)));
        //printf("combo %d %d %d has passed first 3 tests, with cos=%.2f, %.2f!\n",l1, l2, l3,cosArms, cosArms2);
        double val = std::abs(std::abs(cosArms)-1.0);
        if (val>acos_tolerance) continue;
        

        // Test 4: we keep the one with the highest average intensity
        double sumIntensity = KeptAvgIntensity[l1]+KeptAvgIntensity[l2]+KeptAvgIntensity[l3];
        if (sumIntensity > maxIntensity) {
          maxIntensity = sumIntensity;
          FoundLEDs = true;
          BestCombo[0] = l1;
          BestCombo[1] = l2;
          BestCombo[2] = l3;
        }
      }
    }
    }

  if (FoundLEDs) {
    // We then push the best results
    for (index = 0; index < 3; index++) {
      detection_centers.push_back(KeptContoursPosition[BestCombo[index]]);
    }

    // Order the dot from left to right
    if(detection_centers[0].x <= detection_centers[1].x && detection_centers[0].x <= detection_centers[2].x){
      distorted_detection_centers.push_back(detection_centers[0]);
      if(detection_centers[1].x <= detection_centers[2].x){
        distorted_detection_centers.push_back(detection_centers[1]);
        distorted_detection_centers.push_back(detection_centers[2]);
      }
      else{
        distorted_detection_centers.push_back(detection_centers[2]);
        distorted_detection_centers.push_back(detection_centers[1]);
      }
    }
    else if(detection_centers[1].x <= detection_centers[0].x && detection_centers[1].x <= detection_centers[2].x){
      distorted_detection_centers.push_back(detection_centers[1]);
      if(detection_centers[0].x <= detection_centers[2].x){
        distorted_detection_centers.push_back(detection_centers[0]);
        distorted_detection_centers.push_back(detection_centers[2]);
      }
      else{
        distorted_detection_centers.push_back(detection_centers[2]);
        distorted_detection_centers.push_back(detection_centers[0]);
      }
    }
    else{
      distorted_detection_centers.push_back(detection_centers[2]);
      if(detection_centers[0].x <= detection_centers[1].x){
        distorted_detection_centers.push_back(detection_centers[0]);
        distorted_detection_centers.push_back(detection_centers[1]);
      }
      else{
        distorted_detection_centers.push_back(detection_centers[1]);
        distorted_detection_centers.push_back(detection_centers[0]);
      }
    }

  } 

  else {
    // We flush whatever was in there, to indicate that we didn't find anything.
    distorted_detection_centers.clear();
  }
}

// LED detector
void LEDDetector::findLeds(const cv::Mat &image, cv::Rect ROI, const int &threshold_value, const double &gaussian_sigma,
                           const double &min_blob_area, const double &max_blob_area,
                           const double &max_width_height_distortion, const double &max_circular_distortion,
                           List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           const cv::Mat &camera_matrix_P)
{
  // Threshold the image
  cv::Mat bw_image;
  //cv::threshold(image, bwImage, threshold_value, 255, cv::THRESH_BINARY);

  //cv::threshold(image(ROI), bw_image, threshold_value, 255, cv::THRESH_TOZERO);

  // Gaussian blur the image
  cv::Mat gaussian_image;
  cv::Size ksize; // Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
  ksize.width = 0;
  ksize.height = 0;
 // ROS_INFO("1Everyone of them is a fake EVERYONE");
  GaussianBlur(image(ROI), gaussian_image, ksize, gaussian_sigma, gaussian_sigma, cv::BORDER_DEFAULT);

  //ROS_INFO("2Everyone of them is a fake EVERYONE");
  //cv::imshow( "Gaussian", gaussian_image );

  // Find all contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  //ROS_INFO("3Everyone of them is a fake EVERYONE");
  unsigned numPoints = 0; // Counter for the number of detected LEDs

  // Vector for containing the detected points that will be undistorted later
  std::vector<cv::Point2f> distorted_points;

  // Identify the blobs in the image
  for (unsigned i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4; // Average radius

    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

    // Look for round shaped blobs of the correct size
    if (area >= min_blob_area && area <= max_blob_area
        && std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))
            <= max_width_height_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) <= max_circular_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) <= max_circular_distortion)
    {
      distorted_points.push_back(mc);
      numPoints++;
    }
  }

  // These will be used for the visualization
  distorted_detection_centers = distorted_points;

  if (numPoints > 0)
  {
    // Vector that will contain the undistorted points
    std::vector<cv::Point2f> undistorted_points;

    // Undistort the points
    cv::undistortPoints(distorted_points, undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
                        camera_matrix_P);

    // Resize the vector to hold all the possible LED points
    pixel_positions.resize(numPoints);

    // Populate the output vector of points
    for (unsigned j = 0; j < numPoints; ++j)
    {
      Eigen::Vector2d point;
      point(0) = undistorted_points[j].x;
      point(1) = undistorted_points[j].y;
      pixel_positions(j) = point;
    }
  }
}

cv::Rect LEDDetector::determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                                   const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                   const cv::Mat &camera_matrix_P)
{
  double x_min = INFINITY;
  double x_max = 0;
  double y_min = INFINITY;
  double y_max = 0;

  for (unsigned i = 0; i < pixel_positions.size(); ++i)
  {
    if (pixel_positions(i)(0) < x_min)
    {
      x_min = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(0) > x_max)
    {
      x_max = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(1) < y_min)
    {
      y_min = pixel_positions(i)(1);
    }
    if (pixel_positions(i)(1) > y_max)
    {
      y_max = pixel_positions(i)(1);
    }
  }

  std::vector<cv::Point2f> undistorted_points;

  undistorted_points.push_back(cv::Point2f(x_min, y_min));
  undistorted_points.push_back(cv::Point2f(x_max, y_max));

  std::vector<cv::Point2f> distorted_points;

  // Distort the points
  distortPoints(undistorted_points, distorted_points, camera_matrix_K, camera_distortion_coeffs, camera_matrix_P);

  double x_min_dist = distorted_points[0].x;
  double y_min_dist = distorted_points[0].y;
  double x_max_dist = distorted_points[1].x;
  double y_max_dist = distorted_points[1].y;

  double x0 = std::max(0.0, std::min((double)image_size.width, x_min_dist - border_size));
  double x1 = std::max(0.0, std::min((double)image_size.width, x_max_dist + border_size));
  double y0 = std::max(0.0, std::min((double)image_size.height, y_min_dist - border_size));
  double y1 = std::max(0.0, std::min((double)image_size.height, y_max_dist + border_size));

  cv::Rect region_of_interest;

  // if region of interest is too small, use entire image
  // (this happens, e.g., if prediction is outside of the image)
  if (x1 - x0 < 1 || y1 - y0 < 1)
  {
    region_of_interest = cv::Rect(0, 0, image_size.width, image_size.height);
  }
  else
  {
    region_of_interest.x = x0;
    region_of_interest.y = y0;
    region_of_interest.width = x1 - x0;
    region_of_interest.height = y1 - y0;
  }

  return region_of_interest;
}

void LEDDetector::distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst,
                                const cv::Mat & camera_matrix_K, const std::vector<double> & distortion_matrix,
                                const cv::Mat & projection_matrix_P)
{
  dst.clear();
  double fx_K = camera_matrix_K.at<double>(0, 0);
  double fy_K = camera_matrix_K.at<double>(1, 1);
  double cx_K = camera_matrix_K.at<double>(0, 2);
  double cy_K = camera_matrix_K.at<double>(1, 2);

  double fx_P = projection_matrix_P.at<double>(0, 0);
  double fy_P = projection_matrix_P.at<double>(1, 1);
  double cx_P = projection_matrix_P.at<double>(0, 2);
  double cy_P = projection_matrix_P.at<double>(1, 2);

  double k1 = distortion_matrix[0];
  double k2 = distortion_matrix[1];
  double p1 = distortion_matrix[2];
  double p2 = distortion_matrix[3];
  double k3 = distortion_matrix[4];

  for (unsigned int i = 0; i < src.size(); i++)
  {
    // Project the points into the world
    const cv::Point2d &p = src[i];
    double x = (p.x - cx_P) / fx_P;
    double y = (p.y - cy_P) / fy_P;
    double xCorrected, yCorrected;

    // Correct distortion
    {
      double r2 = x * x + y * y;

      // Radial distortion
      xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      // Tangential distortion
      xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
      yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
    }

    // Project coordinates onto image plane
    {
      xCorrected = xCorrected * fx_K + cx_K;
      yCorrected = yCorrected * fy_K + cy_K;
    }
    dst.push_back(cv::Point2d(xCorrected, yCorrected));
  }
}

} // namespace
