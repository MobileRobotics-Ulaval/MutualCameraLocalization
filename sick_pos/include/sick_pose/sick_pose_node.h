#ifndef SICK_POSE_NODE_H_
#define SICK_POSE_NODE_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <sick_pose/sickPoseConfig.h>


#include <Eigen/StdVector>


namespace sick_pose
{
typedef  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ListVector2d;

class SP{
public:
	SP(ros::NodeHandle n);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	ListVector2d findCube(std::vector<geometry_msgs::Point32> &data);
	Eigen::Vector2d ComputeCubeCenter(Eigen::Vector2d Q1, Eigen::Vector2d Q2);
	void ComputeCubeCenterWithLine(ListVector2d pointsInLine, int begin, int end, Eigen::Vector2d & p1,  Eigen::Vector2d & p2, std::vector<geometry_msgs::Point32> &data);
	int SplitAndMerge(ListVector2d data,
					  int begin,
					  int end,
					  double & dist);
	double smallestAngle(double old, double next);

	void initMarker();
	void toCVS(sensor_msgs::PointCloud &cloud);
	void dynamicParametersCallback(sick_pose::sickPoseConfig &config, uint32_t level);
private:
	bool receive_scan_;
	bool cube_initiation;
	std::vector<Eigen::Vector2d> cubes;
  std::vector<double> cubesAngles;

	std::vector<int> missing_association;

  ros::Publisher marker_pub_;
  ros::Publisher cubeA_pub_;
  ros::Publisher cubeB_pub_;
  ros::Publisher marker_cubeA_pub_;
  ros::Publisher marker_cubeB_pub_;
  ros::Publisher scan_aug_pub_;
  ros::Publisher zone_pub_;
  ros::Publisher cube_poly_pub_;
  ros::Subscriber laser_scan_;

  geometry_msgs::PolygonStamped zone;

  ros::NodeHandle nh;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  int NBRCUBES;

  visualization_msgs::Marker cubeA_marker_;
  visualization_msgs::Marker cubeB_marker_;
  geometry_msgs::PoseStamped pose_;
  //tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

  dynamic_reconfigure::Server<sick_pose::sickPoseConfig> dr_server_; //!< The dynamic reconfigure server
  dynamic_reconfigure::Server<sick_pose::sickPoseConfig>::CallbackType cb_; //!< The dynamic reconfigure callback type

  double right_wall, left_wall, front_wall, back_wall, front_room_delim, left_room_delim;
  double cluster_distance_threshold, min_cluster_size, min_line_length, max_line_length, edge_split_and_merge_threshold, split_and_merge_threshold, max_translation;
  int max_tick_ghost;

};

}


#endif /* SICK_POSE_NODE_H_ */
