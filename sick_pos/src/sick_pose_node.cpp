#include "sick_pose/sick_pose_node.h"

using namespace std;

namespace sick_pose
{
SP::SP(ros::NodeHandle n) : 
    n_(n),
    receive_scan_(false),
    cube_initiation(false)
  {
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<sick_pose::sickPoseConfig>::CallbackType cb_;
  cb_ = boost::bind(&SP::dynamicParametersCallback, this, _1, _2);
  dr_server_.setCallback(cb_);

	laser_scan_ = n_.subscribe("/scan", 1, &SP::scanCallback, this);

    cubeA_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/cubeA_pose",1);
    cubeB_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/cubeB_pose",1);
    scan_aug_pub_ = n_.advertise<sensor_msgs::PointCloud>("/scan_augment",1);
    zone_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/zone",1);

    //marker_pub_ = n.advertise<visualization_msgs::Marker>("planes", 10);

    initMarker();
}

void SP::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
	

	sensor_msgs::PointCloud cloud;
  	projector_.transformLaserScanToPointCloud("laser",*scan_msg, cloud, listener_);

  	if(!receive_scan_){
		ROS_INFO("Receiving scan!!!");
		receive_scan_ = true;
		toCVS(cloud);
	}


	vector<Eigen::Vector2d> CubeCenter = findCube(cloud.points);

	pose_.pose.position.z = 0;
	if(cube_initiation){
		//ROS_INFO("Angle A:%f", cubesAngles[0]*180.0/M_PI);
		//ROS_INFO("Angle B:%f", cubesAngles[1]*180.0/M_PI);
	    pose_.pose.position.x = cubes[0][0];
	    pose_.pose.position.y = cubes[0][1];
	    
	    //Convert from angle to Quaterion
	    Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(cubesAngles[0], Eigen::Vector3d::UnitZ()));

        pose_.pose.orientation.x = orientation.x();
        pose_.pose.orientation.y = orientation.y();
        pose_.pose.orientation.z = orientation.z();
        pose_.pose.orientation.w = orientation.w();

		cubeA_pub_.publish(pose_);

		//Samething for cube B
	    pose_.pose.position.x = cubes[1][0];
	    pose_.pose.position.y = cubes[1][1];

	   orientation = Eigen::Quaterniond(Eigen::AngleAxisd(cubesAngles[1], Eigen::Vector3d::UnitZ()));

        pose_.pose.orientation.x = orientation.x();
        pose_.pose.orientation.y = orientation.y();
        pose_.pose.orientation.z = orientation.z();
        pose_.pose.orientation.w = orientation.w();

		cubeB_pub_.publish(pose_);

		if(missing_association[0] + missing_association[1] > 0)
			ROS_INFO("MIA A%i B%i", missing_association[0], missing_association[1]);
	}

	scan_aug_pub_.publish(cloud);

	geometry_msgs::PolygonStamped zone;
	geometry_msgs::Point32 p;
	p.y = left_wall;
	p.x = front_wall;
	zone.polygon.points.push_back(p);
	p.y = right_wall;
	zone.polygon.points.push_back(p);
	p.x = back_wall;
	zone.polygon.points.push_back(p);
	p.y = left_wall;
	zone.polygon.points.push_back(p);

	zone.header.frame_id = "/laser";

	zone_pub_.publish(zone);
   /* geometry_msgs::Point p1, p2;
	p1.x = cloud.points[0].x;
	p1.y = cloud.points[0].y;
	p1.z = cloud.points[0].z;

	p2.x = cloud.points[30].x;
	p2.y = cloud.points[30].y;
	p2.z = cloud.points[30].z;

	planes_.points.clear();
	planes_.points.push_back(p1);
	planes_.points.push_back(p2);
    marker_pub_.publish(planes_);*/

}

vector<Eigen::Vector2d> SP::findCube(vector<geometry_msgs::Point32> &p_data){
	// === FILTERING ======
	// First we remove the walls
	vector<Eigen::Vector2d> data;
	vector<geometry_msgs::Point32> pointss;
	for(int i = 0; i < p_data.size(); i++){
		if(!((p_data[i].x > front_wall) || (p_data[i].x < back_wall) || (p_data[i].y > left_wall) || (p_data[i].y < right_wall))
			&& !((p_data[i].x > front_room_delim) && (p_data[i].y < left_room_delim))){
			data.push_back(Eigen::Vector2d(p_data[i].x, p_data[i].y));
			pointss.push_back(p_data[i]);
		}
	}
	p_data = pointss;
	if(data.size() == 0){
		ROS_INFO("No dot found!!!");
		vector<Eigen::Vector2d> c;
		return c;
	}

	// Group into clusters. Use min distance between points to threshold a new
	// cluster. We assume that the point cloud is ordered based on scan angle.
	//int iCluster = 0;

	//vector< vector<Eigen::Vector2d> > ClusterMembers(1, vector<Eigen::Vector2d>(data[0]));
	vector< vector<Eigen::Vector2d> > ClusterMembers;
	vector<Eigen::Vector2d> firstCluster;
	firstCluster.push_back(data[0]);// first point is member of first cluster

	ClusterMembers.push_back(firstCluster);


	for(int i = 1; i < data.size(); i++){
	    if ((ClusterMembers.back().back() - data[i]).norm() > cluster_distance_threshold){
	        // Trigger a new cluster member
	        //iCluster++;
			vector<Eigen::Vector2d> emptyCluster;
			emptyCluster.push_back(data[i]);
	        ClusterMembers.push_back(emptyCluster);
	    }
	    else{
	        // add it to current cluster
	        ClusterMembers.back().push_back(data[i]);
	    }
	}


	// Now prune the clusters based on a number of conditions.
	vector< vector<Eigen::Vector2d> > KeptCluster;
	for( int i = 0; i < ClusterMembers.size(); i++){
	    // Flush the cluster if: -the number of members is insufficient
	    //                       -the distance between the first and last point is too short
	    if(ClusterMembers[i].size() >= min_cluster_size && 
	        (ClusterMembers[i][0]- ClusterMembers[i].back()).norm() > min_line_length &&
	        (ClusterMembers[i][0]- ClusterMembers[i].back()).norm() < max_line_length ){
	        // we keep current cluster, and create a new one
	        KeptCluster.push_back(ClusterMembers[i]);
	    }
	}
	
	// Perform the split and merge algorithm
	double dist;
	int SplitIndex;
	vector<Eigen::Vector2d> CubeCenter;
	Eigen::Vector2d zeroAngularVector(1,0);
	Eigen::Vector2d Center1, Center2;

	double lengthLeft, lengthRight;

	std::vector<double> anglesCluster;
	double angle;
	for(int iCluster = 0;  iCluster < KeptCluster.size(); iCluster++){
		//ROS_INFO("a");
	    SplitIndex = SplitAndMerge(KeptCluster[iCluster], dist);
		//ROS_INFO("b%f",dist);
	    if (dist > split_and_merge_threshold){
	        // we have two lines. We will be lazy and pick the first and last
	        // point to figure out the line segment
	        lengthLeft = (KeptCluster[iCluster][0] - KeptCluster[iCluster][SplitIndex]).norm();
	        lengthRight = (KeptCluster[iCluster][SplitIndex] - KeptCluster[iCluster].back()).norm();
	        if( lengthLeft > 1.5 && lengthRight > 1.5){
	        	//Center part
		        Center1 = ComputeCubeCenter(KeptCluster[iCluster][0],KeptCluster[iCluster][SplitIndex]);
		        Center2 = ComputeCubeCenter(KeptCluster[iCluster][SplitIndex],KeptCluster[iCluster].back());

		        CubeCenter.push_back(0.5*(Center1+Center2));
		        //Angle part
		        //We take the longuest side to plot the angle
		        //if(lengthLeft > lengthRight)
		        	//angle = 
		        //anglesCluster.push_back(angle);
		        if(SplitIndex > KeptCluster[iCluster].size() - SplitIndex)
		        	anglesCluster.push_back(M_PI/2.0 - acos((KeptCluster[iCluster][0] - KeptCluster[iCluster][SplitIndex]).dot(zeroAngularVector)/lengthLeft));
		        else
		        	anglesCluster.push_back(M_PI/2.0 - acos((KeptCluster[iCluster][SplitIndex] - KeptCluster[iCluster].back()).dot(zeroAngularVector)/lengthRight));
		        //ROS_INFO("Angle:%f", anglesCluster.back());
	        }
	        else if(lengthLeft > 1.5){
		        CubeCenter.push_back(ComputeCubeCenter(KeptCluster[iCluster][0],KeptCluster[iCluster][SplitIndex]));
		        anglesCluster.push_back(M_PI/2.0 - acos((KeptCluster[iCluster][0] - KeptCluster[iCluster][SplitIndex]).dot(zeroAngularVector)/lengthLeft));
		        //ROS_INFO("Angle:%f", anglesCluster.back());
	        }
	        else if(lengthRight > 1.5){
		        CubeCenter.push_back(ComputeCubeCenter(KeptCluster[iCluster][SplitIndex],KeptCluster[iCluster].back()));
		        anglesCluster.push_back(M_PI/2.0 - acos((KeptCluster[iCluster][SplitIndex] - KeptCluster[iCluster].back()).dot(zeroAngularVector)/lengthRight));
		       // ROS_INFO("Angle:%f", anglesCluster.back());
	        }
	    }
	    else{
	        // we have one line
	        CubeCenter.push_back(ComputeCubeCenter(KeptCluster[iCluster][0], KeptCluster[iCluster].back()));
	        anglesCluster.push_back(M_PI/2.0 - acos((KeptCluster[iCluster][0] - KeptCluster[iCluster].back()).dot(zeroAngularVector)/(KeptCluster[iCluster][0] - KeptCluster[iCluster].back()).norm()));
		    //ROS_INFO("Angle:%f", anglesCluster.back());
	    }
	}

	// if the cube are not already initiated
	if(!cube_initiation && CubeCenter.size() >= 2){
		cubes.push_back(CubeCenter[0]);
		cubes.push_back(CubeCenter[1]);
		missing_association.push_back(0);
		missing_association.push_back(0);


		cubesAngles.push_back(anglesCluster[0]);
		cubesAngles.push_back(anglesCluster[1]);
		cube_initiation = true;
	}
	else if(cube_initiation ){
		for(int index = 0; index < 2; index++){
			if(CubeCenter.size() >= 1){
				int nearest_id = -1;
				//double near
				for(int i = 0; i < CubeCenter.size(); i++){
					if((nearest_id == -1) || // Take first element if not initiated
					   (CubeCenter[i] - cubes[index]).norm() <= (CubeCenter[nearest_id] - cubes[index]).norm()){
						nearest_id = i;
					}
				}
				if((CubeCenter[nearest_id] - cubes[index]).norm() < max_translation){

					//We assign the vector to the cube and remove it from the stack
					missing_association[index] = 0;
					cubes[index] = (CubeCenter[nearest_id] - cubes[index])/2 + cubes[index];
					cubesAngles[index] = smallestAngle(cubesAngles[index], anglesCluster[nearest_id]);

					// We erase from the stack that cluster
					CubeCenter.erase(CubeCenter.begin() + nearest_id);
					anglesCluster.erase(anglesCluster.begin() + nearest_id);
				}
				else{
					missing_association[index]++;
				}
			}
			else{
				missing_association[index]++;
			}
		}
		// if there is still dots
		if(CubeCenter.size() > 0){
			for(int index = 0; index < 2; index++){
				if(missing_association[index] > max_tick_ghost){

					int nearest_id = -1;
					for(int i = 0; i < CubeCenter.size(); i++){
						if((nearest_id == -1) || // Take first element if not initiated
						   (CubeCenter[i] - cubes[index]).norm() <= (CubeCenter[nearest_id] - cubes[index]).norm()){
							nearest_id = i;
						}
					}
					if(nearest_id != -1){
						cubes[index] = CubeCenter[nearest_id];
						cubesAngles[index] = smallestAngle(cubesAngles[index], anglesCluster[nearest_id]);

						CubeCenter.erase(CubeCenter.begin() + nearest_id);
						anglesCluster.erase(anglesCluster.begin() + nearest_id);
						missing_association[index] = 0;
					}
				}
			}

		}
	}

	return CubeCenter;
}

double wrap(double x){
	return x-2*M_PI*floor(x/(2*M_PI)+0.5);
}
double SP::smallestAngle(double old, double next){
	double a;
	double best = -1;
	for(int i = 0; i < 4; i++){
		a = next + i * M_PI/2.0; // a ; a+ 90; a+ 180 ...
		if(best == -1 || abs(wrap(a - old)) < abs(wrap(best - old))){
			best = a;
		}
	}

	return wrap(wrap(best - old)*0.5+ old);
}

int SP::SplitAndMerge(vector<Eigen::Vector2d> data, double & dist){
	// Perform the split and merge algorithm
    int split = -1;

    Eigen::Vector3d Q1(data[0][0], 	   data[0][1], 	   0);
    Eigen::Vector3d Q2(data.back()[0], data.back()[1], 0);
    dist = 0;

    double temp_dist;
    for(int i = 0; i < data.size(); i++){
        Eigen::Vector3d P(data[i][0], data[i][1], 0);

        temp_dist = (Q2-Q1).cross(P-Q1).norm()/((Q2-Q1).norm());
       	if(temp_dist > dist){
       		dist = temp_dist;
       		split = i;
       	}
    }

    return split;
    
}

Eigen::Vector2d SP::ComputeCubeCenter(Eigen::Vector2d Q1, Eigen::Vector2d Q2){
    Eigen::Vector2d b = Q2-Q1;
    b.normalize();
    Eigen::Vector2d midpoint = 0.5*(Q2+Q1);

    Eigen::Matrix2d m;
    m << 0, -1,
    	 1, 0;
    /*m(0, 0) = 0;
    m(0, 1) = -1;
    m(1, 0) = 1;
    m(1, 1) = 0;*/

    // 2.25 is the length of cube
    Eigen::Vector2d Center1 = 0.5 * 2.25 * m * b  + midpoint; // center on one side of line
    Eigen::Vector2d Center2 = 0.5 * 2.25 * m * (-b) + midpoint; // center on other side of line

    // we have to pick the center farthest away from (0,0)
    if (Center1.norm() > Center2.norm())
        return Center1;
    else
        return Center2;
}



void SP::initMarker(){
    planes_.header.frame_id = pose_.header.frame_id = "laser";

    planes_.action = visualization_msgs::Marker::ADD;
    planes_.pose.orientation.w = 1.0;

    planes_.type = visualization_msgs::Marker::LINE_STRIP;
    planes_.scale.x = 0.1;

    planes_.color.b = 1;
    planes_.color.a = 1;
}

void SP::toCVS(sensor_msgs::PointCloud &cloud){
	string filename;
    ros::param::get("~path", filename);
    if(filename.compare("")){

	    ofstream myfile;
	  	myfile.open (filename.c_str());

		for(int i =0;i<cloud.points.size();i++){
	  		myfile<<cloud.points[i].x<<"," <<cloud.points[i].y<< "\n";

	    }
	  	myfile.close();

		ROS_INFO("File saved!!!");

    }
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void SP::dynamicParametersCallback(sick_pose::sickPoseConfig &config, uint32_t level){
	front_wall = config.front_wall;
	back_wall = config.back_wall;
	left_wall = config.left_wall;
	right_wall = config.right_wall;

	front_room_delim = config.front_room_delim;
	left_room_delim = config.left_room_delim;

	cluster_distance_threshold = config.cluster_distance_threshold;
	split_and_merge_threshold = config.split_and_merge_threshold;
	min_cluster_size = config.min_cluster_size;
	max_line_length = config.max_line_length;
	min_line_length = config.min_line_length;
	max_tick_ghost = config.max_tick_ghost;
	max_translation = config.max_translation;

	ROS_INFO("Parameters changed");
  
}


} // namespace sick_pose


int main(int argc, char* argv[]){

    
	ROS_INFO("Main start...\n");
	ros::init(argc, argv,  "sick_pose_node");

	ros::NodeHandle n;
	sick_pose::SP sick_pose(n);
  	ros::spin();
}