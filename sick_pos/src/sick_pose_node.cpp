#include "sick_pose/sick_pose_node.h"

using namespace std;

namespace sick_pose
{
SP::SP(ros::NodeHandle n) :
   nh(n),
    receive_scan_(false),
    cube_initiation(false)
  {

    if(ros::param::has("~nbrCubes")){
        ros::param::get("~nbrCubes", NBRCUBES);
    }
    else{
        ROS_WARN("No \"_nbrCubes\" parameter detected, number of cube set to 1");
        NBRCUBES = 1;
    }

    ROS_INFO("Number of cube(s): %i \n", NBRCUBES);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<sick_pose::sickPoseConfig>::CallbackType cb_;
    cb_ = boost::bind(&SP::dynamicParametersCallback, this, _1, _2);
    dr_server_.setCallback(cb_);

    laser_scan_ = this->nh.subscribe("/scan", 1, &SP::scanCallback, this);

    cubeA_pub_     = this->nh.advertise<geometry_msgs::PoseStamped>("/cubeA_pose",1);
    cubeB_pub_     = this->nh.advertise<geometry_msgs::PoseStamped>("/cubeB_pose",1);
    scan_aug_pub_  = this->nh.advertise<sensor_msgs::PointCloud>("/scan_augment",1);
    zone_pub_      = this->nh.advertise<geometry_msgs::PolygonStamped>("/zone",1);
    cube_poly_pub_ = this->nh.advertise<geometry_msgs::PolygonStamped>("/cube_poly",1);

    marker_cubeA_pub_ = this->nh.advertise<visualization_msgs::Marker>("/cubeA_marker", 10);
    marker_cubeB_pub_ = this->nh.advertise<visualization_msgs::Marker>("/cubeB_marker", 10);

    initMarker();
}

void SP::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
	

	sensor_msgs::PointCloud cloud;
  	projector_.transformLaserScanToPointCloud("laser",*scan_msg, cloud, listener_);

  	if(!receive_scan_){
		ROS_INFO("Receiving scan!!!");
		receive_scan_ = true;
		//toCVS(cloud);
	}


	ListVector2d CubeCenter = findCube(cloud.points);

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

        cubeA_marker_.pose = pose_.pose;
		marker_cubeA_pub_.publish(cubeA_marker_);
		cubeA_pub_.publish(pose_);

		//Samething for cube B
	    pose_.pose.position.x = cubes[1][0];
	    pose_.pose.position.y = cubes[1][1];

	   orientation = Eigen::Quaterniond(Eigen::AngleAxisd(cubesAngles[1], Eigen::Vector3d::UnitZ()));

        pose_.pose.orientation.x = orientation.x();
        pose_.pose.orientation.y = orientation.y();
        pose_.pose.orientation.z = orientation.z();
        pose_.pose.orientation.w = orientation.w();

        cubeB_marker_.pose = pose_.pose;
		marker_cubeB_pub_.publish(cubeB_marker_);
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

ListVector2d SP::findCube(vector<geometry_msgs::Point32> &p_data){
	// === FILTERING ======
	// First we remove the walls
	ListVector2d data;
	vector<geometry_msgs::Point32> pointss;
	for(int i = 0; i < p_data.size(); i++){
		if(!((p_data[i].x > front_wall) || (p_data[i].x < back_wall) || (p_data[i].y > left_wall) || (p_data[i].y < right_wall))
			&& !((p_data[i].x > front_room_delim) && (p_data[i].y < left_room_delim))){
			data.push_back(Eigen::Vector2d(p_data[i].x, p_data[i].y));
			//pointss.push_back(p_data[i]);
		}
	}

	p_data.clear();
	//p_data = pointss;
	if(data.size() == 0){
		ROS_INFO("No dot found!!!");
		ListVector2d c;
		return c;
	}

	// Group into clusters. Use min distance between points to threshold a new
	// cluster. We assume that the point cloud is ordered based on scan angle.
	//int iCluster = 0;

	//vector< vector<Eigen::Vector2d> > ClusterMembers(1, vector<Eigen::Vector2d>(data[0]));
	vector< ListVector2d > ClusterMembers;
	ListVector2d firstCluster;
	firstCluster.push_back(data[0]);// first point is member of first cluster

	ClusterMembers.push_back(firstCluster);


	for(int i = 1; i < data.size(); i++){
	    if ((ClusterMembers.back().back() - data[i]).norm() > cluster_distance_threshold){
	        // Trigger a new cluster member
	        //iCluster++;
			ListVector2d emptyCluster;
			emptyCluster.push_back(data[i]);
	        ClusterMembers.push_back(emptyCluster);
	    }
	    else{
	        // add it to current cluster
	        ClusterMembers.back().push_back(data[i]);
	    }
	}


	// Now prune the clusters based on a number of conditions.
	vector< ListVector2d > KeptCluster;
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


	double dist;
	int SplitIndex;
	ListVector2d CubeCenter;
	//Eigen::Vector2d zeroAngularVector(1,0);
	Eigen::Vector2d p1, p2;

	double lengthLeft, lengthRight;
	zone.polygon.points.clear();
	std::vector<double> anglesCluster;
	//double angle;
	for(int iCluster = 0;  iCluster < KeptCluster.size(); iCluster++){
		//ROS_INFO("a");
	   // SplitIndex = SplitAndMerge(KeptCluster[iCluster], dist);
		int begin = 0;
		int end = KeptCluster[iCluster].size() - 1;
		SplitIndex = SplitAndMerge(KeptCluster[iCluster],
								   begin,
								   end,
								   dist);
		/*ListVector2d::iterator beginIt = KeptCluster[iCluster].begin();
		ListVector2d::iterator endIt = KeptCluster[iCluster].end();
		SplitIndex = SplitAndMerge(beginIt, endIt, dist);*/

	    if (dist > split_and_merge_threshold){
	        // we have two lines. We will be lazy and pick the first and last
			// point to figure out the line segment
	        lengthLeft = (KeptCluster[iCluster][0] - KeptCluster[iCluster][SplitIndex]).norm();
			lengthRight = (KeptCluster[iCluster][SplitIndex] - KeptCluster[iCluster].back()).norm();

			if(lengthLeft > 1.5 && lengthRight > 1.5){

				// This resolve some issue with where a person was detected has part of the cube
				// By removing the points made by the person from the cube, we can find the correct
				// center of the cube
				double distZeroToCorner, distCornerToEnd;
				//ListVector2d::iterator splitPlusOneIt = beginIt + SplitIndex + 1;
				//ListVector2d::iterator splitIt = beginIt + SplitIndex;
				int SplitIndexZeroToCorner = SplitAndMerge(KeptCluster[iCluster],
														   0,
														   SplitIndex,
														   distZeroToCorner);
				int SplitIndexCornerToEnd = SplitAndMerge(KeptCluster[iCluster],
														  SplitIndex,
														  end,
														  distCornerToEnd);

				if(distZeroToCorner > edge_split_and_merge_threshold){
					ROS_INFO("Person detected on the down side!!!");
					begin = SplitIndexZeroToCorner;
				}
				if(distCornerToEnd > edge_split_and_merge_threshold){
					ROS_INFO("Person detected on the upper side!!!");
					end = SplitIndexCornerToEnd;
				}

				ComputeCubeCenterWithLine(KeptCluster[iCluster], begin, SplitIndex, p1, p2, p_data);
				Eigen::Vector2d centerA = ComputeCubeCenter(p1, p2);
				ComputeCubeCenterWithLine(KeptCluster[iCluster], SplitIndex, end, p1, p2, p_data);
				CubeCenter.push_back((ComputeCubeCenter(p1, p2) - centerA) * 0.5 + centerA);
	        }
			else if(lengthLeft > 1.5 && SplitIndex > 13){
		        ComputeCubeCenterWithLine(KeptCluster[iCluster], 0, SplitIndex, p1, p2, p_data);
				CubeCenter.push_back(ComputeCubeCenter(p1, p2));
	        }
	        //else if(lengthRight > 1.5 && KeptCluster[iCluster].size() - SplitIndex > 13){
			else{
		        ComputeCubeCenterWithLine(KeptCluster[iCluster], SplitIndex, KeptCluster[iCluster].size() - 1, p1, p2, p_data);
				CubeCenter.push_back(ComputeCubeCenter(p1, p2));
			}
	    }
	    else{
	        // we have one line
			//ROS_INFO("We have one line");
		     ComputeCubeCenterWithLine(KeptCluster[iCluster], 0, KeptCluster[iCluster].size() - 1, p1, p2, p_data);
	    	CubeCenter.push_back(ComputeCubeCenter(p1, p2));
	    }

	    //CubeCenter.push_back(ComputeCubeCenter(p1, p2));

	   // anglesCluster.push_back((p1 - p2)[1]/abs((p1 - p2)[1]) * acos((p1 - p2)[0]/(p1 - p2).norm()));
	    anglesCluster.push_back(atan2((p1 - p2)[1], (p1 - p2)[0]));
	}

	// if the cube are not already initiated
	if(!cube_initiation && CubeCenter.size() >= NBRCUBES){
		for(int i = 0; i < NBRCUBES; i++){
			cubes.push_back(CubeCenter[i]);
			missing_association.push_back(0);
			cubesAngles.push_back(anglesCluster[i]);
		}
		cube_initiation = true;
	}
	else if(cube_initiation ){
		for(int index = 0; index < NBRCUBES; index++){
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
					cubes[index] = (CubeCenter[nearest_id] - cubes[index]) * 0.5 + cubes[index];
					cubesAngles[index] = smallestAngle(cubesAngles[index], anglesCluster[nearest_id]);

					// We erase from the stack this center
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
			for(int index = 0; index < NBRCUBES; index++){
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

	zone.header.frame_id = "/laser";
	cube_poly_pub_.publish(zone);
	return CubeCenter;
}

double wrap(double x){
	return x-2*M_PI*floor(x/(2*M_PI)+0.5);
}
double SP::smallestAngle(double old, double next){
	double a;
	double best = -1;
	//ROS_INFO("Angle old:%f", old * 180.0 / M_PI);
	for(int i = 0; i < 4; i++){
		a = next + i * M_PI * 0.5; // a ; a + 90; a + 180 ...
		//ROS_INFO("%f", a * 180.0 / M_PI);
		if(best == -1 || abs(wrap(a - old)) < abs(wrap(best - old))){
			best = a;
		}
	}
	//ROS_INFO("Best:%f / %f", best * 180.0 / M_PI, wrap(best) * 180.0 / M_PI);
	//ROS_INFO("Best correction:%f", wrap(wrap(best - old)*0.5+ old) * 180.0 / M_PI);

	return wrap(wrap(best - old)*0.5+ old);
	//return best;
}

/*int SP::SplitAndMerge(ListVector2d::iterator begin,
					  ListVector2d::iterator end,
					  double & dist){*/
int SP::SplitAndMerge(ListVector2d data, int begin, int end, double & dist){
	// Perform the split and merge algorithm
    int split = -1;

    Eigen::Vector3d Q1(data[begin][0], data[begin][1], 0);
    Eigen::Vector3d Q2(data[end][0],   data[end][1],   0);
    dist = 0;

    double temp_dist;
    for(int i = begin; i <= end; i++){
        Eigen::Vector3d P(data[i][0], data[i][1], 0);

        temp_dist = (Q2-Q1).cross(P-Q1).norm()/((Q2-Q1).norm());
       	if(temp_dist > dist){
       		dist = temp_dist;
       		split = i;
        }
    }

    return split;
    
}

void SP::ComputeCubeCenterWithLine(ListVector2d pointsInLine, int begin, int end, Eigen::Vector2d & p1,  Eigen::Vector2d & p2, vector<geometry_msgs::Point32> &p_data){
	//Eigen::linearRegression::linearRegression(end, &(pointsInLine[0]), &line_coeffs, 0);
	// ===== linear Regression ==== 
	// explication => http://onlinestatbook.com/2/regression/intro.html
	//begin += 2;
	//end -= 2;
	geometry_msgs::Point32 dots;
	dots.x = pointsInLine[begin][0];
	dots.y = pointsInLine[begin][1];
	p_data.push_back(dots);
	dots.x = pointsInLine[end][0];
	dots.y = pointsInLine[end][1];
	p_data.push_back(dots);

	vector<Eigen::Vector2d> ptsLine;
	bool inverseXY = false;
	// we inverse the x/y when the line is almost parrallele to the x's axe
	Eigen::Vector2d delimeter = pointsInLine[end] - pointsInLine[begin];
	if( atan(abs(delimeter[1]/delimeter[0])) > M_PI * 0.25 ){
		inverseXY = true;
		for(int i = begin; i < end; i++){
			ptsLine.push_back(pointsInLine[i]);
			ptsLine.back()[0] = pointsInLine[i][1];
			ptsLine.back()[1] = pointsInLine[i][0];
		}
	}
	else{
		for(int i = begin; i < end; i++){
			ptsLine.push_back(pointsInLine[i]);
		}
	}

	// Line smoothing
	/*
	for(int i = 1; i < ptsLine.size() - 1; i++){
		// If the distance between the dot before and after is too short, we erase it
		if((ptsLine[i-1] - ptsLine[i+1]).norm() < 0.1){
			ptsLine.erase(ptsLine.begin() + i);
			i--;
		}
		else{
			dots.x = ptsLine[i][0];
			dots.y = ptsLine[i][1];
			p_data.push_back(dots);
		}

	}*/

	// Mean:
	Eigen::Vector2d mean(0, 0);
	for(int i = 0; i < ptsLine.size(); i++){
		mean += ptsLine[i];
	}
	mean = mean / ptsLine.size();
	//cout<<"Mean:\n" << mean << endl;

	// "r" correlation:
	double xy = 0;
	double x2 = 0;
	double y2 = 0;
	for(int i = 0; i < ptsLine.size(); i++){
		xy += (ptsLine[i][0] - mean[0]) * (ptsLine[i][1]- mean[1]);
		x2 += (ptsLine[i][0] - mean[0]) * (ptsLine[i][0]- mean[0]);
		y2 += (ptsLine[i][1] - mean[1]) * (ptsLine[i][1]- mean[1]);
	}
	double r = xy / sqrt(x2 * y2);

	//cout<<"r:\n"<<r<<endl;

	// Standard deviation:
	Eigen::Vector2d s(0, 0);

	for(int i = 0; i < ptsLine.size(); i++){
		s[0] += (ptsLine[i][0]- mean[0]) * (ptsLine[i][0]- mean[0]);
		s[1] += (ptsLine[i][1]- mean[1]) * (ptsLine[i][1]- mean[1]);
	}
	s = s/ptsLine.size();
	s[0] = sqrt(s[0]);
	s[1] = sqrt(s[1]);

	//cout<<"standardDev:\n"<<s<<endl;

	double b = r * s[1] / s[0];
	double a = mean[1] - b * mean[0];

	//Eigen::Vector2d line_coeffs( a, b);
	//cout<<"a: "<<a<<"\nb: "<<b<<endl;

	
	Eigen::Vector2d lineV(1/sqrt(1+b*b), b/sqrt(1+b*b));
	Eigen::Vector2d onTheLine(0,a);
	geometry_msgs::Point32 pA, pB;

	// We project the point on a line
	p1 = (ptsLine[0] - onTheLine).dot(lineV)/lineV.dot(lineV) * lineV + onTheLine;
	p2 = (ptsLine.back()   - onTheLine).dot(lineV)/lineV.dot(lineV) * lineV + onTheLine;
	
	if(inverseXY){
		Eigen::Vector2d tampon;
		tampon[0] = p1[1];
		tampon[1] = p1[0];
		p1 = tampon;
		tampon[0] = p2[1];
		tampon[1] = p2[0];
		p2 = tampon;
	}
	// Rviz visualization
	pA.x = p1[0];
	pA.y = p1[1];
	pB.x = p2[0];
	pB.y = p2[1];
	zone.polygon.points.push_back(pA);
	zone.polygon.points.push_back(pB);

	//return ComputeCubeCenter(vA, vB);
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
    cubeA_marker_.header.frame_id = pose_.header.frame_id = "/laser";

    cubeA_marker_.action = visualization_msgs::Marker::ADD;

    cubeA_marker_.type = visualization_msgs::Marker::CUBE;
    cubeA_marker_.scale.x = 2.25;
    cubeA_marker_.scale.y = 2.25;
    cubeA_marker_.scale.z = 2.25;

    cubeA_marker_.color.b = 1;
    cubeA_marker_.color.a = 0.5;

    cubeB_marker_.header.frame_id = pose_.header.frame_id = "/laser";

    cubeB_marker_.action = visualization_msgs::Marker::ADD;

    cubeB_marker_.type = visualization_msgs::Marker::CUBE;
    cubeB_marker_.scale.x = 2.25;
    cubeB_marker_.scale.y = 2.25;
    cubeB_marker_.scale.z = 2.25;

    cubeB_marker_.color.r = 1;
    cubeB_marker_.color.a = 0.5;
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
	edge_split_and_merge_threshold = config.edge_split_and_merge_threshold;
	min_cluster_size = config.min_cluster_size;
	max_line_length = config.max_line_length;
	min_line_length = config.min_line_length;
	max_tick_ghost = config.max_tick_ghost;
	max_translation = config.max_translation;

	ROS_INFO("Parameters changed");
  
}


} // namespace sick_pose


int main(int argc, char* argv[]){

    
    ROS_INFO("Main start...");
	ros::init(argc, argv,  "sick_pose_node");

	ros::NodeHandle n;
	sick_pose::SP sick_pose(n);
  	ros::spin();
}
