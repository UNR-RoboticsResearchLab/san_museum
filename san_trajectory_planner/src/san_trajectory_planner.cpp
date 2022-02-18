/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <san_trajectory_planner/san_trajectory_planner.h>
#include <san_trajectory_planner/san_trajectory_planner_ros.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/Point32.h>

#include <san_trajectory_planner/trajectoryPointService.h>
#include <san_trajectory_planner/point_grid.h>
#include <san_feature_extractor/Trajectory.h>

#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//added for PaCcET features
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

//tf stuff
#include <tf/transform_broadcaster.h>

//for computing path distance
#include <queue>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

using namespace std;
using namespace costmap_2d;

//must be the same value in PaCcET.h
#define NUM_OBJECTIVES 6

namespace san_trajectory_planner {

void TrajectoryPlanner::reconfigure(base_local_planner::BaseLocalPlannerConfig &cfg)
{
	base_local_planner::BaseLocalPlannerConfig config(cfg);

	boost::mutex::scoped_lock l(configuration_mutex_);

	acc_lim_x_ = config.acc_lim_x;
	acc_lim_y_ = config.acc_lim_y;
	acc_lim_theta_ = config.acc_lim_theta;

	max_vel_x_ = config.max_vel_x;
	min_vel_x_ = config.min_vel_x;

	max_vel_th_ = config.max_vel_theta;
	min_vel_th_ = config.min_vel_theta;
	min_in_place_vel_th_ = config.min_in_place_vel_theta;

	sim_time_ = config.sim_time;
	sim_granularity_ = config.sim_granularity;
	angular_sim_granularity_ = config.angular_sim_granularity;

	path_distance_bias_ = config.path_distance_bias;
	goal_distance_bias_ = config.goal_distance_bias;
	occdist_scale_ = config.occdist_scale;

	if (meter_scoring_) {
		//if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
		double resolution = costmap_.getResolution();
		goal_distance_bias_ *= resolution;
		path_distance_bias_ *= resolution;
	}

	oscillation_reset_dist_ = config.oscillation_reset_dist;
	escape_reset_dist_ = config.escape_reset_dist;
	escape_reset_theta_ = config.escape_reset_theta;

	vx_samples_ = config.vx_samples;
	vtheta_samples_ = config.vtheta_samples;

	if (vx_samples_ <= 0) {
		config.vx_samples = 1;
		vx_samples_ = config.vx_samples;
		ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
	}
	if(vtheta_samples_ <= 0) {
		config.vtheta_samples = 1;
		vtheta_samples_ = config.vtheta_samples;
		ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
	}

	heading_lookahead_ = config.heading_lookahead;

	holonomic_robot_ = config.holonomic_robot;

	backup_vel_ = config.escape_vel;

	dwa_ = config.dwa;

	heading_scoring_ = config.heading_scoring;
	heading_scoring_timestep_ = config.heading_scoring_timestep;

	simple_attractor_ = config.simple_attractor;

	//y-vels
	string y_string = config.y_vels;
	vector<string> y_strs;
	boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

	vector<double>y_vels;
	for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
		istringstream iss(*it);
		double temp;
		iss >> temp;
		y_vels.push_back(temp);
		//ROS_INFO("Adding y_vel: %e", temp);
	}

	y_vels_ = y_vels;

}

geometry_msgs::PoseStamped globalPose1;
geometry_msgs::PoseStamped globalPose2;
geometry_msgs::PoseStamped globalPose3;
geometry_msgs::PoseStamped tfedCenter;
geometry_msgs::PoseStamped tfedIntendedPosition;

void robot1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	globalPose1.pose = msg->pose;
}

void robot2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	globalPose2.pose = msg->pose;
}

void robot3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	globalPose3.pose = msg->pose;
}

void centerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tfedCenter.pose = msg->pose;
}

void intentCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tfedIntendedPosition.pose = msg->pose;
}

ros::NodeHandle n;
ros::ServiceClient futureclient;
ros::Publisher futurepoints;
geometry_msgs::Point fututePoint;
ros::Subscriber sub1 = n.subscribe("/transformed_human", 1000, robot1Callback);
ros::Subscriber sub2 = n.subscribe("/transformed_human2", 1000, robot2Callback);
ros::Subscriber sub3 = n.subscribe("/transformed_human3", 1000, robot3Callback);
ros::Subscriber sub4 = n.subscribe("/transformed_center", 1000, centerCallback);
ros::Subscriber sub5 = n.subscribe("/transformed_intended_position", 1000, intentCallback);

TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
                                     const Costmap2D& costmap,
                                     std::vector<geometry_msgs::Point> footprint_spec,
                                     double acc_lim_x, double acc_lim_y, double acc_lim_theta,
                                     double sim_time, double sim_granularity,
                                     int vx_samples, int vtheta_samples,
                                     double path_distance_bias, double goal_distance_bias, double occdist_scale,
                                     double heading_lookahead, double oscillation_reset_dist,
                                     double escape_reset_dist, double escape_reset_theta,
                                     bool holonomic_robot,
                                     double max_vel_x, double min_vel_x,
                                     double max_vel_th, double min_vel_th, double min_in_place_vel_th,
                                     double backup_vel,
                                     bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
                                     vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
	: path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
	goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
	costmap_(costmap),
	world_model_(world_model), footprint_spec_(footprint_spec),
	sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
	vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
	path_distance_bias_(path_distance_bias), goal_distance_bias_(goal_distance_bias), occdist_scale_(occdist_scale),
	acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
	prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
	oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
	escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
	max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
	max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
	backup_vel_(backup_vel),
	dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
	simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
{
	//the robot is not stuck to begin with
	stuck_left = false;
	stuck_right = false;
	stuck_left_strafe = false;
	stuck_right_strafe = false;
	rotating_left = false;
	rotating_right = false;
	strafe_left = false;
	strafe_right = false;

	escaping_ = false;
	final_goal_position_valid_ = false;

	// advertise topics
	futurepoints = n.advertise<geometry_msgs::Point>("future_points",1000);
	futureclient = n.serviceClient<san_feature_extractor::Trajectory>("future_trajectory_points");

	costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
}

TrajectoryPlanner::~TrajectoryPlanner(){
}

bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
	MapCell cell = path_map_(cx, cy);
	MapCell goal_cell = goal_map_(cx, cy);
	if (cell.within_robot) {
		return false;
	}
	occ_cost = costmap_.getCost(cx, cy);
	if (cell.target_dist == path_map_.obstacleCosts() ||
	    cell.target_dist == path_map_.unreachableCellCosts() ||
	    occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
		return false;
	}
	path_cost = cell.target_dist;
	goal_cost = goal_cell.target_dist;
	total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ * goal_cost + occdist_scale_ * occ_cost;
	return true;
}

/**
 * create and score a trajectory given the current pose of the robot and selected velocities
 */
void TrajectoryPlanner::generateTrajectory(
	double x, double y, double theta,
	double vx, double vy, double vtheta,
	double vx_samp, double vy_samp, double vtheta_samp,
	double acc_x, double acc_y, double acc_theta,
	double impossible_cost,
	Trajectory& traj) {

	// make sure the configuration doesn't change mid run
	boost::mutex::scoped_lock l(configuration_mutex_);

	double x_i = x;
	double y_i = y;
	double theta_i = theta;

	double vx_i, vy_i, vtheta_i;

	vx_i = vx;
	vy_i = vy;
	vtheta_i = vtheta;

	//compute the magnitude of the velocities
	double vmag = hypot(vx_samp, vy_samp);

	//compute the number of steps we must take along this trajectory to be "safe"
	int num_steps;
	if(!heading_scoring_) {
		num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
	} else {
		num_steps = int(sim_time_ / sim_granularity_ + 0.5);
	}

	//we at least want to take one step... even if we won't move, we want to score our current position
	if(num_steps == 0) {
		num_steps = 1;
	}

	double dt = sim_time_ / num_steps;
	double time = 0.0;

	//create a potential trajectory
	traj.resetPoints();
	traj.xv_ = vx_samp;
	traj.yv_ = vy_samp;
	traj.thetav_ = vtheta_samp;
	traj.cost_ = -1.0;

	//initialize the costs for the trajectory
	double path_dist = 0.0;
	double goal_dist = 0.0;
	double occ_cost = 0.0;
	double heading_diff = 0.0;

	for(int i = 0; i < num_steps; ++i) {
		//get map coordinates of a point
		unsigned int cell_x, cell_y;

		//we don't want a path that goes off the know map
		if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
			traj.cost_ = -1.0;
			return;
		}

		//check the point on the trajectory for legality
		double footprint_cost = footprintCost(x_i, y_i, theta_i);

		//if the footprint hits an obstacle this trajectory is invalid
		if(footprint_cost < 0) {
			traj.cost_ = -1.0;
			return;
			//TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
			//it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
			//come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
			//but safe.
			/*
			   double max_vel_x, max_vel_y, max_vel_th;
			   //we want to compute the max allowable speeds to be able to stop
			   //to be safe... we'll make sure we can stop some time before we actually hit
			   getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

			   //check if we can stop in time
			   if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
			   ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
			   //if we can stop... we'll just break out of the loop here.. no point in checking future points
			   break;
			   }
			   else{
			   traj.cost_ = -1.0;
			   return;
			   }
			 */
		}

		occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

		//do we want to follow blindly
		if (simple_attractor_) {
			goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
			            (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
			            (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
			            (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
		} else {

			bool update_path_and_goal_distances = true;

			// with heading scoring, we take into account heading diff, and also only score
			// path and goal distance for one point of the trajectory
			if (heading_scoring_) {
				if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
					heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
				} else {
					update_path_and_goal_distances = false;
				}
			}

			if (update_path_and_goal_distances) {
				//update path and goal distances
				path_dist = path_map_(cell_x, cell_y).target_dist;
				goal_dist = goal_map_(cell_x, cell_y).target_dist;

				//if a point on this trajectory has no clear path to goal it is invalid
				if(impossible_cost <= goal_dist || impossible_cost <= path_dist) {
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
					traj.cost_ = -2.0;
					return;
				}
			}
		}


		//the point is legal... add it to the trajectory
		traj.addPoint(x_i, y_i, theta_i);

		//calculate velocities
		vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
		vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
		vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

		//calculate positions
		x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
		y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
		theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

		//increment time
		time += dt;
	} // end for i < numsteps

	//Interpersonal distance
	geometry_msgs::PoseStamped localPose1;
	localPose1 = globalPose1;
	geometry_msgs::PoseStamped localPose2 = globalPose2;
	geometry_msgs::PoseStamped localPose3 = globalPose3;
	geometry_msgs::PoseStamped localCenter = tfedCenter;
	geometry_msgs::PoseStamped localIntendedPosition = tfedIntendedPosition;

	traj.inter_dist.clear();

	traj.inter_dist.push_back(sqrt(pow((localPose1.pose.position.x - x_i), 2) + pow((localPose1.pose.position.y - y_i), 2)));
	traj.inter_dist.push_back(sqrt(pow((localPose2.pose.position.x - x_i), 2) + pow((localPose2.pose.position.y - y_i), 2)));
	traj.inter_dist.push_back(sqrt(pow((localPose3.pose.position.x - x_i), 2) + pow((localPose3.pose.position.y - y_i), 2)));

	//Distance from center of the O formation calculation
	traj.d_center = sqrt(pow((localCenter.pose.position.x - x_i), 2) + pow((localCenter.pose.position.y - y_i), 2));
	//Distance from future point to O formation's intended posotion calculation
	traj.d_intent = sqrt(pow((localIntendedPosition.pose.position.x - x_i), 2) + pow((localIntendedPosition.pose.position.y - y_i), 2));
	//cout << "Inter_dist" <<  "/t" << traj.inter_dist << endl;
	//cout << "Traj Position" << "\t" << x_i << "\t" << y_i << endl;


	// //Not used for PaCcET. This could be included if we want to add this to the traditional planner
	// double inter_dist_fitness = 1000000000;
	// if (run_paccet==false){
	//   //cout << "Inter Distance" << "\t" << p_traj.inter_dist << endl;
	//   if (traj.inter_dist <= inter_dist_threshold){
	//   //if the trajectory makes the robot hit the person
	//     if (traj.inter_dist==0){
	//     cout << "Hit Person" << endl;
	//     //high fitness value should keep the robot from making this choice
	//     inter_dist_fitness = 1000000000;
	//     }
	//   //if the robot is within the ID threashold then the fitness value will increase exponentially
	//   inter_dist_fitness = exp(1/traj.inter_dist);
	//   }
	// }
	// //if the trajectory is outside of the threashold for interpersonal distance
	// else{
	//   //assert (traj.inter_dist > inter_dist_threshold);
	//   //if not in the threashold then robot will not be penalized for this movement
	//   inter_dist_fitness = 0;
	// }
	// traj.inter_dist_fitness = inter_dist_fitness;

	//ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
	double cost = -1.0;
	if (!heading_scoring_) {
		cost = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;
	} else {
		cost = occdist_scale_ * occ_cost + path_distance_bias_ * path_dist + 0.3 * heading_diff + goal_dist * goal_distance_bias_;
	}

	/**************************************************/
	// DFS: get probability for scaling parameter


	//geometry_msgs::Point32 trajectory_point;


	double scale_prob = 1.0;

	//     //Tio test if there is any future trajectory points
	// 	fututePoint.x = x_i;
	//     fututePoint.y = y_i;
	//     fututePoint.z = 0;
	//     futurepoints.publish(fututePoint);

	// 	san_feature_extractor::Trajectory srv;
	// 	srv.request.x = x_i;
	// 	srv.request.y = y_i;

	//     //ROS_INFO("Client requesting for probability. Future points (%f. %f)", x_i,y_i);

	// 	//ros::Rate loop_rate(10);

	// 	if (futureclient.call(srv))
	// 	{
	// 	  scale_prob = 1-srv.response.prob;
	//       //ROS_INFO("Probability - %f", srv.response.prob);
	// 	}
	// 	else
	// 	{
	// 	  ROS_ERROR("Failed to call service future_trajectory_points");
	// 	}

	// 	//gmm_model::LookupProb::Request req;
	// 	//gmm_model::LookupProb::Response res;

	// 	//req.robot_pos.x = x_i;
	// 	//req.robot_pos.y = y_i;
	// 	//req.robot_pos.z = 0;

	// 	/*if( !ros::service::call("/lookup_prob", req, res ) )
	// 	 {
	// 	 ROS_WARN( "lookup_prob service call failed" );
	// 	 }
	// 	 else {
	// 	 scale_prob = 1-res.prob;
	// 	 //if( scale_prob < 1e-6 ) scale_prob = 1e-6;
	// 	 }*/




	// geometry_msgs::Point32 point;
	//   point.x = x_i;
	//   point.y = y_i;
	//   point.z = 0;
	//   cloud_->points.push_back(point);
	/*************************************************/

	//**is this scale_prob going to be an issue
	traj.cost_ = cost * scale_prob;
	//std::cout << traj.cost << endl;
	cloud_->channels[0].values.push_back(scale_prob);



//ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
	// geometry_msgs::PoseStamped future_i;
	// future_i.header.frame_id = "/robot_0/odom";
	// future_i.header.stamp = ros::Time();
	// future_i.pose.position.x = x_i;
	// future_i.pose.position.y = y_i;
	// future_i.pose.position.z = 0;
	// future_i.pose.orientation.x = 0;
	// future_i.pose.orientation.y = 0;
	// future_i.pose.orientation.z = 0;
	// future_i.pose.orientation.w = 1;

	//int start_s=clock();
	// the code you wish to time goes here


	//cout << "future_i, odom" << "\t" << future_i.pose.position.x << "\t" << future_i.pose.position.y << endl;

	// tf::TransformListener listener;
	// listener.waitForTransform("/robot_0/odom", "/map", ros::Time(0), ros::Duration(1.0));
	// geometry_msgs::PoseStamped mapRobot0Pose;
	// try{
	//       //ROS_INFO("Im here");
	//       listener.transformPose("map", future_i, mapRobot0Pose);
	//       //globalPose1 = mapRobot0Pose;
	//       //ROS_INFO("Robot Future Position (%f,%f)", mapRobot0Pose.pose.position.x, mapRobot0Pose.pose.position.y);
	//   }
	//   catch(tf::TransformException& ex){
	//       ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
	//   }

	//int stop_s=clock();
	//cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 << endl;





	//traj.occ_cost = 0;
	//traj.path_dist = 0;
	//traj.heading_diff = 0;
	//traj.goal_dist = 0;
	//traj.occ_cost = occ_cost;
	//traj.path_dist = path_dist;
	//traj.heading_diff = heading_diff;
	//traj.goal_dist = goal_dist;

	//traj.objectives.clear();
	//traj.objectives.push_back(traj.occ_cost);
	//traj.objectives.push_back(traj.path_dist);
	//traj.objectives.push_back(traj.heading_diff);
	//traj.objectives.push_back(traj.goal_dist);
	//traj.objectives.push_back(traj.inter_dist_fitness);
	//traj.objectives.push_back(traj.cost_);
	//std::cout << traj.objectives.size() << endl;
}

double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
	double heading_diff = DBL_MAX;
	unsigned int goal_cell_x, goal_cell_y;
	const double v2_x = cos(heading);
	const double v2_y = sin(heading);

	// find a clear line of sight from the robot's cell to a farthest point on the path
	for (int i = global_plan_.size() - 1; i >=0; --i) {
		if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
			if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
				double gx, gy;
				costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
				double v1_x = gx - x;
				double v1_y = gy - y;

				double perp_dot = v1_x * v2_y - v1_y * v2_x;
				double dot = v1_x * v2_x + v1_y * v2_y;

				//get the signed angle
				double vector_angle = atan2(perp_dot, dot);

				heading_diff = fabs(vector_angle);
				return heading_diff;
			}
		}
	}
	return heading_diff;
}

//calculate the cost of a ray-traced line
double TrajectoryPlanner::lineCost(int x0, int x1,
                                   int y0, int y1){
	//Bresenham Ray-Tracing
	int deltax = abs(x1 - x0);    // The difference between the x's
	int deltay = abs(y1 - y0);    // The difference between the y's
	int x = x0;                   // Start x off at the first pixel
	int y = y0;                   // Start y off at the first pixel

	int xinc1, xinc2, yinc1, yinc2;
	int den, num, numadd, numpixels;

	double line_cost = 0.0;
	double point_cost = -1.0;

	if (x1 >= x0)             // The x-values are increasing
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else                      // The x-values are decreasing
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y1 >= y0)             // The y-values are increasing
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else                      // The y-values are decreasing
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay)     // There is at least one x-value for every y-value
	{
		xinc1 = 0;        // Don't change the x when numerator >= denominator
		yinc2 = 0;        // Don't change the y for every iteration
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax; // There are more x-values than y-values
	} else {                  // There is at least one y-value for every x-value
		xinc2 = 0;        // Don't change the x for every iteration
		yinc1 = 0;        // Don't change the y when numerator >= denominator
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay; // There are more y-values than x-values
	}

	for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
		point_cost = pointCost(x, y); //Score the current point

		if (point_cost < 0) {
			return -1;
		}

		if (line_cost < point_cost) {
			line_cost = point_cost;
		}

		num += numadd;    // Increase the numerator by the top of the fraction
		if (num >= den) { // Check if numerator >= denominator
			num -= den; // Calculate the new numerator value
			x += xinc1; // Change the x as appropriate
			y += yinc1; // Change the y as appropriate
		}
		x += xinc2;       // Change the x as appropriate
		y += yinc2;       // Change the y as appropriate
	}

	return line_cost;
}

double TrajectoryPlanner::pointCost(int x, int y){
	unsigned char cost = costmap_.getCost(x, y);
	//if the cell is in an obstacle the path is invalid
	if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION) {
		return -1;
	}

	return cost;
}

void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
	global_plan_.resize(new_plan.size());
	for(unsigned int i = 0; i < new_plan.size(); ++i) {
		global_plan_[i] = new_plan[i];
	}

	if( global_plan_.size() > 0 ) {
		geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
		final_goal_x_ = final_goal_pose.pose.position.x;
		final_goal_y_ = final_goal_pose.pose.position.y;
		final_goal_position_valid_ = true;
	} else {
		final_goal_position_valid_ = false;
	}

	if (compute_dists) {
		//reset the map for new operations
		path_map_.resetPathDist();
		goal_map_.resetPathDist();

		//make sure that we update our path based on the global plan and compute costs
		path_map_.setTargetCells(costmap_, global_plan_);
		goal_map_.setLocalGoal(costmap_, global_plan_);
		ROS_DEBUG("Path/Goal distance computed");
	}
}

bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
                                        double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
	Trajectory t;

	double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

	//if the trajectory is a legal one... the check passes
	if(cost >= 0) {
		return true;
	}
	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

	//otherwise the check fails
	return false;
}

double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy,
                                          double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
	Trajectory t;
	double impossible_cost = path_map_.obstacleCosts();
	generateTrajectory(x, y, theta,
	                   vx, vy, vtheta,
	                   vx_samp, vy_samp, vtheta_samp,
	                   acc_lim_x_, acc_lim_y_, acc_lim_theta_,
	                   impossible_cost, t);

	// return the cost.
	return double( -8.0 );
}

//Calculates the fintess value for the inter personal distance objective
vector<double> TrajectoryPlanner::Calculate_Interpersonal_Distance_Fitness(Trajectory& p_traj){
	//this assumes pestimistic view
	double inter_dist_fitness1 = 1000000000;
	double inter_dist_fitness2 = 1000000000;
	double inter_dist_fitness3 = 1000000000;
	//cout << "Inter Distance" << "\t" << p_traj.inter_dist << endl;
	//no follow behavior
	if (follow_behavior == false) {
		if (p_traj.inter_dist.at(0) <= inter_dist_threshold) {
			//if the trajectory makes the robot hit the person
			inter_dist_fitness1 = exp(1/p_traj.inter_dist.at(0));
			if (p_traj.inter_dist.at(0)==0) {
				cout << "Hit Person" << endl;
				//high fitness value should keep the robot from making this choice
				inter_dist_fitness1 = 1000000000;
			}
			//if the robot is within the ID threashold then the fitness value will increase exponentially
		}
		//if the trajectory is outside of the threashold for interpersonal distance
		else{
			//assert (p_traj.inter_dist > inter_dist_threshold);
			//if not in the threashold then robot will not be penalized for this movement
			inter_dist_fitness1 = 0;
		}
	}

	if (follow_behavior == false) {
		if (p_traj.inter_dist.at(1) <= inter_dist_threshold) {
			//if the trajectory makes the robot hit the person
			inter_dist_fitness2 = exp(1/p_traj.inter_dist.at(1));
			if (p_traj.inter_dist.at(1)==0) {
				cout << "Hit Person" << endl;
				//high fitness value should keep the robot from making this choice
				inter_dist_fitness2 = 1000000000;
			}
			//if the robot is within the ID threashold then the fitness value will increase exponentially
		}
		//if the trajectory is outside of the threashold for interpersonal distance
		else{
			//assert (p_traj.inter_dist > inter_dist_threshold);
			//if not in the threashold then robot will not be penalized for this movement
			inter_dist_fitness2 = 0;
		}
	}

	if (follow_behavior == false) {
		if (p_traj.inter_dist.at(2) <= inter_dist_threshold) {
			//if the trajectory makes the robot hit the person
			inter_dist_fitness3 = exp(1/p_traj.inter_dist.at(2));
			if (p_traj.inter_dist.at(2)==0) {
				cout << "Hit Person" << endl;
				//high fitness value should keep the robot from making this choice
				inter_dist_fitness3 = 1000000000;
			}
			//if the robot is within the ID threashold then the fitness value will increase exponentially
		}
		//if the trajectory is outside of the threashold for interpersonal distance
		else{
			//assert (p_traj.inter_dist > inter_dist_threshold);
			//if not in the threashold then robot will not be penalized for this movement
			inter_dist_fitness3 = 0;
		}
	}
	// //follow behavior; we didn't use in IROS, for future use.
	// if (follow_behavior == true){
	//     if (p_traj.inter_dist <= inter_dist_threshold){
	//       //if the trajectory makes the robot hit the person
	//       if (p_traj.inter_dist==0){
	//         cout << "Hit Person" << endl;
	//         //high fitness value should keep the robot from making this choice
	//         inter_dist_fitness = 1000000000;
	//       }
	//       //if the robot is within the ID threashold then the fitness value will increase exponentially
	//       inter_dist_fitness = exp(1/p_traj.inter_dist);
	//     }
	//     //if the trajectory is outside of the threashold for interpersonal distance
	//     else if(p_traj.inter_dist >= inter_dist_threshold+0.5){
	//     //assert (p_traj.inter_dist > inter_dist_threshold);
	//     //if not in the threashold then robot will not be penalized for this movement
	//     inter_dist_fitness = exp(p_traj.inter_dist);
	//     }
	//     else{
	//     //assert (p_traj.inter_dist > inter_dist_threshold);
	//     //if not in the threashold then robot will not be penalized for this movement
	//     inter_dist_fitness = 0;
	//     }
	// }
	vector<double> inter_dist_fitness;
	inter_dist_fitness.clear();
	inter_dist_fitness.push_back(inter_dist_fitness1);
	inter_dist_fitness.push_back(inter_dist_fitness2);
	inter_dist_fitness.push_back(inter_dist_fitness3);
	return inter_dist_fitness;
}

double TrajectoryPlanner::calculateRadiusFitness(Trajectory& p_traj)
{
	double radius_fitness = 1000000000;
	double d_center_threshold = 1.5;
	if (p_traj.d_center <= 0)//d_center_threshold)
	{
		radius_fitness = exp(1/p_traj.d_center);
		if (p_traj.d_center == 0)
		{
			cout << "In the center" << endl;
			radius_fitness = 1000000000;
		}
	}
	else
	{
		radius_fitness = 0;
	}
	return radius_fitness;
}

double TrajectoryPlanner::calculateIntentFitness(Trajectory& p_traj)
{
	double intent_fitness = 1000000000;

	if (p_traj.d_intent > intent_threshold) //&& p_traj.d_intent <= 5.0)
	{
		cout << "In Intent" << endl;
		intent_fitness = exp(p_traj.d_intent);
	}
	if (p_traj.d_intent <= intent_threshold)
	{
		cout << "Reached social area" << endl;
		intent_fitness = 0;
	}
	return intent_fitness;
}


//Checks to see where in the current population of trajectories the passed in trajectory should be placed based on its PaCcET fitness
struct TrajectoryPlanner::Less_Than_Policy_Fitness {
	inline bool operator() (const Trajectory& struct1, const Trajectory& struct2){
		return (struct1.paccet_fitness < struct2.paccet_fitness);
	}
};

//Sorts the population of trajectories based on their PaCcET fitness from lowest to highest by passing in one trajectoriy at a time to the sort function
void TrajectoryPlanner::Sort_Policies_By_Fitness(std::vector<Trajectory> *pac_traj){
	for (int i=0; i<pac_traj->size(); i++) {
		sort(pac_traj->begin(), pac_traj->end(), Less_Than_Policy_Fitness());
	}
}

//Stores the trajectory if it is a legal movement
void TrajectoryPlanner::Store_trajectory(std::vector<Trajectory> *pac_traj, Trajectory& traj){
	if (traj.cost_ >=0) {                       //Ssing the cost/fitness of the first objective if the cost is greater than or equal to zero the trajectory is valid
		//assert(traj.objectives.size()==4);
		pac_traj->push_back(traj);    //Since the trajectory is valid it is stored in a vector of type trajectory
		//std::cout << "PaCcET Has A Trajectoy" << endl;s
		//assert(pac_traj->back().objectives.size()==NUM_OBJECTIVES);
		//assert(pac_traj->back().cost_ >=0);
	}
	//std::cout << "PaCcET Has" << "\t" << pac_traj->size() << "\t" << "Trajectories" << endl;
}


//Gets PaCcET fitness for a trajectory
void TrajectoryPlanner::PaCcET_Fitness(PaCcET *pT, int i, std::vector<Trajectory> *pac_traj)
{
	std::vector<double> MO;             //Creates a multi-objective vector of doubles to store the fitness score for each objective
	std::vector<double>* pMO = &MO;     //Sets a point to address of the previously made vector of doubles
	MO = pac_traj->at(i).objectives;    //Sets the vector to the objective vector of the trajectory
	pT->execute_N_transform(pMO);       //Transforms the multi-objective vector in the PaCcET space
	pac_traj->at(i).paccet_fitness = 0; //Set the PaCcET fintess to zero
	assert (MO.size()==NUM_OBJECTIVES);
	for (int j=0; j<MO.size(); j++) {
		if(j==0) {
			pac_traj->at(i).paccet_fitness += 0.4*MO.at(j);
		}
		if(j==4) {
			pac_traj->at(i).paccet_fitness += 0*MO.at(j);
		}
		if(j==5) {
			pac_traj->at(i).paccet_fitness += 1*MO.at(j);
		}
		if(j==1 || j==2 || j==3) {
			pac_traj->at(i).paccet_fitness += 0.5*MO.at(j); //Uses a unweighted linear combination of the transformed multi-objective fitness values
		}
	}
}


//Gets the PaCcET fitness for each ptrajectory
void TrajectoryPlanner::Get_PaCcET_Fitness(PaCcET *pT, std::vector<Trajectory> *pac_traj)
{
	//for each trajectory this for loop takes each of the objectives fitness scores and pushes them into a objectives vector of type double
	for (int i=0; i<pac_traj->size(); i++) {
		Trajectory* p_traj = &pac_traj->at(i); //SBB this where you start!
		pac_traj->at(i).inter_dist_fitness.clear();
		pac_traj->at(i).inter_dist_fitness = Calculate_Interpersonal_Distance_Fitness(*p_traj);
		pac_traj->at(i).radius_fitness = calculateRadiusFitness(*p_traj);
		pac_traj->at(i).intent_fitness = calculateIntentFitness(*p_traj);
		pac_traj->at(i).objectives.clear();
		//pac_traj->at(i).objectives.push_back(0);
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).cost_);  //traditional cost function
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).inter_dist_fitness.at(0)); //interpersonal distance fitness score
		//Dummy objectives
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).inter_dist_fitness.at(1));
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).inter_dist_fitness.at(2));
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).radius_fitness);
		pac_traj->at(i).objectives.push_back(pac_traj->at(i).intent_fitness);
		assert(pac_traj->at(i).objectives.size()==NUM_OBJECTIVES);    //checks that the number of objectives in the vector is equal to the expected number of objectives
	}

	//cout << "XXXX"  << endl;
	//the Pareto front is updated based on the objective's fitness scores for each trrajectory
	for (int i=0; i<pac_traj->size(); i++)
	{
		pac_traj->at(i).paccet_fitness = 0;
		pT->Pareto_Check(pac_traj->at(i).objectives);
	}

	//cout << "XXyX"  << endl;
	//Perfroms the objective space transformation for each trajectory and assigns a PaCcET fitness score
	for (int i=0; i<pac_traj->size(); i++)
	{
		PaCcET_Fitness(pT, i, pac_traj);
	}
	//cout << "XyyX"  << endl;
}


/*
 * create the trajectories we wish to score
 */
Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
                                                                     double vx, double vy, double vtheta,
                                                                     double acc_x, double acc_y, double acc_theta) {
	//compute feasible velocity limits in robot space
	double max_vel_x = max_vel_x_, max_vel_theta;
	double min_vel_x, min_vel_theta;

	if( final_goal_position_valid_ ) {
		double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
		max_vel_x = min( max_vel_x, final_goal_dist / sim_time_ );
	}

	//should we use the dynamic window approach?
	if (dwa_) {
		max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
		min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

		max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
		min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
	} else {
		max_vel_x = max(min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
		min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

		max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
		min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
	}


	//we want to sample the velocity space regularly
	double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
	double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

	double vx_samp = min_vel_x;
	double vtheta_samp = min_vel_theta;
	double vy_samp = 0.0;

	//keep track of the best trajectory seen so far
	Trajectory* best_traj = &traj_one;
	std::vector<Trajectory> pac_traj;

	best_traj->cost_ = -1.0;
	pac_traj.resize (0);

	Trajectory* comp_traj = &traj_two;
	comp_traj->cost_ = -1.0;

	Trajectory* swap = NULL;

	//any cell with a cost greater than the size of the map is impossible
	double impossible_cost = path_map_.obstacleCosts();

	//if we're performing an escape we won't allow moving forward
	if (!escaping_) {
		//loop through all x velocities
		for(int i = 0; i < vx_samples_; ++i) {
			vtheta_samp = 0;
			//first sample the straight trajectory
			generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
			                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

			//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
			Store_trajectory(&pac_traj, *comp_traj);

			//if the new trajectory is better... let's take it
			//**this will no longer be the case with paccet
			if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)) {
				swap = best_traj;
				best_traj = comp_traj;
				comp_traj = swap;
			}

			vtheta_samp = min_vel_theta;
			//next sample all theta trajectories
			for(int j = 0; j < vtheta_samples_ - 1; ++j) {
				generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
				                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

				//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
				Store_trajectory(&pac_traj, *comp_traj);

				//if the new trajectory is better... let's take it
				if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)) {
					swap = best_traj;
					best_traj = comp_traj;
					comp_traj = swap;
				}
				vtheta_samp += dvtheta;
			}
			vx_samp += dvx;
		}

		//only explore y velocities with holonomic robots
		if (holonomic_robot_) {
			//explore trajectories that move forward but also strafe slightly
			vx_samp = 0.1;
			vy_samp = 0.1;
			vtheta_samp = 0.0;
			generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
			                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

			//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
			Store_trajectory(&pac_traj, *comp_traj);

			//if the new trajectory is better... let's take it
			if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)) {
				swap = best_traj;
				best_traj = comp_traj;
				comp_traj = swap;
			}

			vx_samp = 0.1;
			vy_samp = -0.1;
			vtheta_samp = 0.0;
			generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
			                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

			//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
			Store_trajectory(&pac_traj, *comp_traj);

			//if the new trajectory is better... let's take it
			if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)) {
				swap = best_traj;
				best_traj = comp_traj;
				comp_traj = swap;
			}
		}
	} // end if not escaping

	//next we want to generate trajectories for rotating in place
	vtheta_samp = min_vel_theta;
	vx_samp = 0.0;
	vy_samp = 0.0;

	//let's try to rotate toward open space
	double heading_dist = DBL_MAX;

	for(int i = 0; i < vtheta_samples_; ++i) {
		//enforce a minimum rotational velocity because the base can't handle small in-place rotations
		double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_)
	: min(vtheta_samp, -1.0 * min_in_place_vel_th_);

		generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
		                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

		//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
		Store_trajectory(&pac_traj, *comp_traj);

		//if the new trajectory is better... let's take it...
		//note if we can legally rotate in place we prefer to do that rather than move with y velocity
		if(comp_traj->cost_ >= 0
		   && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
		   && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)) {
			double x_r, y_r, th_r;
			comp_traj->getEndpoint(x_r, y_r, th_r);
			x_r += heading_lookahead_ * cos(th_r);
			y_r += heading_lookahead_ * sin(th_r);
			unsigned int cell_x, cell_y;

			//make sure that we'll be looking at a legal cell
			if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
				double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
				if (ahead_gdist < heading_dist) {
					//if we haven't already tried rotating left since we've moved forward
					if (vtheta_samp < 0 && !stuck_left) {
						swap = best_traj;
						best_traj = comp_traj;
						comp_traj = swap;
						heading_dist = ahead_gdist;
					}
					//if we haven't already tried rotating right since we've moved forward
					else if(vtheta_samp > 0 && !stuck_right) {
						swap = best_traj;
						best_traj = comp_traj;
						comp_traj = swap;
						heading_dist = ahead_gdist;
					}
				}
			}
		}

		vtheta_samp += dvtheta;
	}

	//do we have a legal trajectory
	if (best_traj->cost_ >= 0) {
		// avoid oscillations of in place rotation and in place strafing
		if ( !(best_traj->xv_ > 0)) {
			if (best_traj->thetav_ < 0) {
				if (rotating_right) {
					stuck_right = true;
				}
				rotating_right = true;
			} else if (best_traj->thetav_ > 0) {
				if (rotating_left) {
					stuck_left = true;
				}
				rotating_left = true;
			} else if(best_traj->yv_ > 0) {
				if (strafe_right) {
					stuck_right_strafe = true;
				}
				strafe_right = true;
			} else if(best_traj->yv_ < 0) {
				if (strafe_left) {
					stuck_left_strafe = true;
				}
				strafe_left = true;
			}

			//set the position we must move a certain distance away from
			prev_x_ = x;
			prev_y_ = y;
		}

		double dist = hypot(x - prev_x_, y - prev_y_);
		if (dist > oscillation_reset_dist_) {
			rotating_left = false;
			rotating_right = false;
			strafe_left = false;
			strafe_right = false;
			stuck_left = false;
			stuck_right = false;
			stuck_left_strafe = false;
			stuck_right_strafe = false;
		}

		dist = hypot(x - escape_x_, y - escape_y_);
		if(dist > escape_reset_dist_ ||
		   fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
			escaping_ = false;
		}

		return *best_traj;
	}

	//only explore y velocities with holonomic robots
	if (holonomic_robot_) {
		//if we can't rotate in place or move forward... maybe we can move sideways and rotate
		vtheta_samp = min_vel_theta;
		vx_samp = 0.0;

		//loop through all y velocities
		for(unsigned int i = 0; i < y_vels_.size(); ++i) {
			vtheta_samp = 0;
			vy_samp = y_vels_[i];
			//sample completely horizontal trajectories
			generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
			                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

			if(comp_traj->cost_>=0) {
				//assert(comp_traj->objectives.size()==NUM_OBJECTIVES);
			}

			//If running PaCcET and the trajectory is valid i.e. cost_>=0 the trajectory is stored for PaccET
			Store_trajectory(&pac_traj, *comp_traj);

			//if the new trajectory is better... let's take it
			if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)) {
				double x_r, y_r, th_r;
				comp_traj->getEndpoint(x_r, y_r, th_r);
				x_r += heading_lookahead_ * cos(th_r);
				y_r += heading_lookahead_ * sin(th_r);
				unsigned int cell_x, cell_y;

				//make sure that we'll be looking at a legal cell
				if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
					double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
					if (ahead_gdist < heading_dist) {
						//if we haven't already tried strafing left since we've moved forward
						if (vy_samp > 0 && !stuck_left_strafe) {
							swap = best_traj;
							best_traj = comp_traj;
							comp_traj = swap;
							heading_dist = ahead_gdist;
						}
						//if we haven't already tried rotating right since we've moved forward
						else if(vy_samp < 0 && !stuck_right_strafe) {
							swap = best_traj;
							best_traj = comp_traj;
							comp_traj = swap;
							heading_dist = ahead_gdist;
						}
					}
				}
			}
		}
	}

	//do we have a legal trajectory
	if (best_traj->cost_ >= 0) {
		if (!(best_traj->xv_ > 0)) {
			if (best_traj->thetav_ < 0) {
				if (rotating_right) {
					stuck_right = true;
				}
				rotating_left = true;
			} else if(best_traj->thetav_ > 0) {
				if(rotating_left) {
					stuck_left = true;
				}
				rotating_right = true;
			} else if(best_traj->yv_ > 0) {
				if(strafe_right) {
					stuck_right_strafe = true;
				}
				strafe_left = true;
			} else if(best_traj->yv_ < 0) {
				if(strafe_left) {
					stuck_left_strafe = true;
				}
				strafe_right = true;
			}

			//set the position we must move a certain distance away from
			prev_x_ = x;
			prev_y_ = y;

		}

		double dist = hypot(x - prev_x_, y - prev_y_);
		if(dist > oscillation_reset_dist_) {
			rotating_left = false;
			rotating_right = false;
			strafe_left = false;
			strafe_right = false;
			stuck_left = false;
			stuck_right = false;
			stuck_left_strafe = false;
			stuck_right_strafe = false;
		}

		dist = hypot(x - escape_x_, y - escape_y_);
		if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
			escaping_ = false;
		}

		return *best_traj;
	}

	//This is the main  function for running PaCcET
	PaCcET* pT;                                       //A pointer to the PaCcET class is always created regardless of running PaCcET
	if (run_paccet == true)
	{                                                  //If we are running PaCcET the following will happen
		PaCcET T;                                 //An instance of the PaCcET class is created
		pT = &T;                                  //Set the point to the address of the PaCcET instance
		Get_PaCcET_Fitness(pT, &pac_traj);        //Performs the Pareto check, objective space transformation, and gives a single PaCcET fitness to each trajectory
		Sort_Policies_By_Fitness(&pac_traj);      //Sorts the trajectories from best to worst based on their PaCcET fitness
		//std::cout << "PaCcET Has" << "\t" << pac_taj.size() << "\t" << "Trajectories" << endl;

		//sets the best trajectory as the best paccet trajectory
		if (pac_traj.size()==0)
		{                  //If there were no trajectories stored for PaCcET
			//std::cout << "PaCcET Has No Trajectories" << endl;
		}
		if (pac_traj.size()>0)
		{                   //If there exists at least one trajectory for PaCcET
			//std::cout << "PaCcET Has" << "\t" << pac_traj.size() << "\t" << "Trajectories" << endl;
			cout << "Inter_Distance" << "\t P_1 " << pac_traj.at(0).inter_dist.at(0) << "\t P_2 " << pac_traj.at(0).inter_dist.at(1) << "\t P_3 " << pac_traj.at(0).inter_dist.at(2) << "\t d_center "<< pac_traj.at(0).d_center << "\t d_intent "<< pac_traj.at(0).d_intent << endl; //displays the interpersonal distance for the best PaCcET trajectory
			*best_traj = pac_traj.at(0);      //Sets the best trajectory for the local planner to the best PaCcET trajectory
		}
	}

	return *best_traj;                                //Returns the best trajectory to the local planners

	//and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
	//vtheta_samp = 0.0;
	//vx_samp = backup_vel_;
	//vy_samp = 0.0;
	generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
	                   acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

	//not needed as it would be the only move avaliable so just select it
	//Store_trajectory(pac_traj, *comp_traj);

	//if the new trajectory is better... let's take it
	/*
	   if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
	   swap = best_traj;
	   best_traj = comp_traj;
	   comp_traj = swap;
	   }
	 */

	//we'll allow moving backwards slowly even when the static map shows it as blocked
	swap = best_traj;
	best_traj = comp_traj;
	comp_traj = swap;

	double dist = hypot(x - prev_x_, y - prev_y_);
	if (dist > oscillation_reset_dist_) {
		rotating_left = false;
		rotating_right = false;
		strafe_left = false;
		strafe_right = false;
		stuck_left = false;
		stuck_right = false;
		stuck_left_strafe = false;
		stuck_right_strafe = false;
	}

	//only enter escape mode when the planner has given a valid goal point
	if (!escaping_ && best_traj->cost_ > -2.0) {
		escape_x_ = x;
		escape_y_ = y;
		escape_theta_ = theta;
		escaping_ = true;
	}

	dist = hypot(x - escape_x_, y - escape_y_);

	if (dist > escape_reset_dist_ ||
	    fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
		escaping_ = false;
	}


	//if the trajectory failed because the footprint hits something, we're still going to back up
	if(best_traj->cost_ == -1.0)
		best_traj->cost_ = 1.0;

	return *best_traj;

}

//given the current state of the robot, find a good trajectory
Trajectory TrajectoryPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                                               geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities) {

	Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
	Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));

	//reset the map for new operations
	path_map_.resetPathDist();
	goal_map_.resetPathDist();

	//temporarily remove obstacles that are within the footprint of the robot
	std::vector<base_local_planner::Position2DInt> footprint_list =
		footprint_helper_.getFootprintCells(
			pos,
			footprint_spec_,
			costmap_,
			true);

	//mark cells within the initial footprint of the robot
	for (unsigned int i = 0; i < footprint_list.size(); ++i) {
		path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
	}

	//make sure that we update our path based on the global plan and compute costs
	path_map_.setTargetCells(costmap_, global_plan_);
	goal_map_.setLocalGoal(costmap_, global_plan_);
	ROS_DEBUG("Path/Goal distance computed");

	//rollout trajectories and find the minimum cost one
	Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
	                                                         vel[0], vel[1], vel[2],
	                                                         acc_lim_x_, acc_lim_y_, acc_lim_theta_);
	ROS_DEBUG("Trajectories created");

	/*
	   //If we want to print a ppm file to draw goal dist
	   char buf[4096];
	   sprintf(buf, "base_local_planner.ppm");
	   FILE *fp = fopen(buf, "w");
	   if(fp){
	   fprintf(fp, "P3\n");
	   fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
	   fprintf(fp, "255\n");
	   for(int j = map_.size_y_ - 1; j >= 0; --j){
	    for(unsigned int i = 0; i < map_.size_x_; ++i){
	      int g_dist = 255 - int(map_(i, j).goal_dist);
	      int p_dist = 255 - int(map_(i, j).path_dist);
	      if(g_dist < 0)
	        g_dist = 0;
	      if(p_dist < 0)
	        p_dist = 0;
	      fprintf(fp, "%d 0 %d ", g_dist, 0);
	    }
	    fprintf(fp, "\n");
	   }
	   fclose(fp);
	   }
	 */

	if(best.cost_ < 0) {
		drive_velocities.pose.position.x = 0;
		drive_velocities.pose.position.y = 0;
		drive_velocities.pose.position.z = 0;
		drive_velocities.pose.orientation.w = 1;
		drive_velocities.pose.orientation.x = 0;
		drive_velocities.pose.orientation.y = 0;
		drive_velocities.pose.orientation.z = 0;
	}
	else{
		drive_velocities.pose.position.x = best.xv_;
		drive_velocities.pose.position.y = best.yv_;
		drive_velocities.pose.position.z = 0;
		tf2::Quaternion q;
		q.setRPY(0, 0, best.thetav_);
		tf2::convert(q, drive_velocities.pose.orientation);
	}

	return best;
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
	//check if the footprint is legal
	return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
}


void TrajectoryPlanner::getLocalGoal(double& x, double& y){
	x = path_map_.goal_x_;
	y = path_map_.goal_y_;
}

};
