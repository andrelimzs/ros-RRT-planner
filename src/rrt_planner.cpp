#include "rrt_planner/rrt_planner.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
	private_nh_("~"),
	map_received_(false),
	init_pose_received_(false),
	goal_received_(false)
{
	// Get map and path topics from parameter server
	std::string map_topic, path_topic;
	private_nh_.param<std::string>("map_topic", map_topic, "/map");
	private_nh_.param<std::string>("path_topic", path_topic, "/path");

	// Subscribe to map topic
	map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
		map_topic, 1, &RRTPlanner::mapCallback, this);

	// Subscribe to initial pose topic that is published by RViz
	init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
		"/initialpose", 1, &RRTPlanner::initPoseCallback, this);

	// Subscribe to goal topic that is published by RViz
	goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
		"/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

	// Advertise topic where calculated path is going to be published
	path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

	// This loops until the node is running, will exit when the node is killed
	while (ros::ok()) {
		// if map, initial pose, and goal have been received
		// build the map image, draw initial pose and goal, and plan
		if (map_received_ && init_pose_received_ && goal_received_) {
			buildMapImage();
			drawGoalInitPose();
			plan();
		} else {
			if (map_received_) {
				displayMapImage();
			}
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		}
	}
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
	map_grid_ = msg;

	// Build and display the map image
	buildMapImage();
	displayMapImage();

	// Reset these values for a new planning iteration
	map_received_ = true;
	init_pose_received_ = false;
	goal_received_ = false;

	ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
	if (init_pose_received_) {
		buildMapImage();
	}

	// Convert mas to Point2D
	poseToPoint(init_pose_, msg->pose.pose);

	// Reject the initial pose if the given point is occupied in the map
	if (!isPointUnoccupied(init_pose_)) {
		init_pose_received_ = false;
		ROS_WARN(
			"The initial pose specified is on or too close to an obstacle please specify another point");
	} else {
		init_pose_received_ = true;
		drawGoalInitPose();
		ROS_INFO("Initial pose obtained successfully.");
	}

	displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	if (goal_received_) {
		buildMapImage();
	}

	// Convert msg to Point2D
	poseToPoint(goal_, msg->pose);

	// Reject the goal pose if the given point is occupied in the map
	if (!isPointUnoccupied(goal_)) {
		goal_received_ = false;
		ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
	} else {
		goal_received_ = true;
		drawGoalInitPose();
		ROS_INFO("Goal obtained successfully.");
	}

	displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
	if (goal_received_) {
		drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
	}
	if (init_pose_received_) {
		drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
	}
}

void RRTPlanner::plan()
{
	// Reset these values so planning only happens once for a
	// given pair of initial pose and goal points
	goal_received_ = false;
	init_pose_received_ = false;

	// TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
	//       path through the map starting from the initial pose and ending at the goal pose
}

void RRTPlanner::publishPath()
{
	// Create new Path msg
	nav_msgs::Path path;
	path.header.frame_id = map_grid_->header.frame_id;
	path.header.stamp = ros::Time::now();

	// TODO: Fill nav_msgs::Path msg with the path calculated by RRT

	// Publish the calculated path
	path_pub_.publish(path);

	displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
	// TODO: Fill out this function to check if a given point is occupied/free in the map

	return true;
}

void RRTPlanner::buildMapImage()
{
	// Create a new opencv matrix with the same height and width as the received map
	map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
																							map_grid_->info.width,
																							CV_8UC3,
																							cv::Scalar::all(255)));

	// Fill the opencv matrix pixels with the map points
	for (int i = 0; i < map_grid_->info.height; i++) {
		for (int j = 0; j < map_grid_->info.width; j++) {
			if (map_grid_->data[toIndex(i, j)]) {
				map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
			} else {
				map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
			}
		}
	}
}

void RRTPlanner::displayMapImage(int delay)
{
	cv::imshow("Output", *map_);
	cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
	cv::circle(
		*map_,
		cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
		radius,
		color,
		-1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
	cv::line(
		*map_,
		cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
		cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
		color,
		thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = p.y() * map_grid_->info.resolution;
	pose.pose.position.y = p.x() * map_grid_->info.resolution;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = map_grid_->header.frame_id;
	return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
	p.x(pose.position.y / map_grid_->info.resolution);
	p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
	return x * map_grid_->info.width + y;
}

}  // namespace rrt_planner
