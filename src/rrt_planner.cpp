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
	private_nh_.param<std::string>("/map_topic", map_topic, "/map");
	private_nh_.param<std::string>("/path_topic", path_topic, "/path");

	// Get RRT parameters from parameter server
	private_nh_.getParam("/RRT_show_planning", showPlanning_);
	private_nh_.getParam("/RRT_K", K_);
	private_nh_.getParam("/RRT_timestep", timestep_);
	private_nh_.getParam("/RRT_vel_max", velMax_);
	private_nh_.getParam("/RRT_occupied_threshold", occupiedThreshold_);
	private_nh_.getParam("/RRT_goal_bias", goalBias_);

	ROS_INFO("Parameters loaded | K=%d, timestep=%0.1f, vel_max=%0.1f, goal bias=%0.1f", K_, timestep_, velMax_, goalBias_);

	// Initialize tree
	generator = std::default_random_engine (std::chrono::system_clock::now().time_since_epoch().count());

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
	if (map_received_ == true) {
		return;
	}

	map_grid_ = msg;

	// Store map bounds
	xLimitLower_ = msg->info.origin.position.x;
	yLimitLower_ = msg->info.origin.position.y;
	xLimitUpper_ = xLimitLower_ + msg->info.width * msg->info.resolution;
	yLimitUpper_ = yLimitLower_ + msg->info.height * msg->info.resolution;

	// Store map origin
	mapOrigin_ = Point2D(msg->info.origin.position.x, msg->info.origin.position.y);

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

	// Convert msg to Point2D
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
	bool planSuccessful = false;

	// TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
	//       path through the map starting from the initial pose and ending at the goal pose

	// Reset tree
	treeInit_.reset(K_);
	treeGoal_.reset(K_);

	// Add initial pose to tree
	treeInit_.addPoint(init_pose_);
	treeGoal_.addPoint(goal_);
	ROS_INFO("init_pose_: (%0.2f, %0.2f)", init_pose_[0], init_pose_[1]);
	ROS_INFO("goal_: (%0.2f, %0.2f)", goal_[0], goal_[1]);

	// Iterate to get K vertices
	int kInit = 0;
	int kGoal = 0;
	Point2D xLastInit = init_pose_;
	Point2D xLastGoal = goal_;
	while (kInit < K_ && kGoal < K_) {
		bool pathValid;
		// Alternate between tree at init and tree at goal
		if (kInit < kGoal) {
			planSuccessful = generateRRT(&treeInit_, xLastGoal, &pathValid, &xLastInit);
			if (pathValid) { kInit++; }
		} else {
			planSuccessful = generateRRT(&treeGoal_, xLastInit, &pathValid, &xLastGoal);
			if (pathValid) { kGoal++; }
		}

		if (planSuccessful) { break; }
	}

	if (planSuccessful) {
		// Retrieve path from tree
		std::vector<Point2D> path = treeInit_.findPath();
		std::vector<Point2D> path2 = treeGoal_.findPath();
		std::reverse(path2.begin(), path2.end());
		path.insert(path.end(), path2.begin(), path2.end());

		// Publish
		publishPath(path);

		// Draw path on map
		Point2D prev_it = init_pose_;
		for (auto& it : path) {
			drawLine(prev_it, it, cv::Scalar(12, 255, 12), 2);
			prev_it = it;
		}
	}
}

bool RRTPlanner::generateRRT(Tree *tree, Point2D target, bool *pathValid, Point2D *x_last)
{
	// Find random state
	Point2D x_rand = randomState(target);

	// Find nearest neighbour to random state
	int nearestIndex = tree->findNearest(x_rand);
	Point2D nearestNeighbour = tree->retrieve(nearestIndex);

	// Find control input and new state
	Point2D u = selectControlInput(nearestNeighbour, x_rand);
	Point2D x_new = computeNewState(nearestNeighbour, x_rand, u);

	// Select input to get to random state
	// In simple (x,y) state case, draw a line between and check for intersections
	*pathValid = checkCollisionFree(x_new, nearestNeighbour);

	// Add vertex & edge to tree
	// Assume quadcopter is holomonic, therefore use x_rand instead of x_new
	if (*pathValid) {
		tree->addPoint(x_new, nearestIndex);

		if (showPlanning_) {
			drawLine(nearestNeighbour, x_new, cv::Scalar(255, 12, 12), 1);
			drawCircle(x_new, 2, cv::Scalar(12, 12, 255));
			displayMapImage();
		}

		// Check if reached goal
		float distToGoal = (target - x_new).norm();
		if (distToGoal < 0.5*(velMax_ * timestep_) && checkCollisionFree(x_new, target)) {
			// Add goal to tree
			int nearestIndex = tree->findNearest(target);
			tree->addPoint(target, nearestIndex);

			ROS_INFO("Reached!");
			return true;
		}

		// Update x_last for other tree
		*x_last = x_new;
	}

	return false;
}

Point2D RRTPlanner::randomState(Point2D target)
{
	// Add bias toward goal
	std::uniform_real_distribution<float> distributionGoalBias(0, 1);
	float setAsGoal = distributionGoalBias(generator);

	if (setAsGoal <= goalBias_) {
		return target;
	} else {
		// Generate random state within map boundaries
		std::uniform_real_distribution<float> distributionX(xLimitLower_, xLimitUpper_);
		float random_x = distributionX(generator);

		std::uniform_real_distribution<float> distributionY(yLimitLower_, yLimitUpper_);
		float random_y = distributionY(generator);

		return Point2D(random_x, random_y);
	}
}

bool RRTPlanner::checkCollisionFree(Point2D p1, Point2D p2)
{
	// TODO
	
	// Use a line intersection as a 2D approximation of collision checking (because state is x,y)
	// Use openCV's LineIterator as a well implemented Bresenham's
	cv::LineIterator it(*map_, rosToCVPoint(p1), rosToCVPoint(p2), 4);

	bool collision = false;
	for (int i = 0; i < it.count; i++, ++it)
	{
		Point2D pose = CVToRosPoint(it.pos());
		if (!isPointUnoccupied(pose)) {
			// [DEBUG] Mark collisions
			if (showPlanning_) {
				cv::circle(*map_, it.pos(), 2, cv::Scalar(0,255,0), -1);
			}

			return false;
		}
	}

	return true;
}

Point2D RRTPlanner::selectControlInput(Point2D initial, Point2D destination)
{
	// Set control input as a straight line from initial to destination
	Point2D u = destination - initial;

	if (u.norm() < 1e-6) {
		throw std::runtime_error("selectControlInput | Points are too close together");
	}

	// Implement a constant velocity model, therefore use max velocity
	u = u * (velMax_ / u.norm());

	return u;
}

Point2D RRTPlanner::computeNewState(Point2D initial, Point2D destination, Point2D control_input)
{
	// Use first-order euler integration (Sufficient for a first-order system, vel -> pos)
	Point2D step = control_input * timestep_;

	if (step.norm() < (destination-initial).norm()) {
		return initial + step;
	} else {
		return destination;
	}
}

void RRTPlanner::publishPath(std::vector<Point2D> path)
{
	// Create new Path msg
	nav_msgs::Path msg;
	msg.header.frame_id = map_grid_->header.frame_id;
	msg.header.stamp = ros::Time::now();

	// TODO: Fill nav_msgs::Path msg with the path calculated by RRT

	// Convert vector of Point2Ds to array of PoseStamped
	for (auto& it : path) {
		geometry_msgs::PoseStamped newPose;
		newPose.header = msg.header;
		newPose.pose.position.x = it[0];
		newPose.pose.position.y = it[1];

		msg.poses.push_back(newPose);
	}

	// Publish the calculated path
	path_pub_.publish(msg);

	displayMapImage();
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
	// TODO: Fill out this function to check if a given point is occupied/free in the map

	// Convert Point2D to grid location
	Point2D shifted = p - mapOrigin_;
	shifted[0] = round(shifted[0] / map_grid_->info.resolution);
	shifted[1] = round(shifted[1] / map_grid_->info.resolution);

	// ROS_INFO("(%0.1f, %0.1f) -> (%d,%d)", p[0],p[1], shifted[0],shifted[1]);

	if (shifted[0] < 0 || shifted[0] > map_grid_->info.width
		|| shifted[1] < 0 || shifted[1] > map_grid_->info.height) {
		ROS_WARN("Grid indices exceed grid");
		return false;
	}

	// Check map
	bool occupied = (map_grid_->data[toIndex(shifted)] > occupiedThreshold_);

	return !occupied;
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
	cv::namedWindow("Output", cv::WINDOW_NORMAL);
	cv::imshow("Output", *map_);
	cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
	cv::circle(*map_,
			   rosToCVPoint(p),
			   radius,
			   color,
			   -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
	cv::line(
		*map_,
		rosToCVPoint(p1),
		rosToCVPoint(p2),
		color,
		thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = p[1];
	pose.pose.position.y = p[0];
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = map_grid_->header.frame_id;
	return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
	p[0] = pose.position.x;
	p[1] = pose.position.y;
}

inline cv::Point RRTPlanner::rosToCVPoint(Point2D p)
{
	// Convert from ROS ENU 	x: right	y: up
	// to CV's coordainte frame	x: right	y: down
	// Also shift the origin to match the map origin
	Point2D shifted = (p - mapOrigin_) / map_grid_->info.resolution;
	return cv::Point(shifted[0], map_grid_->info.height - shifted[1]);
}

inline Point2D RRTPlanner::CVToRosPoint(cv::Point pt)
{
	// Convert from CV's coordainte	x: right	y: down
	// to ROS ENU 					x: right	y: up
	// Also shift the origin to match the map origin
	Point2D p(pt.x, map_grid_->info.height - pt.y);
	return p * map_grid_->info.resolution + mapOrigin_;
}

inline int RRTPlanner::toIndex(int x, int y)
{
	return x * map_grid_->info.width + y;
}

inline int RRTPlanner::toIndex(Point2D p)
{
	return p[0] + p[1] * map_grid_->info.width;
}

}  // namespace rrt_planner
