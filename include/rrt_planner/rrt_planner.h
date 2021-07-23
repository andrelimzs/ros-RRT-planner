#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <random>

namespace rrt_planner
{

/**
 * A utility class to represent a 2D point
 */
typedef Eigen::Vector2d Point2D;

/**
 * Utility class to represent a (simplified) tree
 * Vertices can be added but not removed
 * Each vertex will be added with an edge at the same time
 */
class Tree
{
public:
	void reset(int size) {
		adj_.resize(size, size);
		adj_ = Eigen::MatrixXi::Zero(size, size);
	}

	float computeDistance(Point2D a, Point2D b) {
		return (a-b).norm();
	}

	int findNearest(Point2D newPoint) {
		// Find the nearest neighbour (in Euclidean distance)
		float minDist = INFINITY;
		int minIndex = -1;
		for(int i = 0; i < points.size(); i++) {
			float newDist = computeDistance(newPoint, points[i]);
			if (newDist < minDist) {
				minDist = newDist;
				minIndex = i;
			}
		}

		if (minIndex < 0) { throw "Tree.findNearest failed to find a solution"; }

		return minIndex;
	}

	void addPoint(Point2D newPoint, int nearIndex) {
		// Store (x,y)
		points.push_back(newPoint);

		// Add edge
		int newIndex = points.size() - 1;
		adj_(newIndex, nearIndex) = 1;
		adj_(nearIndex, newIndex) = 1;
	}

	Point2D retrieve(int index) {
		return points[index];
	}

private:
	Eigen::MatrixXi adj_;
	std::vector<Point2D> points;
};


/**
 * Main class which implements the RRT algorithm
 */
class RRTPlanner
{
public:
	explicit RRTPlanner(ros::NodeHandle *);

	~RRTPlanner() = default;

	/**
	 * Given a map, the initial pose, and the goal, this function will plan
	 * a collision-free path through the map from the initial pose to the goal
	 * using the RRT algorithm
	 *
	 * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
	 */
	void plan();

	/**
	 * Callback for map subscriber
	 */
	void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);

	/**
	 * Callback for initial pose subscriber
	 */
	void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

	/**
	 * Callback for goal subscriber
	 */
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

private:

	/**
	 * Publishes the path calculated by RRT as a nav_msgs::Path msg
	 *
	 * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
	 */
	void publishPath();

	/**
	 * Utility function to check if a given point is free/occupied in the map
	 * @param p: point in the map
	 * @return boolean true if point is unoccupied, false if occupied
	 *
	 * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
	 */
	bool isPointUnoccupied(const Point2D & p);

	/**
	 * Utility function to build a CV::Mat from a nav_msgs::OccupancyGrid for display
	 */
	void buildMapImage();

	/**
	 * Utility function to display the CV::Mat map image
	 * @param delay
	 */
	void displayMapImage(int delay = 1);

	/**
	 * Utility function to draw initial pose and goal pose on the map image
	 */
	void drawGoalInitPose();

	/**
	 * Utility function to draw a circle on the map
	 * @param p: center point of the circle
	 * @param radius: radius of the circle
	 * @param color: color of the circle
	 */
	void drawCircle(Point2D & p, int radius, const cv::Scalar & color);

	/**
	 * Utility function to draw a line on the map
	 * @param p1: starting point of the line
	 * @param p2: end point of the line
	 * @param color: color of the line
	 * @param thickness: thickness of the line
	 */
	void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);

	/**
	 * Utility function to convert a Point2D object to a geometry_msgs::PoseStamped object
	 * @return corresponding geometry_msgs::PoseStamped object
	 */
	inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

	/**
	 * Utility function to convert a geometry_msgs::PoseStamped object to a Point2D object
	 */
	inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

	/**
	 * Utility function to convert (x, y) matrix coordinate to corresponding vector coordinate
	 */
	inline int toIndex(int, int);

	/**
	 * Generate a random state
	 */
	Point2D randomState();

	/**
	 * Check for collisions between two points
	 */
	bool checkCollisionFree(Point2D, Point2D);

	/**
	 * Generate control input, u from two points
	 * (This will be useful when generalizing RRT to higher dimensions)
	 */
	Point2D selectControlInput(Point2D, Point2D);

	/**
	 * Calculate new state from a initial point and control input
	 * (This will be useful when generalizing RRT to higher dimensions)
	 */
	Point2D computeNewState(Point2D, Point2D);

	ros::NodeHandle * nh_;
	ros::NodeHandle private_nh_;

	bool map_received_;
	std::unique_ptr<cv::Mat> map_;
	nav_msgs::OccupancyGrid::Ptr map_grid_;

	bool init_pose_received_;
	Point2D init_pose_;

	bool goal_received_;
	Point2D goal_;

	ros::Subscriber map_sub_;
	ros::Subscriber init_pose_sub_;
	ros::Subscriber goal_sub_;
	ros::Publisher path_pub_;

	// RRT Parameters
	Tree tree_;
	int K_;
	float timestep_;
	float velMax_;

	float xLimitLower_;
	float xLimitUpper_;
	float yLimitLower_;
	float yLimitUpper_;
};

}

#endif  // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
