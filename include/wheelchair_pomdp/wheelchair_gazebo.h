#ifndef WHEELCHAIR_GAZEBO_H
#define WHEELCHAIR_GAZEBO_H

#include "wheelchair_model.h"

#include <despot/interface/world.h>
// #include <despot/util/coord.h>
#include <ros/ros.h>

// #include <chrono>

// actionlib
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
// #include <gazebo_msgs/ModelState.h>
// #include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Custom Messages
#include <voronoi_msgs_and_types/PathList.h>
// #include "wheelchair_dmp.h"

// #include "wheelchair_pomdp/plot_scan.hpp"

using namespace despot;

class GazeboWheelchairWorld: public World
{
public:
    ros::NodeHandlePtr node_handle_;
    // ros::Subscriber scan_subscriber_;
    ros::Subscriber path_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber joystick_subscriber_;
    ros::Subscriber goal_subscriber_;
    ros::Subscriber map_subscriber_;
    ros::Subscriber bdwa_subscriber_;
    ros::Subscriber pdwa_subscriber_;
    ros::Subscriber weight_subscriber_;

    ros::Publisher vel_publisher_;
    ros::Publisher ucmd_publisher_;
    ros::Publisher pomdp_publisher_;
    ros::Publisher cancel_publisher_;
    // ros::Publisher weight_publisher_;
    ros::Publisher path_publisher_;
    ros::Publisher footprint_publisher_;
    ros::Publisher adapt_publisher_;
    ros::Publisher goal_publisher_;
    ros::Timer update_timer_;
    ros::Timer publish_timer_;
    // ros::Publisher wypt_publisher_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    visualization_msgs::Marker cmd_line;
    visualization_msgs::Marker pomdp_line;
    visualization_msgs::Marker adapt_txt;
    
    WheelchairState* wheelchair_state_;
    WheelchairDSPOMDP* wheelchair_model_;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    // geometry_msgs::TransformStamped lidarTransform;

    GazeboWheelchairWorld(WheelchairDSPOMDP* model_);
    virtual ~GazeboWheelchairWorld();
    
    WheelchairStruct initial_wheelchair_local_status;

    // empty global goal ID
    actionlib_msgs::GoalID dummy_ID;

    // obtained global goal_list
    // geometry_msgs::PoseArray goal_list;

    // // obtained local goal_list
    // geometry_msgs::PoseArray local_goal_list;

    geometry_msgs::TransformStamped lidar2baseTransform;

    float tf_buffer_timeout = 0.3;

    // float waypoint_dist = 2.0;

    int step_no = 0; //Used to track step count to generate the filenames for printing intermediate goals
    

    // std::chrono::time_point<std::chrono::system_clock> start_time;

    // std::chrono::time_point<std::chrono::system_clock> exe_time;

    // ros::Time exe_time_ros;

    // visualization_msgs::Marker waypoint_viz;

    // lidar info
    // int img_count = 0;
    // plot_scan::PlotScan plot_scan_;
    // sensor_msgs::LaserScanConstPtr scan_pointer;
    // std::vector<geometry_msgs::Point> lidar_points;
    
    // bool scan_receive = false;
    // joystick info
    geometry_msgs::Point joystick_input;
    bool joystick_receive = false;

    bool zero_joystick_input = false;

    // bool value to check whether the wheelchair is inside the collision zone
    bool inside_collision = false;

    // pixel values of the base link and back_link
    int base_pixel = 0, back_pixel = 0;

    // current velocity info
    geometry_msgs::Twist current_vel;
    bool odom_receive = false;

    // final published velocity
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist pub_cmd_vel;
    geometry_msgs::Twist old_cmd_vel;

    // current agent pose
    geometry_msgs::Pose current_pose;

    // path info obtained from published paths
    voronoi_msgs_and_types::PathList path_list;
    bool path_receive = false;

    // the intermediate goals in world frame
    voronoi_msgs_and_types::PathList ex_intermediate_goals;

    // the intermediate goals in local frame
    voronoi_msgs_and_types::PathList in_intermediate_goals;

    // vector of waypoints, vector size same as number of paths
	std::vector<geometry_msgs::PoseStamped> goal_positions;

    // previous goal positions to compare with the current, used for updating belief
	std::vector<geometry_msgs::PoseStamped> pre_positions;

    // obtained local goal_list
    geometry_msgs::PoseArray goal_list;

    // vector of points for belief update
	std::vector<geometry_msgs::Point> belief_points;

    // the proability distribution of different goals
	std::vector<float> belief_goal;

    // the instant proability distribution of different goals
	std::vector<float> instant_goal;

    // the instant goal index
    int instant_index = 0;

    // if the instant joystick input is inside collision zone
	bool dummy_goal_collision = false;

    // if the short instant joystick input is inside collision zone
    bool projected_cmd_collision = false;

    int stuck_count = 0;
    
    // most likely path index
	std_msgs::UInt32 most_likely_path;

	// user control weight
	// std_msgs::Float32 user_control_weight;

    // user adaptability index

    float adapt_idx = 0.1;

    // final goal
    geometry_msgs::Pose final_goal;

    // image to store the local costmap
    cv::Mat local_costmap;
    bool costmap_receive = false;
    float map_resolution;
    // current agent pose
    geometry_msgs::Pose map_pose;

    // frame_ids
    std::string base_frame_id = "base_link", path_frame_id = "map", odom_frame_id = "odom";

    // footprint for Rviz
    geometry_msgs::PolygonStamped footprint_polygon;

    // the size and center point of the rotation rectangle is set by current linear velocity

    float rotate_rect_length = 0, rotate_rect_width = 0, rect_center_x = 0, rect_center_y = 0;

    // the quaternion to represent the rotation of the costmap with respect to the wheelchair local frame
	tf2::Quaternion map_quaternion;

    // the yaw between wheelchair heading and the costmap pose
	float agent2map_yaw = 0;

    // center cell
	int x_center = 0, y_center = 0;

    // the clearance index
    float user_weight = 1.0;
    bool weight_receive = false;

    // the rollout time per step during tree search
    // float transition_time = 0.3;

    float pdwa_v = 0, pdwa_w = 0;
    bool pdwa_receive = false;

    float bdwa_v = 0, bdwa_w = 0;
    bool bdwa_receive = false;

    std::string planner_type = "POMDP";

    // user's instantaneous desired path
	nav_msgs::Path user_path;

    // callback for lidar scan data
    // void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);

    // // callback for odometry
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);

    // callback for joystick input
    void joystickCallback(const geometry_msgs::Point::ConstPtr &msg_point);

    // callback for goal list from path
    void pathCallback(const voronoi_msgs_and_types::PathList &msg_path);

    // callback for the local costmap
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map);

    // callback for pure DWA
    void pdwaCallback(const geometry_msgs::Twist::ConstPtr &msg_pdwa);

    // callback for belief DWA
    void bdwaCallback(const geometry_msgs::Twist::ConstPtr &msg_bdwa);

    // callback for control weight
    void weightCallback(const std_msgs::Float32::ConstPtr &msg_weight);

    // callback for update timer
    void updatetimerCallback(const ros::TimerEvent&);

    // callback for publish timer
    void publishtimerCallback(const ros::TimerEvent&);
    
    // generate short-term goals from paths for belief updating
    void generateGoal();

    // update the internal model info
    void updateModelInfo();

    // generate user's instantaneous desired path
    void generateUserPath();

    // compute the angle discrepancy for instant goal
    float instantRotationValue(float x_input, float y_input, geometry_msgs::Point goal_point) const;

    // find a goal along a path
    // geometry_msgs::Pose findGoal(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path);

    // find intermediate goals along a path
    nav_msgs::Path findIntermediateGoals(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path);

    //Get agent pose using TF
    geometry_msgs::PoseStamped getGlobalAgentPose();

    // the function to read the config file
    std::string ReadConfigFile();

    void PrintBelief();

    // Establish connection with the external system
    virtual bool Connect();

    // Initialize or reset the environment
    // return start state if applicable
    virtual State* Initialize();

    // Get the state of the system 
    virtual State* GetCurrentState() const;

    // TODO: add print state
    virtual void PrintState(const State& s, std::ostream& out) const;

    //Used to print trajectories for experimenting with dmps
    void printGoals();

    //Used to update weights of the dmps
    // void update_DMP_weights();

    // Send action to be executed by the system, 
    // receive observations terminal signals from the system
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    // Reaching check
    bool CheckDestination(const geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose);

    // Collision check
    float CheckCollision();

    // some calculation functions
    float calDistance(float x1, float y1, float x2, float y2);
};  //namespace despot

#endif