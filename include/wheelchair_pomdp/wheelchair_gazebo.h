#ifndef WHEELCHAIR_GAZEBO_H
#define WHEELCHAIR_GAZEBO_H

#include "wheelchair_model.h"

#include <despot/interface/world.h>
// #include <despot/util/coord.h>
#include <ros/ros.h>

// #include <chrono>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
// #include <gazebo_msgs/ModelState.h>
// #include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseArray.h>

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
// #include <opencv2/opencv.hpp>

// Custom Messages
#include <voronoi_msgs_and_types/PathList.h>
#include "wheelchair_dmp.h"

// #include "wheelchair_pomdp/plot_scan.hpp"

using namespace despot;

class GazeboWheelchairWorld: public World
{
public:
    ros::NodeHandlePtr node_handle_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber path_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber joystick_subscriber_;
    ros::Subscriber goal_subscriber_;
    ros::Publisher vel_publisher_;
    ros::Publisher ucmd_publisher_;
    ros::Publisher pomdp_publisher_;
    // ros::Publisher wypt_publisher_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    visualization_msgs::Marker cmd_line;
    visualization_msgs::Marker pomdp_line;
    
    WheelchairState* wheelchair_state_;
    WheelchairDSPOMDP* wheelchair_model_;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf_listener;
    // geometry_msgs::TransformStamped lidarTransform;

    GazeboWheelchairWorld(WheelchairDSPOMDP* model_);
    virtual ~GazeboWheelchairWorld();
    
    WheelchairStruct initial_wheelchair_local_status;

    // obtained global goal_list
    // geometry_msgs::PoseArray goal_list;

    // // obtained local goal_list
    // geometry_msgs::PoseArray local_goal_list;

    geometry_msgs::TransformStamped lidar2baseTransform;

    float tf_buffer_timeout = 0.3;

    float waypoint_dist = 2.0;

    int step_no = 0; //Used to track step count to generate the filenames for printing intermediate goals
    

    // std::chrono::time_point<std::chrono::system_clock> start_time;

    // std::chrono::time_point<std::chrono::system_clock> exe_time;

    // ros::Time exe_time_ros;

    // visualization_msgs::Marker waypoint_viz;

    // lidar info
    // int img_count = 0;
    // plot_scan::PlotScan plot_scan_;
    // sensor_msgs::LaserScanConstPtr scan_pointer;
    std::vector<geometry_msgs::Point> lidar_points;
    std::vector<geometry_msgs::Point> lidar_points_temp;
    bool scan_receive = false;
    // joystick info
    geometry_msgs::Point joystick_input;
    bool joystick_receive = false;

    bool zero_joystick_input = false;

    // current velocity info
    geometry_msgs::Twist current_vel;
    bool odom_receive = false;

    // final published velocity
    geometry_msgs::Twist cmd_vel;

    // current agent pose
    geometry_msgs::Pose current_pose;

    // path info obtained from published paths
    voronoi_msgs_and_types::PathList path_list;
    bool path_receive = false;

    // previous published paths to check the change
    voronoi_msgs_and_types::PathList pre_path_list;

    // the intermediate goals
    voronoi_msgs_and_types::PathList intermediate_goals;

    // final goal
    geometry_msgs::Pose final_goal;

    // frame_ids
    std::string base_frame_id = "base_link", path_frame_id = "map", odom_frame_id = "odom";

    // the rollout time per step during tree search
    float transition_time = 0.3;

    // callback for lidar scan data
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan);

    // // callback for odometry
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom);

    // callback for joystick input
    void joystickCallback(const geometry_msgs::Point::ConstPtr &msg_point);

    // callback for goal list from path
    void pathCallback(const voronoi_msgs_and_types::PathList &msg_path);
    
    // generate short-term goals from paths for belief updating
    void generateGoal();

    // find a goal along a path
    geometry_msgs::Pose findGoal(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path);

    // find intermediate goals along a path
    nav_msgs::Path findIntermediateGoals(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path);

    //Get agent pose using TF
    geometry_msgs::PoseStamped getGlobalAgentPose();

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
    void update_DMP_weights();

    // Send action to be executed by the system, 
    // receive observations terminal signals from the system
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    // Reaching check
    bool CheckDestination(const geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose);

    // Collision check
    bool CheckCollision();

    // some calculation functions
    float calDistance(float x1, float y1, float x2, float y2);
};  //namespace despot

#endif