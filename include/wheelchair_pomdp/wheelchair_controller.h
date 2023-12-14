#ifndef WHEELCHAIR_CONTROLLER_H_
#define WHEELCHAIR_CONTROLLER_H_

#include "wheelchair_pomdp/wheelchair_model.h"
#include "wheelchair_pomdp/wheelchair_gazebo.h"
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>

#include <wheelchair_pomdp/GetCurrentStatus.h>
#include <wheelchair_pomdp/WheelchairActionObs.h>

class WheelchairController{
    public:
        WheelchairController();
        ~WheelchairController();
        void start();
        void stop();
        void initialization();

    private:
        ros::NodeHandlePtr node_handle_;
        ros::AsyncSpinner spinner;

        //topics
        ros::Subscriber joystick_subscriber_;

        // services
        ros::ServiceClient getStatusClient;
        ros::ServiceServer wheelchair_status_srv_;
        ros::ServiceServer control_srv_;

        // action
        // actionlib::SimpleActionClient<xarm_gripper::MoveAction>* gripper_client_ptr_;

        // // MoveIt! Interfaces
        // moveit::planning_interface::MoveGroupInterface *arm_group;
        // // moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

        WheelchairStruct current_status;
        // geometry_msgs::PoseStamped current_pose;

        bool get_current_status(wheelchair_pomdp::GetCurrentStatus::Request &req, wheelchair_pomdp::GetCurrentStatus::Response &res);
        bool wheelchair_action_obs(wheelchair_pomdp::WheelchairActionObs::Request &req, wheelchair_pomdp::WheelchairActionObs::Response &res);

        void joystickCallback(const geometry_msgs::Point &msg_point);
        

        // xarm_gripper::MoveGoal gripper_goal;



        //for arm_action_obs
        enum ACTIONS
        {
            LINEAR_PLUS = 0,
            LINEAR_MINUS = 1,
            ANGULAR_PLUS = 2,
            ANGULAR_MINUS = 3,
            ASK = 4,
            KEEP = 5
        };

        std::vector<float> robot_obs_;
        std::vector<float> joystick_obs_;

        double step_x, step_y, step_z;
        std::vector<int> robot_step;
};

#endif //XARM_INTERFACE_H_