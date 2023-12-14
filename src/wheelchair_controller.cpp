#include "wheelchair_pomdp/wheelchair_controller.h"

const double STEP_SIZE = 0.02;     // 2 cm
const int LARGE_STEP = 8;
const int SPINNER_THREAD_NUM = 2;

// For Cartesian path computation
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

WheelchairController::WheelchairController():
    spinner(SPINNER_THREAD_NUM)
{
    initialization();
}

WheelchairController::~WheelchairController(){}

void WheelchairController::initialization()
{
    node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle());

    // ROS spinning must be running for the MoveGroupInterface to get info about
    // the robot state
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    // MoveIt! Interface
    // arm_group = new moveit::planning_interface::MoveGroupInterface("xarm6");
    // planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

    // topics
    joystick_subscriber_ = node_handle_->subscribe("/arduino/joystick", 1, &WheelchairController::joystickCallback, this);

    // services
    wheelchair_status_srv_ = node_handle_->advertiseService("get_current_status", &WheelchairController::get_current_status, this);
    control_srv_ = node_handle_->advertiseService("wheelchair_action_obs", &WheelchairController::wheelchair_action_obs, this);
    getStatusClient = node_handle_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // action client
    // gripper_client_ptr_ = new actionlib::SimpleActionClient<xarm_gripper::MoveAction>("xarm/gripper_move", true);
    // gripper_client_ptr_->waitForServer();
    ROS_INFO_STREAM("Wheelchair action server started");
}

void WheelchairController::start()
{
    ROS_INFO_STREAM("Spinning");
    spinner.start();
    // Initialise arm 
    // arm_group->startStateMonitor();
    // std::cout << "End effector link: " << arm_group->getEndEffectorLink() << std::endl;
    // std::cout << "Current state: " << arm_group->getCurrentPose() << std::endl;
}

void WheelchairController::stop()
{
    spinner.stop();
}

bool WheelchairController::get_current_status(wheelchair_pomdp::GetCurrentStatus::Request &req, wheelchair_pomdp::GetCurrentStatus::Response &res)
{
    // ROS_INFO_STREAM("GetCurrentStatus service request received");
    // res.wheelchair_pose  = arm_group->getCurrentPose();
    // std::cout << res.pose << std::endl;
    // res.success = true;
    return true;
}

/* ArmActionObs server */
// Given robot action (request), what is the observation (response)
bool WheelchairController::wheelchair_action_obs(wheelchair_pomdp::WheelchairActionObs::Request &req, wheelchair_pomdp::WheelchairActionObs::Response &res)
{
//     // default staus
//     res.grasp_success = false;

//     // check if action is valid
//     if (req.action < 0 || req.action > 5){
//         ROS_ERROR("Invalid action integer. Valid actions (0-5).");
//         return false;
//     }

//     //  Action = "GRASP"
//     if (req.action == GRASP){         
//         res.grasp_success = true;

//         // send request to gripper action server
//         gripper_goal.target_pulse = 500;    // cannot close to 0 in simulation 
//         gripper_goal.pulse_speed = 1500;
//         gripper_client_ptr_->sendGoal(gripper_goal);

//         // set timeout for gripper closing to 5 seconds
//         if (gripper_client_ptr_->waitForResult(ros::Duration(5.0))){
//             actionlib::SimpleClientGoalState state = gripper_client_ptr_->getState();
//             ROS_INFO_STREAM("Gripper closing: %s", state.toString().c_str());
//         }
//         else{
//             ROS_INFO_STREAM("Gripper action did not finish before the time out.");
//             gripper_client_ptr_->cancelAllGoals();
//         }
        
//         // don't allow y inputs anymore
//         res.observation.clear();
//         res.observation.push_back(0);
//         res.observation.push_back(0);
//         res.gripper_pose = current_pose;
//         std::cout << "------------------GRASPED------------------" << std::endl;
//         return true;
//     }

//     /* Calculate Z-step (POMDP action) */
//     if (req.action == UP_LARGE){  
//         step_z = LARGE_STEP;
//     }
//     else if (req.action == DOWN_LARGE){ 
//         step_z = -LARGE_STEP;
//     }
//     else if (req.action == UP_SMALL){ 
//         step_z = 1;
//     }
//     else if (req.action == DOWN_SMALL){ 
//         step_z = -1;
//     }
//     else {
//         step_z = 0;
//     }

//     current_pose = arm_group->getCurrentPose();
//     current_pose.pose.position.z += STEP_SIZE * (step_z);

//     /* Calculate X-step and Y-step (POMDP observation) */
//     // store in another variable since joystick_obs will keep streaming in
//     if (joystick_obs_.size() != 2){
//         ROS_WARN_STREAM("Joystick observation has incorrect number of elements. Assuming no input");
//         robot_obs_.clear();
//         robot_obs_.push_back(0);
//         robot_obs_.push_back(0);
//     }
//     else{
//         robot_obs_ = joystick_obs_;
//     }

//     // Joystick obs stores the steps
//     step_x = robot_obs_[0];
//     step_y = robot_obs_[1];

//     auto temp_pos_y = current_pose.pose.position.y + (STEP_SIZE * step_y);
//     auto temp_pos_x = current_pose.pose.position.x + (STEP_SIZE * step_x);


//     // check if out of bounds
//     if (temp_pos_y > 0.5 || temp_pos_y < -0.5){
//         std::cout << "[WARN] Exceeded arm workspace, y: " << temp_pos_y << std::endl;
//         step_y = 0;
//         robot_obs_[1] = 0;
//     }
//     else {
//         current_pose.pose.position.y = temp_pos_y;   
//         // std::cout << "New pose: " << current_pose << std::endl; 
//     }

//     if (temp_pos_x > 0.8){
//         std::cout << "[WARN] Exceeded arm workspace, x: " << temp_pos_x << std::endl;
//         step_x = 0;
//         robot_obs_[0] = 0;
//     }
//     else {
//         current_pose.pose.position.x = temp_pos_x;
//     }
    
//     arm_group->setMaxVelocityScalingFactor(0.7);
//     // arm_group->setStartStateToCurrentState();
//     arm_group->setPoseTarget(current_pose);
//     std::cout << "Step X (user) : " << step_x << " | Step Y (user): " << step_y << " | Step Z (arm): " << step_z << std::endl;

//     int error_code = arm_group->move().val;
//     // arm_group->move() - blocking | asyncMove() - non-blocking
    
//     if (error_code == moveit_msgs::MoveItErrorCodes::PREEMPTED)
//     {
//         ROS_INFO("Preempted while taking action.");
//     }
//     else if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
//     {
//         ROS_INFO("Failed to take action.");
//         return false;
//     }
    

//     std::cout << "[xarm_interface] Joystick input (observation), x: " << robot_obs_[0] << " y: " << robot_obs_[1] << std::endl;
//     std::cout << "-----------------------------------------" << std::endl;

//     res.observation.clear();
//     res.observation.push_back(step_x);
//     res.observation.push_back(step_y);
//     res.gripper_pose = current_pose;

    return true;
}


// To get observation from joystick 
void WheelchairController::joystickCallback(const geometry_msgs::Point &msg_point)
{
    // double x_axis, y_axis;

    // x_axis = joy.axes[1];
    // y_axis = joy.axes[0];

    // // ROS_INFO("Joystick axis value: %.3f", axis_value);
    // // ROS_INFO("Time: %d.%d", joy.header.stamp.sec, joy.header.stamp.nsec);

    // joystick_obs_.clear();

    // // Discretising x
    // if (x_axis > 0.8){  // front
    //     joystick_obs_.push_back(LARGE_STEP);        
    // }
    // else if (x_axis > 0){
    //     joystick_obs_.push_back(1);
    // }
    // else if (x_axis < -0.8){ // back
    //     joystick_obs_.push_back(-LARGE_STEP);      
    // }
    // else if (x_axis < 0){
    //     joystick_obs_.push_back(-1);
    // }
    // else
    //     joystick_obs_.push_back(0);  

    // // Discretising y
    // if (y_axis > 0.8){  // left
    //     joystick_obs_.push_back(LARGE_STEP);        
    // }
    // else if (y_axis > 0){
    //     joystick_obs_.push_back(1);
    // }
    // else if (y_axis < -0.8){  // right
    //     joystick_obs_.push_back(-LARGE_STEP);      
    // }
    // else if (y_axis < 0){
    //     joystick_obs_.push_back(-1);
    // }
    // else
    //     joystick_obs_.push_back(0);  


    // // To open gripper 
    // // if using virtual joystick, press 'Q'
    // if (joy.buttons[0] == 1){
    //     // send request to gripper action server
    //     gripper_goal.target_pulse = 850;    // cannot close to 0 in simulation 
    //     gripper_goal.pulse_speed = 1500;
    //     gripper_client_ptr_->sendGoal(gripper_goal);

    //     // set timeout for gripper closing to 5 seconds
    //     if (gripper_client_ptr_->waitForResult(ros::Duration(5.0))){
    //         actionlib::SimpleClientGoalState state = gripper_client_ptr_->getState();
    //         ROS_INFO_STREAM("Gripper opening: %s", state.toString().c_str());
    //     }
    //     else{
    //         ROS_INFO_STREAM("Gripper action did not finish before the time out.");
    //         gripper_client_ptr_->cancelAllGoals();
    //     }
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheelchair_controller");
    
    WheelchairController wheelchair_controller;

    wheelchair_controller.start();

    // needed since AsyncSpinner is not operating in the same thread
    ros::waitForShutdown();
    return 0;
}