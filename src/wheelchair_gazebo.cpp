#include "wheelchair_pomdp/wheelchair_gazebo.h"

using namespace despot;
using namespace std;

GazeboWheelchairWorld::GazeboWheelchairWorld(WheelchairDSPOMDP* model_)
{
    // start_time = std::chrono::system_clock::now();
    // exe_time = std::chrono::system_clock::now();
    // exe_time_ros = ros::Time::now();
    initial_wheelchair_local_status.agent_pose.position.x = 0;
    initial_wheelchair_local_status.agent_pose.position.y = 0;
    initial_wheelchair_local_status.agent_pose.position.z = 0;
    initial_wheelchair_local_status.agent_pose.orientation.x = 0;
    initial_wheelchair_local_status.agent_pose.orientation.y = 0;
    initial_wheelchair_local_status.agent_pose.orientation.z = 0;
    initial_wheelchair_local_status.agent_pose.orientation.w = 1;
    initial_wheelchair_local_status.agent_velocity.linear.x = 0;
    initial_wheelchair_local_status.agent_velocity.angular.z = 0;
    //Store yaw for use later
	
	
	tf2::Quaternion agent_quat;
	tf2::convert(initial_wheelchair_local_status.agent_pose.orientation, agent_quat);
	initial_wheelchair_local_status.agent_pose_angle = tf2::getYaw(agent_quat);

    // Create a new state here
    wheelchair_state_ = new WheelchairState(initial_wheelchair_local_status, 0);
    wheelchair_model_ = model_;
    //wheelchair_model_->dmp_init_variables = DMP_init_variables(ModelParams::num_paths);
}

GazeboWheelchairWorld::~GazeboWheelchairWorld()
{}

bool GazeboWheelchairWorld::Connect()
{
    // initialise ROS node
    int argc = -1;
    char ** argv;

    ros::init(argc, argv, "wheelchair_pomdp");
    node_handle_ = ros::NodeHandlePtr(new ros::NodeHandle);
    spinner_.reset(new ros::AsyncSpinner(0));
    // exe_time_ros = ros::Time::now();
    // setup topics and service clients
    scan_subscriber_ = node_handle_->subscribe("/scan", 1, &GazeboWheelchairWorld::scanCallback, this);
    path_subscriber_ = node_handle_->subscribe("/move_base/SharedVoronoiGlobalPlanner/all_paths", 1, &GazeboWheelchairWorld::pathCallback, this);
    odom_subscriber_ = node_handle_->subscribe("/odom", 1, &GazeboWheelchairWorld::odomCallback, this);
    //joystick_subscriber_ = node_handle_->subscribe("/arduino/joystick", 1, &GazeboWheelchairWorld::joystickCallback, this);
    joystick_subscriber_ = node_handle_->subscribe("/keyboard", 1, &GazeboWheelchairWorld::joystickCallback, this);
    // wypt_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/waypoints", 1);
    // wheelchair_state_client_ = node_handle_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    vel_publisher_ = node_handle_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ucmd_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/cmd_path", 1);
    pomdp_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/pomdp_path", 1);

    // if (!node_handle_->getParam("transition_time", transition_time))
    //     ROS_WARN_STREAM("Parameter transition_time not set. Using default setting: " << transition_time);
    // if (!node_handle_->getParam("waypoint_dist", waypoint_dist))
    //     ROS_WARN_STREAM("Parameter waypoint_dist not set. Using default setting: " << waypoint_dist);

    transition_time = ModelParams::transition_time;
    waypoint_dist = ModelParams::waypoint_dist;

    // visualization markers
    // waypoint_viz.header.frame_id = base_frame_id;
    // waypoint_viz.ns = "visualization";
    // waypoint_viz.action = visualization_msgs::Marker::ADD;
    // waypoint_viz.pose.orientation.w = 1.0;
    // waypoint_viz.id = 1;
    // waypoint_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    // waypoint_viz.scale.x = 0.2;
    // waypoint_viz.scale.y = 0.2;
    // waypoint_viz.scale.z = 0.2;
    // waypoint_viz.color.r = 1.0;
    // waypoint_viz.color.g = 1.0;
    // waypoint_viz.color.a = 1.0;

    cmd_line.header.frame_id = base_frame_id;
    cmd_line.ns = "visualization";
    cmd_line.action = visualization_msgs::Marker::ADD;
    cmd_line.pose.orientation.w = 1.0;
    cmd_line.id = 1;
    cmd_line.type = visualization_msgs::Marker::LINE_STRIP;
    cmd_line.scale.x = 0.05;
    cmd_line.color.b = 1.0;
    cmd_line.color.a = 1.0;

    pomdp_line.header.frame_id = base_frame_id;
    pomdp_line.ns = "visualization";
    pomdp_line.action = visualization_msgs::Marker::ADD;
    pomdp_line.pose.orientation.w = 1.0;
    pomdp_line.id = 2;
    pomdp_line.type = visualization_msgs::Marker::LINE_STRIP;
    pomdp_line.scale.x = 0.05;
    pomdp_line.color.g = 1.0;
    pomdp_line.color.a = 1.0;

    // initialize tf_listener
    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    spinner_->start();

    ros::Duration(2.0).sleep();
    // ros::spinOnce();

    // // only receive messages in the first 3s.
	// while ((std::chrono::system_clock::now() - start_time).count() / 1000000000.0 <= 3)
    // {
	// 	ros::spinOnce();
	// }

	// ros::spin();

	return true;
}

// TODO
State* GazeboWheelchairWorld::Initialize()
{
    // ros::spinOnce();
    /**
     * Temporary initialization of values
     * should initialize based on gazebo objects
     */
    // ===========================================
    // Obtain goal states(poses) from Gazebo
    // ===========================================

    // Getting goal positions from simulation world
    // waypoint_positions.clear();

    // transform the goal positions into local frame
    GazeboWheelchairWorld::generateGoal();
    // wheelchair_state_->state_goal_point = wheelchair_model_->goal_positions;
    // wheelchair_state_->state_goal_index = wheelchair_model_->goal_indices;

    // Get wheelchair status (velocity)
    if (!odom_receive)
    {
        ROS_ERROR_STREAM("Failed to receive odom message");
    }
    else
    {
        // must update both model and state classes
        // only update the velocities because wheelchair model is using the local frame
        wheelchair_model_->current_wheelchair_status.agent_velocity = current_vel;
        wheelchair_state_->wheelchair.agent_velocity = current_vel;
    }

    // Pass the lidar info to WheelchairDSPOMDP model
    if (!scan_receive)
    {
        ROS_ERROR_STREAM("Lidar info not received. Initialization failed...");
    }
    else
    {
        wheelchair_model_->lidar_points = lidar_points;
    }

    // Pass the joystick input to WheelchairDSPOMDP model
    if (!joystick_receive)
    {
        ROS_ERROR_STREAM("Joystick input not received.");
    }
    else
    {
        wheelchair_model_->external_joy_x = joystick_input.x;
        wheelchair_model_->external_joy_y = joystick_input.y;
        wheelchair_state_->joystick_x = joystick_input.x;
        wheelchair_state_->joystick_y = joystick_input.y;
    }

    // Pass the transition_time to WheelchairDSPOMDP model
    wheelchair_model_->transition_time = transition_time;

    // ==============================================


    /**
     *  Initializing possible goal positions in the POMDP model 
     */

    // check that number of goals in the model and in the world is the same
    #ifdef DEBUG
    #endif
    if (wheelchair_model_->goal_positions.size() != intermediate_goals.paths.size())
        ROS_WARN_STREAM("Number of possible goals in the world and the model is different!");
    else
        std::cout << "Number of possible goals: " << wheelchair_model_->goal_positions.size() << std::endl;

    // std::cout << "Goal positions: " << std::endl;

    // std::vector<double> dist;

    // for (int i = 0; i < size; i ++)
    // {
    //     wheelchair_model_->goal_positions[i] = waypoint_positions[i];
    //     std::cout << "Goal " << i+1 << ": " << waypoint_positions[i].x << " " << waypoint_positions[i].y << " " << waypoint_positions[i].z << std::endl;
    //     double d = pow(fabs(wheelchair_model_->wheelchair_status.agent_pose.position.x - waypoint_positions[i].x), 2);
    //     d += pow(fabs(wheelchair_model_->wheelchair_status.agent_pose.position.y - waypoint_positions[i].y), 2);
    //     d = sqrt(d);
    //     dist.push_back(d);
    // }

    // Pick left door as the initial goal and udpate wheelchair state
    std::srand(std::time(0));
    wheelchair_state_->path_idx = std::rand() % intermediate_goals.paths.size();

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    // GetCurrentState();
    std::cout << "========================= Intialized world =============================" << std::endl;

    return wheelchair_state_;
}

State* GazeboWheelchairWorld::GetCurrentState() const
{
    // ros::spinOnce();
    if (wheelchair_state_ == NULL)
    {
        std::cout << "State is NULL" << std::endl;
    }
    // std::cout << arm_state_->text() << std::endl;
    return wheelchair_state_;
}

// Send action to be executed by the system, receive obervations from system
bool GazeboWheelchairWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs)
{
    #ifdef DEBUG
    // float time_elapsed = (std::chrono::system_clock::now() - exe_time).count() / 1000000000.0;
    // cout << "ExecuteAction called in " << time_elapsed << "s in reality." << endl;
    // exe_time = std::chrono::system_clock::now();

    // float time_elapsed_ros = (ros::Time::now() - exe_time_ros).toSec();
    // cout << "ExecuteAction called in " << time_elapsed_ros << "s in ROS." << endl;
    // exe_time_ros = ros::Time::now();
    #endif
    // Stop the wheelchair if there's no joystick input or the planning step reaches the max number
    if (zero_joystick_input || (action == -1))
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        vel_publisher_.publish(cmd_vel);
        ucmd_publisher_.publish(cmd_line);
        pomdp_publisher_.publish(pomdp_line);
        cmd_line.points.clear();
        pomdp_line.points.clear();
        return false;
    }

    // ros::spinOnce();
    // Clear hashmap from previous round first
    wheelchair_model_->obsHashMap.clear();
    bool terminate_execution = false;
    WheelchairObs wheelchair_external_obs;
    wheelchair_external_obs.continuous_obs = true;
    std::ostringstream obs_string;
    // Update wheelchair status (velocity)
    if (!odom_receive)
    {
        ROS_ERROR_STREAM("Failed to receive odom message");
        return 1; // exit
    }
    else
    {
        current_vel.linear.x = round(current_vel.linear.x * 100) / 100;
        current_vel.angular.z = round(current_vel.angular.z * 100) / 100;
    }
    // if (action == WheelchairDSPOMDP::ASK)
    // {
    //     // stop and ask question
    //     cmd_vel.linear.x = 0;
    //     cmd_vel.angular.z = 0;
    // }
    // cout << "Before sending command, current linear v = " << current_vel.linear.x << ", angular w = " << current_vel.angular.z << endl;
    // cout << "Before sending command, command linear v = " << cmd_vel.linear.x << ", angular w = " << cmd_vel.angular.z << endl;
    // if (fabs(current_vel.linear.x - cmd_vel.linear.x) >= 0.2)
    // {
    //     cmd_vel.linear.x = current_vel.linear.x;
    // }
    // if (fabs(current_vel.angular.z - cmd_vel.angular.z) >= 0.2)
    // {
    //     cmd_vel.angular.z = current_vel.angular.z;
    // }
    if (action == WheelchairDSPOMDP::LINEAR_PLUS)
    {
        // linear acceleration
        if (current_vel.linear.x >= 1)
        {
            cmd_vel.linear.x = current_vel.linear.x;
        }
        else if (current_vel.linear.x < -0.1 && current_vel.linear.x > -0.2)
        {
            cmd_vel.linear.x = 0;
        }
        else
        {
            // cmd_vel.linear.x += 0.2;
            cmd_vel.linear.x = current_vel.linear.x + 0.2;
        }
        // cmd_vel.linear.x = (current_vel.linear.x >= 1) ? current_vel.linear.x : cmd_vel.linear.x + 0.2;
        // cmd_vel.linear.x = (current_vel.linear.x < -0.1 && current_vel.linear.x > -0.2) ? 0.0 : cmd_vel.linear.x + 0.2;
        // cmd_vel.angular.z = current_vel.angular.z;
    }
    else if (action == WheelchairDSPOMDP::LINEAR_MINUS)
    {
        // linear deceleration
        if (current_vel.linear.x <= -1)
        {
            cmd_vel.linear.x = current_vel.linear.x;
        }
        else if (current_vel.linear.x > 0.1 && current_vel.linear.x < 0.2)
        {
            cmd_vel.linear.x = 0;
        }
        else
        {
            // cmd_vel.linear.x -= 0.2;
            cmd_vel.linear.x = current_vel.linear.x - 0.2;
        }
        // cmd_vel.linear.x = (current_vel.linear.x <= -1) ? current_vel.linear.x : cmd_vel.linear.x - 0.2;
        // cmd_vel.linear.x = (current_vel.linear.x > 0.1 && current_vel.linear.x < 0.2) ? 0.0 : cmd_vel.linear.x - 0.2;
        // cmd_vel.angular.z = current_vel.angular.z;
    }
    else if (action == WheelchairDSPOMDP::ANGULAR_PLUS)
    {
        // angular acceleration
        if (current_vel.angular.z >= 1)
        {
            cmd_vel.angular.z = current_vel.angular.z;
        }
        else if (current_vel.angular.z < -0.1 && current_vel.angular.z > -0.5)
        {
            cmd_vel.angular.z = 0;
        }
        else
        {
            // cmd_vel.angular.z += 0.2;
            cmd_vel.angular.z = current_vel.angular.z + 0.5;
        }
        // cmd_vel.linear.x = current_vel.linear.x;
        // cmd_vel.angular.z = (current_vel.angular.z >= 1) ? current_vel.angular.z : cmd_vel.angular.z + 0.2;
        // cmd_vel.angular.z = (current_vel.angular.z < -0.1 && current_vel.angular.z > -0.2) ? 0.0 : cmd_vel.angular.z + 0.2;
    }
    else if (action == WheelchairDSPOMDP::ANGULAR_MINUS)
    {
        // angular deceleration
        if (current_vel.angular.z <= -1)
        {
            cmd_vel.angular.z = current_vel.angular.z;
        }
        else if (current_vel.angular.z > 0.1 && current_vel.angular.z < 0.5)
        {
            cmd_vel.angular.z = 0;
        }
        else
        {
            // cmd_vel.angular.z -= 0.2;
            cmd_vel.angular.z = current_vel.angular.z - 0.5;
        }
        // cmd_vel.linear.x = current_vel.linear.x;
        // cmd_vel.angular.z = (current_vel.angular.z <= -1) ? current_vel.angular.z : cmd_vel.angular.z - 0.2;
        // cmd_vel.angular.z = (current_vel.angular.z > 0.1 && current_vel.angular.z < 0.2) ? 0.0 : cmd_vel.angular.z - 0.2;
    }
    else if (action == WheelchairDSPOMDP::KEEP)
    {
        // keep current velocities
        cmd_vel.linear.x = current_vel.linear.x;
        cmd_vel.angular.z = current_vel.angular.z;
    }
    // execute the action
    #ifdef DEBUG
    #endif

    geometry_msgs::Point cmd_line_point, pomdp_line_point;
    float theta_cmd = 0, theta_pomdp = 0;

    for (int i = 0; i < 10; i++)
    {
        cmd_line_point.x += joystick_input.x * cos(theta_cmd) * ModelParams::transition_time;
        cmd_line_point.y += joystick_input.x * sin(theta_cmd) * ModelParams::transition_time;
        theta_cmd += joystick_input.y * ModelParams::transition_time;
        cmd_line.points.push_back(cmd_line_point);

        pomdp_line_point.x += cmd_vel.linear.x * cos(theta_pomdp) * ModelParams::transition_time;
        pomdp_line_point.y += cmd_vel.linear.x * sin(theta_pomdp) * ModelParams::transition_time;
        theta_pomdp += cmd_vel.angular.z * ModelParams::transition_time;
        pomdp_line.points.push_back(pomdp_line_point);
    }
    // cout << "Published Command: linear v: " << cmd_vel.linear.x << ", angular w: " << cmd_vel.angular.z << endl;
    vel_publisher_.publish(cmd_vel);
    ucmd_publisher_.publish(cmd_line);
    pomdp_publisher_.publish(pomdp_line);
    cmd_line.points.clear();
    pomdp_line.points.clear();

    //=================== Whether to wait for a few ms to execute the cmd_vel??? ======================//

    // ros::Duration(0.010).sleep();

    // ros::spinOnce();
    // Observation part
    
    wheelchair_external_obs.wheelchair_pose = initial_wheelchair_local_status.agent_pose;
    wheelchair_external_obs.agent_pose_angle = initial_wheelchair_local_status.agent_pose_angle; //Do we need to calculate pose angle from pose here?
    wheelchair_external_obs.wheelchair_twist = current_vel;

    // must update both model and state classes
    wheelchair_state_->wheelchair.agent_velocity = current_vel;
    wheelchair_model_->current_wheelchair_status.agent_velocity = current_vel;
    // cout << "Wheelchair position: x = " << wheelchairState.response.pose.position.x << "; y = " << wheelchairState.response.pose.position.y << endl;
    // cout << "Wheelchair orientation: x = " << wheelchairState.response.pose.orientation.x << "; y = " << wheelchairState.response.pose.orientation.y 
    //     << "; z = " << wheelchairState.response.pose.orientation.z << "; w = " << wheelchairState.response.pose.orientation.w << endl;
    // cout << "Wheelchair velocity: v = " << wheelchairState.response.twist.linear.x << "; w = " << wheelchairState.response.twist.angular.z << endl;
    // cout << "After sending command, current linear v = " << current_vel.linear.x << ", angular w = " << current_vel.angular.z << endl;
    // cout << "Goal door global position: x = " << waypoint_positions[wheelchair_state_->path_idx].x << "; y = " << waypoint_positions[wheelchair_state_->path_idx].y << endl;
    // reset the wheelchhair position in the wheelchair_model_ to the origin, so that it plans from the origin in the POMDP world
    // wheelchair_model_->wheelchair_status.agent_pose.position.x = 0;
    // wheelchair_model_->wheelchair_status.agent_pose.position.y = 0;
    // wheelchair_model_->wheelchair_status.agent_pose.orientation.x = 0;
    // wheelchair_model_->wheelchair_status.agent_pose.orientation.y = 0;
    // wheelchair_model_->wheelchair_status.agent_pose.orientation.z = 0;
    // wheelchair_model_->wheelchair_status.agent_pose.orientation.w = 1;
    // wheelchair_state_->wheelchair.agent_pose.position.x = 0;
    // wheelchair_state_->wheelchair.agent_pose.position.y = 0;
    // wheelchair_state_->wheelchair.agent_pose.orientation.x = 0;
    // wheelchair_state_->wheelchair.agent_pose.orientation.y = 0;
    // wheelchair_state_->wheelchair.agent_pose.orientation.z = 0;
    // wheelchair_state_->wheelchair.agent_pose.orientation.w = 1;

    // update the goal positions in the local frame
    GazeboWheelchairWorld::generateGoal();
    // wheelchair_state_->state_goal_point = wheelchair_model_->goal_positions;
    // wheelchair_state_->state_goal_index = wheelchair_model_->goal_indices;

    // tf2::Quaternion wheelchair_quat;
	// tf2::convert(agent_pose.orientation, wheelchair_quat);
	// tf2Scalar roll, pitch, yaw;
	// tf2::Matrix3x3 wheelchair_matrix(wheelchair_quat);
	// wheelchair_matrix.getRPY(roll, pitch, yaw);

    // cout << "Agent position in global frame: x " << agent_pose.position.x << ", y " << agent_pose.position.y << ", theta " << yaw * 180 / M_PI << endl;

    // cout << "Goal position in global frame: x " << goal_list.poses[wheelchair_state_->path_idx].position.x << ", y " << goal_list.poses[wheelchair_state_->path_idx].position.y << endl;

    // cout << "Goal position in local frame: x " << wheelchair_model_->goal_positions[wheelchair_state_->path_idx].point.x << ", y " << wheelchair_model_->goal_positions[wheelchair_state_->path_idx].point.y << endl;

    if (CheckDestination(current_pose, final_goal))
    {
        // terminate when the goal is reached
        cout << "Final goal has been reached, the task will be terminated soon!" << endl;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        vel_publisher_.publish(cmd_vel);
        // cout << "Final goal: x = " << final_goal.position.x << ",y = " << final_goal.position.y << endl;
        return true;
    }

    if (CheckCollision())
    {
        // terminate when a collision happens
        cout << "A collision has happened, the task will be terminated!" << endl;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        vel_publisher_.publish(cmd_vel);
        // cout << "Final goal: x = " << final_goal.position.x << ",y = " << final_goal.position.y << endl;
        return true;
    }

    // observation
    // if ask question
    // if (action == WheelchairDSPOMDP::ASK)
    // {
    //     wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_NONE; // obs = NONE

    //     ROS_INFO_STREAM("Asking question...");
    //     // continue
    // }
    // other actions
    // else
    // {
    /**
     * ====================== UPDATE WORLD STATE ======================
     * World states and model states are different!
     * POMDP model doesn't take the world state for calculations, 
     * only takes the observations.
     * Need to update the world states separately if we want to track the
     * states in the output log
     */
    if (!joystick_receive)
    {
        ROS_ERROR_STREAM("Joystick input not received.");
        return 1; // exit
    }
    wheelchair_model_->external_joy_x = joystick_input.x;
    wheelchair_model_->external_joy_y = joystick_input.y;
    wheelchair_external_obs.joystick_signal.x = joystick_input.x;
    wheelchair_external_obs.joystick_signal.y = joystick_input.y;
    wheelchair_external_obs.joystick_signal.z = joystick_input.z;
    // if (joystick_input.x == 0 && joystick_input.y == 0)
    // {
    //     // cout << "zero input" << endl;
    //     wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_NONE;
    // }
    // else
    // {
    //     // cout << "non-zero input" << endl;
    //     tf2::Vector3 heading_direction(1, 0, 0);
    //     tf2::Vector3 joystick_direction(joystick_input.x, joystick_input.y, 0);
    //     float angle_diff = heading_direction.angle(joystick_direction);
    //     float cross_product = heading_direction.getX() * joystick_direction.getY() - heading_direction.getY() * joystick_direction.getX();
    //     angle_diff = (cross_product >= 0)? angle_diff : -angle_diff;

    //     if (angle_diff <= -0.375 * M_PI)
    //     {
    //         wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_RIGHT;
    //     }
    //     else if (angle_diff > -0.375 * M_PI && angle_diff <= -0.125 * M_PI)
    //     {
    //         wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_FR_R;
    //     }
    //     else if (angle_diff > -0.125 * M_PI && angle_diff < 0.125 * M_PI)
    //     {
    //         wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_FRONT;
    //     }
    //     else if (angle_diff >= 0.125 * M_PI && angle_diff < 0.375 * M_PI)
    //     {
    //         wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_FR_L;
    //     }
    //     else		// angle_diff >= 0.375 * M_PI
    //     {
    //         wheelchair_external_obs.joystick_obs = WheelchairDSPOMDP::OBS_LEFT;
    //     }
    // }
    // continue
    // }

    wheelchair_model_->PrintObs(wheelchair_external_obs, obs_string);
    // takes printed string to obs_string and converts to a hash value
    uint64_t hashValue = wheelchair_model_->obsHash(obs_string.str());
    #ifdef DEBUG
    std::cout << "Hash value: " << hashValue << std::endl;
    #endif
    // takes hashValue and updates value of obs in ObservationClass
    wheelchair_external_obs.SetIntObs(hashValue);

    // Updates hashmap:
    // takes hash value as key and stores wheelchair_obs as value in map
    // First check if hash value is already stored
    if (wheelchair_model_->obsHashMap_execution.find(hashValue) == wheelchair_model_->obsHashMap_execution.end())
    {
        wheelchair_model_->obsHashMap_execution[hashValue] = wheelchair_external_obs;
    }

    // if (wheelchair_model_->obsHashMap_execution.find(hashValue) == wheelchair_model_->obsHashMap_execution.end())
    // {
    //     ROS_WARN_STREAM("Hash value not found.");
    // }
    // else
    // {
    //     ROS_WARN_STREAM("Hash value found.");
    // }
    obs = (OBS_TYPE) hashValue;

    // =================================================== 

    // Check if state is updated
    #ifdef DEBUG
    #endif
    // std::cout << "Execute Action: \n\t";
    // std::cout << wheelchair_state_->text() << std::endl; 
    // update the lidar info
    wheelchair_model_->lidar_points = lidar_points;


    // cv::Mat scan_img;

    // plot_scan_.drawScan(scan_pointer, scan_img);

    // std::string filename = "/home/ray/DESPOT-LOG/" + std::to_string(img_count) + ".jpg";
    // cv::imwrite(filename, scan_img);

    // img_count ++;

    if (Globals::config.useGPU)
    {
        wheelchair_model_->UpdateGPUModel();
    }
    return false;
}

void GazeboWheelchairWorld::PrintState(const State& state, ostream& out) const
{
	const WheelchairState& wheelchair_state = static_cast<const WheelchairState&>(state);
	out << "Wheelchair position: x = " << wheelchair_state.wheelchair.agent_pose.position.x << "; y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
	out << "Wheelchair orientation: x = " << wheelchair_state.wheelchair.agent_pose.orientation.x << "; y = " << wheelchair_state.wheelchair.agent_pose.orientation.y 
		<< "; z = " << wheelchair_state.wheelchair.agent_pose.orientation.z << "; w = " << wheelchair_state.wheelchair.agent_pose.orientation.w << endl;
	out << "Goal door global position: x = " << wheelchair_model_->goal_positions[wheelchair_state.path_idx].point.x << "; y = " << wheelchair_model_->goal_positions[wheelchair_state.path_idx].point.y << endl;
}

void GazeboWheelchairWorld::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
{
    //Get static transform from lidar to base_link in case they are not in the same frame
    if (lidar2baseTransform.header.frame_id != base_frame_id && msg_scan->header.frame_id != base_frame_id)
    {
        ROS_INFO("LIDAR is not in base link frame and transform has not been found yet, finding transform");
        try
        {
            lidar2baseTransform = tf_buffer.lookupTransform(base_frame_id, msg_scan->header.frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
            ROS_INFO("Transform found, all future scans received by shared_dwa will be transformed before being used for collision checking");
        }
        catch (tf2::TransformException &Exception)
        {
            ROS_ERROR("LIDAR tranform could not be found, shared_dwa may be incorrect");
            ROS_ERROR_STREAM(Exception.what());
        }
    }
    // scan_pointer = msg_scan;
    lidar_points_temp.clear();
    lidar_points_temp.reserve(msg_scan->ranges.size());
    for (int k = 0; k < msg_scan->ranges.size(); ++k)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = msg_scan->ranges[k] * cos(msg_scan->angle_min + k * msg_scan->angle_increment);
        temp_point.y = msg_scan->ranges[k] * sin(msg_scan->angle_min + k * msg_scan->angle_increment);

        //If transform header is not empty
        if (!lidar2baseTransform.header.frame_id.empty())
            tf2::doTransform<geometry_msgs::Point>(temp_point, temp_point, lidar2baseTransform);

        if (fabs(temp_point.x) + fabs(temp_point.y) <= 0.4)
        {
            cout << "temp_point.x " << temp_point.x << ", temp_point.y " << temp_point.y << endl;
            cout << "range " << msg_scan->ranges[k] << endl;
        }
		else if (sqrtf(powf(temp_point.x, 2) + powf(temp_point.y, 2)) <= 0.4)
        {
            cout << "temp_point.x " << temp_point.x << ", temp_point.y " << temp_point.y << endl;
            cout << "range " << msg_scan->ranges[k] << endl;
        }
        lidar_points_temp.emplace_back(std::move(temp_point));
    }
    lidar_points = lidar_points_temp;
    scan_receive = true;
}

void GazeboWheelchairWorld::joystickCallback(const geometry_msgs::Point::ConstPtr &msg_point)
{
    joystick_input.x = round(msg_point->x * 100) / 100;
    joystick_input.y = round(msg_point->y * 100) / 100;
    joystick_input.z = 0;

    joystick_input.x = (fabs(joystick_input.x) >= 0.05) ? joystick_input.x : 0;
    joystick_input.y = (fabs(joystick_input.y) >= 0.05) ? joystick_input.y : 0;

    if (joystick_input.x == 0 && joystick_input.y == 0)
    {
        zero_joystick_input = true;
    }
    else
    {
        zero_joystick_input = false;
    }
    joystick_receive = true;
}

void GazeboWheelchairWorld::odomCallback(const nav_msgs::Odometry::ConstPtr &msg_odom)
{
    current_vel = msg_odom->twist.twist;
    odom_receive = true;
}

bool GazeboWheelchairWorld::CheckDestination(const geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
{
    float dist2goal = 0;
	dist2goal = sqrt(powf((current_pose.position.x - goal_pose.position.x), 2) + powf((current_pose.position.y - goal_pose.position.y), 2));
	if (dist2goal <= 0.3)
		return true;
	else
		return false;
}

bool GazeboWheelchairWorld::CheckCollision()
{
	float r_collision = ModelParams::inner_radius - 0.2;

	for (const auto &point : lidar_points)
	{
		//https://stackoverflow.com/a/7227057
		//Check through easy conditions first, getting distance with sqrt is computationally expensive
		double dx = fabs(point.x);
		double dy = fabs(point.y);

		if (dx > r_collision || dy > r_collision)
			continue;

		if (dx + dy <= r_collision)
        {
            cout << "dx " << dx << ", dy " << dy << endl;
            return true;
        }
			
		else if (sqrtf(powf(dx, 2) + powf(dy, 2)) <= r_collision)
        {
            cout << "dx " << dx << ", dy " << dy << endl;
            return true;
        }
			
	}
	return false;
}

void GazeboWheelchairWorld::pathCallback(const voronoi_msgs_and_types::PathList &msg_path)
{
    path_list = msg_path;
    // check if the paths exist or have changed
    if (path_list.paths.size() == 0)
    {
        ROS_ERROR_STREAM("Path candidates not received!");
        path_receive = false;
        return;
    }
    else
    {
        path_receive = true;
    }
}

//Used to update weights of the dmps
void GazeboWheelchairWorld::update_DMP_weights()
{
    int num_goals = wheelchair_model_->goal_positions.size();
    wheelchair_model_->dmp_init_variables.update_weights(num_goals, wheelchair_model_->intermediate_goal_list);
    
}
void GazeboWheelchairWorld::printGoals()
{
    std::ofstream out_file;
    std::string out_file_name = "data/Voronoi_paths_step_" + std::to_string(step_no)+ ".txt";
    out_file.open(out_file_name);
    int num_goals = wheelchair_model_->goal_positions.size();
    out_file << num_goals << std::endl;
    tf2::Quaternion goal_quat;
     int path_size;
    for(int i=0;i< num_goals;i++){

        out_file << wheelchair_model_->goal_positions[i].point.x << " " 
        << wheelchair_model_->goal_positions[i].point.y << std::endl;

        path_size = wheelchair_model_->intermediate_goal_list.paths[i].poses.size();
        out_file << path_size  << std::endl;
        for(int j=0; j < path_size; j++)
        {

            out_file << wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose.position.x;
            out_file << " ";
            out_file << wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose.position.y;
            out_file << " ";
            tf2::convert(wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose.orientation, goal_quat);
            out_file << tf2::getYaw(goal_quat) << std::endl;

        }
        
    }
    out_file.close();


    //Print lidar points
    out_file_name = "data/Lidar_" + std::to_string(step_no)+ ".txt";
    out_file.open(out_file_name);
    int num_lidar_points = wheelchair_model_->lidar_points.size();
    out_file << num_lidar_points << std::endl;
    for(int i=0;i< num_lidar_points;i++){
        out_file<< wheelchair_model_->lidar_points[i].x << " "
        << wheelchair_model_->lidar_points[i].y << std::endl;
    }

    out_file.close();
}
void GazeboWheelchairWorld::generateGoal()
{
    if (path_receive)
    {
        // geometry_msgs::Point point_marker;
        // waypoint_viz.points.clear();
        intermediate_goals.paths.clear();
        wheelchair_model_->goal_positions.clear();
        wheelchair_model_->intermediate_goal_list.paths.clear();
        intermediate_goals.paths.resize(path_list.paths.size());
        wheelchair_model_->goal_positions.resize(path_list.paths.size());
        wheelchair_model_->intermediate_goal_list.paths.resize(path_list.paths.size());
        current_pose = getGlobalAgentPose().pose;
        for (int i = 0; i < path_list.paths.size(); i++)
        {
            intermediate_goals.paths[i] = GazeboWheelchairWorld::findIntermediateGoals(current_pose, path_list.paths[i]);
            intermediate_goals.paths[i].header.frame_id = path_frame_id;
            wheelchair_model_->goal_positions[i].header = path_list.paths[i].header;
            wheelchair_model_->intermediate_goal_list.paths[i].header = path_list.paths[i].header;
            // point_marker.x = goal_list.poses[i].position.x;
            // point_marker.y = goal_list.poses[i].position.y;
            // point_marker.z = 0;
            // goal_marker.points.push_back(point_marker);
        }

        final_goal = path_list.paths[0].poses[path_list.paths[0].poses.size() - 1].pose;
        // cout << "Newly obtained current pose: x = " << current_pose.position.x << ",y = " << current_pose.position.y << endl;
        // cout << "Newly obtained final goal: x = " << final_goal.position.x << ",y = " << final_goal.position.y << endl;

        // transform from map to base_link
        geometry_msgs::TransformStamped map2localTransform;
        try
        {
            map2localTransform = tf_buffer.lookupTransform(base_frame_id, path_frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
        }
        catch (tf2::TransformException &Exception)
        {
            ROS_ERROR_STREAM(Exception.what());
        }
        // update local intermediate goals
        // cout << "Size of itedmediate goal lists: " << endl;
        for (int i = 0; i < intermediate_goals.paths.size(); i++)
        {
            wheelchair_model_->intermediate_goal_list.paths[i].poses.resize(intermediate_goals.paths[i].poses.size());
            for (int j = 0; j < intermediate_goals.paths[i].poses.size(); j++)
            {
                tf2::doTransform<geometry_msgs::Pose>(intermediate_goals.paths[i].poses[j].pose,
                    wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose, map2localTransform);
                // waypoint_viz.points.push_back(wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose.position);
            }
            // wheelchair_model_->goal_positions[i].point = local_goal_list.poses[i].position;
            wheelchair_model_->intermediate_goal_list.paths[i].header.frame_id = base_frame_id;
            wheelchair_model_->goal_positions[i].point =
                wheelchair_model_->intermediate_goal_list.paths[i].poses[intermediate_goals.paths[i].poses.size() - 1].pose.position;
            wheelchair_model_->goal_positions[i].header.frame_id = base_frame_id;
            // cout << "Path " << i + 1 << ", size: " << wheelchair_model_->intermediate_goal_list.paths[i].poses.size() << endl;
        }
        update_DMP_weights();
        //printGoals();
        //step_no++;
    }
}

geometry_msgs::PoseStamped GazeboWheelchairWorld::getGlobalAgentPose()
{
    geometry_msgs::PoseStamped agent_pose_temp;
    agent_pose_temp.header.frame_id = base_frame_id;
    agent_pose_temp.pose.orientation.w = 1.0;

    try{
        auto tf = tf_buffer.lookupTransform(path_frame_id, base_frame_id, ros::Time(0), ros::Duration(0.15));
        tf2::doTransform<geometry_msgs::PoseStamped>(agent_pose_temp, agent_pose_temp, tf);
    }
    catch(tf2::TransformException &exception){
        ROS_ERROR("%s", exception.what());
    }

    return agent_pose_temp;
}

nav_msgs::Path GazeboWheelchairWorld::findIntermediateGoals(const geometry_msgs::Pose &agent_pose, const nav_msgs::Path &msg_path)
{
    nav_msgs::Path intermediate_goal_list;
    geometry_msgs::PoseStamped intermediate_goal;
    tf2::Vector3 goal_direction;
    tf2::Quaternion goal_quat;
    tf2Scalar yaw;
    intermediate_goal.header = msg_path.header;
    float acmlt_dist = GazeboWheelchairWorld::calDistance(agent_pose.position.x, agent_pose.position.y,
        msg_path.poses[0].pose.position.x, msg_path.poses[0].pose.position.y);
    for (int i = 1; i < msg_path.poses.size(); i++)
    {
        intermediate_goal.pose.position = msg_path.poses[i].pose.position;
        goal_direction.setValue(msg_path.poses[i].pose.position.x - msg_path.poses[i - 1].pose.position.x,
            msg_path.poses[i].pose.position.y - msg_path.poses[i - 1].pose.position.y, 0);
        yaw = goal_direction.getY() >= 0 ? goal_direction.angle(tf2::Vector3(1, 0, 0)) : - goal_direction.angle(tf2::Vector3(1, 0, 0));
        goal_quat.setRPY(0, 0, yaw);
        tf2::convert(goal_quat, intermediate_goal.pose.orientation);

        intermediate_goal_list.poses.push_back(intermediate_goal);

        acmlt_dist += GazeboWheelchairWorld::calDistance(msg_path.poses[i - 1].pose.position.x, msg_path.poses[i - 1].pose.position.y,
        msg_path.poses[i].pose.position.x, msg_path.poses[i].pose.position.y);

        if (i == msg_path.poses.size() - 1 || acmlt_dist >= waypoint_dist)
        {
            break;
        }
    }
    return intermediate_goal_list;
}

float GazeboWheelchairWorld::calDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
}