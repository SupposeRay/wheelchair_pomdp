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
    odom_subscriber_ = node_handle_->subscribe("/odometry/filtered", 1, &GazeboWheelchairWorld::odomCallback, this);
    joystick_subscriber_ = node_handle_->subscribe("/joystick_calib", 1, &GazeboWheelchairWorld::joystickCallback, this);
    // joystick_subscriber_ = node_handle_->subscribe("/keyboard", 1, &GazeboWheelchairWorld::joystickCallback, this);
    // map_subscriber_ = node_handle_->subscribe("/move_base/local_costmap/costmap", 1, &GazeboWheelchairWorld::mapCallback, this);
    pdwa_subscriber_ = node_handle_->subscribe("/shared_dwa/pure_dwa_vel", 1, &GazeboWheelchairWorld::pdwaCallback, this);
    bdwa_subscriber_ = node_handle_->subscribe("/shared_dwa/cmd_vel", 1, &GazeboWheelchairWorld::bdwaCallback, this);
    weight_subscriber_ = node_handle_->subscribe("/shared_dwa/user_weight", 1, &GazeboWheelchairWorld::weightCallback, this);
    // wypt_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/waypoints", 1);
    // wheelchair_state_client_ = node_handle_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    vel_publisher_ = node_handle_->advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    ucmd_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/pomdp_user_cmd", 1);
    pomdp_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/pomdp_final_output", 1);
    cancel_publisher_ = node_handle_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    // weight_publisher_ = node_handle_->advertise<std_msgs::Float32>("/shared_dwa/user_weight", 1);
    path_publisher_ = node_handle_->advertise<std_msgs::UInt32>("/preferred_path_ind", 1, true);
    footprint_publisher_ = node_handle_->advertise<geometry_msgs::PolygonStamped>("/visualization/pomdp_dynamic_rect", 1);
    adapt_publisher_ = node_handle_->advertise<visualization_msgs::Marker>("/visualization/pomdp_adapt_index", 1);
    goal_publisher_ = node_handle_->advertise<geometry_msgs::PoseArray>("/goal_distribution", 1, true);
    update_timer_ = node_handle_->createTimer(ros::Duration(ModelParams::update_interval), &GazeboWheelchairWorld::updatetimerCallback, this);
    publish_timer_ = node_handle_->createTimer(ros::Duration(ModelParams::publish_interval), &GazeboWheelchairWorld::publishtimerCallback, this);

    // if (!node_handle_->getParam("transition_time", transition_time))
    //     ROS_WARN_STREAM("Parameter transition_time not set. Using default setting: " << transition_time);
    // if (!node_handle_->getParam("waypoint_dist", waypoint_dist))
    //     ROS_WARN_STREAM("Parameter waypoint_dist not set. Using default setting: " << waypoint_dist);

    // transition_time = ModelParams::transition_time;
    // waypoint_dist = ModelParams::waypoint_dist;

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
    planner_type = ReadConfigFile();

    cmd_line.header.frame_id = base_frame_id;
    cmd_line.ns = "visualization";
    cmd_line.action = visualization_msgs::Marker::ADD;
    cmd_line.pose.orientation.w = 1.0;
    cmd_line.id = 1;
    cmd_line.type = visualization_msgs::Marker::LINE_STRIP;
    cmd_line.scale.x = 0.05;
    cmd_line.color.r = 1.0;
    cmd_line.color.g = 0.5;
    cmd_line.color.a = 1.0;

    pomdp_line.header.frame_id = base_frame_id;
    pomdp_line.ns = "visualization";
    pomdp_line.action = visualization_msgs::Marker::ADD;
    pomdp_line.pose.orientation.w = 1.0;
    pomdp_line.id = 2;
    pomdp_line.type = visualization_msgs::Marker::LINE_STRIP;
    pomdp_line.scale.x = 0.05;
    pomdp_line.color.r = 0.5;
    pomdp_line.color.b = 0.5;
    pomdp_line.color.a = 1.0;

    adapt_txt.header.frame_id = base_frame_id;
    adapt_txt.ns = "visualization";
    adapt_txt.action = visualization_msgs::Marker::ADD;
    adapt_txt.pose.orientation.w = 1.0;
    adapt_txt.id = 3;
    adapt_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    adapt_txt.scale.z = 0.5;
    adapt_txt.color.r = 1.0;
    adapt_txt.color.g = 1.0;
    adapt_txt.color.b = 1.0;
    adapt_txt.color.a = 1.0;
    adapt_txt.pose.position.x = -1;
    adapt_txt.pose.position.y = 0;
    adapt_txt.pose.orientation.w = 1;

    // initialize tf_listener
    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    pre_positions.clear();
    goal_positions.reserve(ModelParams::num_paths);

    // plot_ptr = new plot_scan::PlotScan();
    // float plot_height = 3.0;
    // plot_ptr->setImgSize(ModelParams::costmap_cols, ModelParams::costmap_rows);
    // plot_ptr->setHeight(plot_height);
    // plot_ptr->setFocal(plot_height / ModelParams::map_resolution);
    // cv::namedWindow("plot", CV_WINDOW_NORMAL);
    temp_costmap.create(ModelParams::costmap_rows, ModelParams::costmap_cols, CV_8UC1);
    local_costmap.create(ModelParams::costmap_rows, ModelParams::costmap_cols, CV_8UC1);

    temp_costmap = cv::Mat(ModelParams::costmap_rows, ModelParams::costmap_cols, CV_8UC1, cv::Scalar(0));
    local_costmap = cv::Mat(ModelParams::costmap_rows, ModelParams::costmap_cols, CV_8UC1, cv::Scalar(0));

    max_lidar_range = ModelParams::costmap_rows * ModelParams::map_resolution / 2 + 1;

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
    bool topics_receive = true;

    if (!path_receive)
    {
        ROS_ERROR_STREAM("Path info not received.");
        topics_receive = false;
    }
    if (!odom_receive)
    {
        ROS_ERROR_STREAM("Odometry info not received.");
        topics_receive = false;
    }
    if (!scan_receive)
    {
        ROS_ERROR_STREAM("Lidar info not received.");
        topics_receive = false;
    }
    if (!joystick_receive)
    {
        ROS_ERROR_STREAM("Joystick info not received.");
        topics_receive = false;
    }
    if (!topics_receive)
    {
        ROS_ERROR_STREAM("Required topics are not all received, please check the topic publishers! Shutting down ROS...");
        ros::requestShutdown();
    }

    // ==============================================

    if (planner_type == "POMDP" || planner_type == "POMDPX")
    {
        generateUserPath();
        updateModelInfo();
    }
    /**
     *  Initializing possible goal positions in the POMDP model 
     */

    // check that number of goals in the model and in the world is the same
    #ifdef DEBUG
    #endif
    if (goal_positions.size() != in_intermediate_goals.paths.size())
        ROS_WARN_STREAM("Number of possible goals in the world and the model is different!");
    else
        std::cout << "Number of possible goals: " << goal_positions.size() << std::endl;

    // Pick left door as the initial goal and udpate wheelchair state
    std::srand(std::time(0));
    wheelchair_state_->path_idx = std::rand() % in_intermediate_goals.paths.size();

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_cmd_vel = cmd_vel;
    old_cmd_vel = cmd_vel;

    reach_destination = false;
    stop_wheelchair = false;
    stop_count = 0;
    // GetCurrentState();
    std::cout << "========================= Intialized world =============================" << std::endl;

    return wheelchair_state_;
}

State* GazeboWheelchairWorld::GetCurrentState() const
{
    // ros::spinOnce();
    // if (wheelchair_state_ == NULL)
    // {
    //     std::cout << "State is NULL" << std::endl;
    // }
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
    if (stop_wheelchair)
    {
        cout << "DESPOT terminated." << endl;
        return true;
    }
    WheelchairObs wheelchair_external_obs;
    wheelchair_external_obs.continuous_obs = true;
    std::ostringstream obs_string;
    // Stop the wheelchair if there's no joystick input or the planning step reaches the max number
    if (zero_joystick_input || (action == -1) || reach_destination)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        // vel_publisher_.publish(cmd_vel);
        pub_cmd_vel = cmd_vel;

        if (planner_type == "POMDP" || planner_type == "POMDPX")
        {
            stuck_count = 0;
            wheelchair_external_obs.wheelchair_pose = initial_wheelchair_local_status.agent_pose;
            wheelchair_external_obs.agent_pose_angle = initial_wheelchair_local_status.agent_pose_angle;
            wheelchair_external_obs.wheelchair_twist.linear.x = 0;
            wheelchair_external_obs.wheelchair_twist.angular.z = 0;
            wheelchair_external_obs.joystick_signal.x = joystick_input.x;
            wheelchair_external_obs.joystick_signal.y = joystick_input.y;
            wheelchair_external_obs.joystick_signal.z = joystick_input.z;
            wheelchair_model_->PrintObs(wheelchair_external_obs, obs_string);
            // takes printed string to obs_string and converts to a hash value
            uint64_t hashValue = wheelchair_model_->obsHash(obs_string.str());
            // takes hashValue and updates value of obs in ObservationClass
            wheelchair_external_obs.SetIntObs(hashValue);

            // Updates hashmap:
            // takes hash value as key and stores wheelchair_obs as value in map
            // First check if hash value is already stored
            if (wheelchair_model_->obsHashMap_execution.find(hashValue) == wheelchair_model_->obsHashMap_execution.end())
            {
                wheelchair_model_->obsHashMap_execution[hashValue] = wheelchair_external_obs;
            }
            obs = (OBS_TYPE) hashValue;
        }
        return false;
    }

    // ros::spinOnce();
    // bool terminate_execution = false;

    if (planner_type == "POMDP" || planner_type == "POMDPX")
    {
        // Clear hashmap from previous round first
        wheelchair_model_->obsHashMap.clear();
        wheelchair_model_->collision_index = CheckCollision();
    }

    // cout << "Printing action values in ExecuteAction..." << endl;
    // for (int i = 0; i < Globals::config.action_values.size(); ++ i)
    // {
    //     cout << "Action " << i << ", value = " << Globals::config.action_values[i] << endl;
    // }
    // std::vector<float>::iterator itMax = std::max_element(Globals::config.action_values.begin(), Globals::config.action_values.end());
    // int best_action_index = std::distance(Globals::config.action_values.begin(), itMax);
    // // cout << "Optimal action value " << best_action_index << ", value = " << Globals::config.action_values[best_action_index] << endl;
    // float best_action_value = Globals::config.action_values[best_action_index];

    // Update wheelchair status (velocity)
    // if (!odom_receive)
    // {
    //     ROS_ERROR_STREAM("Failed to receive odom message");
    //     return true; // exit
    // }

    cmd_vel = current_vel;
    float scale_factor = 1.0; // wheelchair_state_->collision_idx > 0 ? ModelParams::step_scale : 1.0;

    float max_linear_speed = joystick_input.x > 0 ? fabs(joystick_input.x) : ModelParams::backing_ratio * fabs(joystick_input.x);
    max_linear_speed = std::min(max_linear_speed, ModelParams::max_linear_speed);

    float linear_step_size = ModelParams::planning_time * ModelParams::std_v_acceleration;
    float angular_step_size = ModelParams::planning_time * ModelParams::std_w_acceleration;

    int total_normal_actions = ModelParams::num_normal_actions + ModelParams::num_dwa_actions;

    if (planner_type == "POMDP" || planner_type == "POMDPX")
    {
        if (base_pixel >= ModelParams::outer_pixel_thres || front_pixel >= ModelParams::outer_pixel_thres)
        {
            cout << "The wheelchair is inside the collision area..." << endl;
            cout << "Moving to path " << instant_index + 1 << "..." << endl;
            cout << "stuck_count = " << stuck_count << "..." << endl;
            if (fabs(current_vel.linear.x) < 0.1 && fabs(current_vel.angular.z) < 0.1)
            {
                stuck_count++;
            }
            if (stuck_count * ModelParams::planning_time >= 2)
            {
                cout << "Switching to manual control mode at a slow speed..." << endl;
                cmd_vel.linear.x = joystick_input.x;
                cmd_vel.angular.z = joystick_input.y;
                if (joystick_input.x > 0.15)
                {
                    cmd_vel.linear.x = 0.15;
                }
                else if (joystick_input.x < -0.15)
                {
                    cmd_vel.linear.x = -0.15;
                }

                if (joystick_input.y > 0.2)
                {
                    cmd_vel.angular.z = 0.2;
                }
                else if (joystick_input.y < -0.2)
                {
                    cmd_vel.angular.z = -0.2;
                }
                // vel_publisher_.publish(cmd_vel);
                pub_cmd_vel = cmd_vel;
                // wheelchair_model_->collision_index = CheckCollision();
                wheelchair_external_obs.wheelchair_pose = initial_wheelchair_local_status.agent_pose;
                wheelchair_external_obs.agent_pose_angle = initial_wheelchair_local_status.agent_pose_angle;
                wheelchair_external_obs.wheelchair_twist = current_vel;
                wheelchair_external_obs.joystick_signal.x = joystick_input.x;
                wheelchair_external_obs.joystick_signal.y = joystick_input.y;
                wheelchair_external_obs.joystick_signal.z = joystick_input.z;
                wheelchair_model_->PrintObs(wheelchair_external_obs, obs_string);
                // takes printed string to obs_string and converts to a hash value
                uint64_t hashValue = wheelchair_model_->obsHash(obs_string.str());
                // takes hashValue and updates value of obs in ObservationClass
                wheelchair_external_obs.SetIntObs(hashValue);

                // Updates hashmap:
                // takes hash value as key and stores wheelchair_obs as value in map
                // First check if hash value is already stored
                if (wheelchair_model_->obsHashMap_execution.find(hashValue) == wheelchair_model_->obsHashMap_execution.end())
                {
                    wheelchair_model_->obsHashMap_execution[hashValue] = wheelchair_external_obs;
                }
                obs = (OBS_TYPE) hashValue;
                cout << "Current velocity: linear v: " << current_vel.linear.x << ", angular w: " << current_vel.angular.z << endl;
                cout << "Joystick input: x: " << joystick_input.x << ", y: " << joystick_input.y << endl;
                cout << "Published Command: linear v: " << cmd_vel.linear.x << ", angular w: " << cmd_vel.angular.z << endl;
                cout << "User control weight is " << user_weight << endl;
                return false;
            }
            action = total_normal_actions + instant_index;
        }
        else
        {
            stuck_count = 0;
        }

        if (projected_cmd_collision)
        {
            cout << "The joystick is pointing towards a collision area..." << endl;
            cout << "Moving to path " << instant_index + 1 << "..." << endl;
            action = total_normal_actions + instant_index;
        }        
    }

    switch (action)
    {
    case 0:     // linear acceleration
    {
        cmd_vel.linear.x = current_vel.linear.x + linear_step_size * scale_factor;
        cmd_vel.linear.x = cmd_vel.linear.x >= ModelParams::max_linear_speed ? ModelParams::max_linear_speed : cmd_vel.linear.x;
        break;
    }
    case 1:     // linear deceleration
    {
        cmd_vel.linear.x = current_vel.linear.x - linear_step_size * scale_factor;
        cmd_vel.linear.x = cmd_vel.linear.x <= - ModelParams::max_linear_speed ? - ModelParams::max_linear_speed : cmd_vel.linear.x;
        break;
    }
    case 2:     // angular acceleration
    {
        cmd_vel.angular.z = current_vel.angular.z + angular_step_size * scale_factor;
        cmd_vel.angular.z = cmd_vel.angular.z >= ModelParams::max_angular_speed ? ModelParams::max_angular_speed : cmd_vel.angular.z;
        break;
    }
    case 3:     // angular deceleration
    {
        cmd_vel.angular.z = current_vel.angular.z - angular_step_size * scale_factor;
        cmd_vel.angular.z = cmd_vel.angular.z <= - ModelParams::max_angular_speed ? - ModelParams::max_angular_speed : cmd_vel.angular.z;
        break;
    }
    case 4:     // keep current velocities
    {
        cmd_vel.linear.x = current_vel.linear.x;
        cmd_vel.angular.z = current_vel.angular.z;
        break;
    }
    case 5:     // stop the wheelchair
    {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        float max_linear_step = ModelParams::planning_time * ModelParams::max_v_acceleration;
        float max_angular_step = ModelParams::planning_time * ModelParams::max_w_acceleration;
        // check if it requires more than 1 step to finish the velocity change
        if (fabs(current_vel.linear.x) > max_linear_step)
        {
            if (current_vel.linear.x > 0)
            {
                cmd_vel.linear.x = current_vel.linear.x - max_linear_step;
            }
            else
            {
                cmd_vel.linear.x = current_vel.linear.x + max_linear_step;
            }
        }
        if (fabs(current_vel.angular.z) > max_angular_step)
        {
            if (current_vel.angular.z > 0)
            {
                cmd_vel.angular.z = current_vel.angular.z - max_angular_step;
            }
            else
            {
                cmd_vel.angular.z = current_vel.angular.z + max_angular_step;
            }
        }
        break;
    }
    case 6:     // follow the user input
    {
        // use the current joystick x&y or previous ones?? wheelchair_model_->external_joy_x or joystick_input.x?
        tf2::Vector3 joystick_heading(joystick_input.x, joystick_input.y, 0);
        float v_follow = 0, w_follow = 0;
        wheelchair_model_->FollowingVel(joystick_heading, current_vel.linear.x, current_vel.angular.z, v_follow, w_follow, max_linear_speed, ModelParams::planning_time);

        cmd_vel.linear.x = v_follow;
        cmd_vel.angular.z = w_follow;
        // cmd_vel.linear.x = pdwa_v;
        // cmd_vel.angular.z = pdwa_w;
        break;
    }
    case 7:    // pure shared-DWA
    {
        cmd_vel.linear.x = pdwa_v;
        cmd_vel.angular.z = pdwa_w;
        break;            
    }
    case 8:    // belief shared-DWA
    {
        cmd_vel.linear.x = bdwa_v;
        cmd_vel.angular.z = bdwa_w;
        break;
    }
    default:    // move to goal actions
    {
        tf2::Vector3 agent2path(0, 0, 0);
        tf2::Vector3 agent_heading(1, 0, 0);
        float angle2turn = 0, distance2goal = 0;
        for (int i = 0; i < in_intermediate_goals.paths[action - total_normal_actions].poses.size(); ++i)
        {
            agent2path.setValue(in_intermediate_goals.paths[action - total_normal_actions].poses[i].pose.position.x,
                in_intermediate_goals.paths[action - total_normal_actions].poses[i].pose.position.y, 0);

            distance2goal = agent2path.length();

            if (distance2goal >= 0.35)
                break;
        }
        // tf2::Vector3 agent2path(in_intermediate_goals.paths[action - total_normal_actions].poses[1].pose.position.x,
        //     in_intermediate_goals.paths[action - total_normal_actions].poses[1].pose.position.y, 0);
        
        // the angle between current heading and the direction facing the path
        angle2turn = agent_heading.angle(agent2path);
        cout << "The angle2turn is " << angle2turn * 180 / M_PI << endl;
        cout << "The distance is " << distance2goal << endl;
        
        // the wheelchair is facing the path
        if (fabs(angle2turn) <= ModelParams::facing_angle)
        {
            // the wheelchair is facing the path, but still turning, stop the wheelchair
            if (fabs(current_vel.angular.z) > angular_step_size)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                // if (fabs(current_vel.angular.z) > ModelParams::planning_time * ModelParams::max_w_acceleration)
                // {
                //     if (current_vel.angular.z > 0)
                //     {
                //         cmd_vel.angular.z = current_vel.angular.z - ModelParams::planning_time * ModelParams::max_w_acceleration;
                //     }
                //     else
                //     {
                //         cmd_vel.angular.z = current_vel.angular.z + ModelParams::planning_time * ModelParams::max_w_acceleration;
                //     }
                // }
            }
            // the wheelchair is facing the path, and not turning, move along the path at a slow speed
            else
            {
                // if (current_vel.linear.x >= ModelParams::planning_time * ModelParams::max_v_acceleration + linear_step_size)
                // {
                //     cmd_vel.linear.x = current_vel.linear.x - ModelParams::planning_time * ModelParams::max_v_acceleration;
                // }
                // else
                // {
                //     cmd_vel.linear.x = linear_step_size * ModelParams::step_scale;
                // }
                cmd_vel.linear.x = linear_step_size;
                // cmd_vel.linear.x = 0.25;
                cmd_vel.angular.z = 0;
            }
        }
        // the wheelchair needs to turn to face the path
        else
        {
            // if wheelchair is still moving, stop it first
            if (fabs(current_vel.linear.x) > 0.1)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                // if (fabs(current_vel.linear.x) > ModelParams::planning_time * ModelParams::max_v_acceleration)
                // {
                //     if (current_vel.linear.x > 0)
                //     {
                //         cmd_vel.linear.x = current_vel.linear.x - ModelParams::planning_time * ModelParams::max_v_acceleration;
                //     }
                //     else
                //     {
                //         cmd_vel.linear.x = current_vel.linear.x + ModelParams::planning_time * ModelParams::max_v_acceleration;
                //     }
                // }
                // if (fabs(current_vel.angular.z) > ModelParams::planning_time * ModelParams::max_w_acceleration)
                // {
                //     if (current_vel.angular.z > 0)
                //     {
                //         cmd_vel.angular.z = current_vel.angular.z - ModelParams::planning_time * ModelParams::max_w_acceleration;
                //     }
                //     else
                //     {
                //         cmd_vel.angular.z = current_vel.angular.z + ModelParams::planning_time * ModelParams::max_w_acceleration;
                //     }
                // }
            }
            else
            {
                float cross_product = agent_heading.getX() * agent2path.getY() - agent_heading.getY() * agent2path.getX();
                angle2turn = (cross_product >= 0)? angle2turn : -angle2turn;

                if (fabs(angle2turn) <= M_PI / 3)
                {
                    float angular_vel = 0;
                    float current_w = current_vel.angular.z;
                    if (current_w > ModelParams::max_angular_speed)
                    {
                        current_w = ModelParams::max_angular_speed;
                    }
                    if (current_w < - ModelParams::max_angular_speed)
                    {
                        current_w = - ModelParams::max_angular_speed;
                    }
                    wheelchair_model_->TurningSteps(angle2turn, current_w, angular_vel, ModelParams::planning_time);

                    cout << "The angular_vel is " << angular_vel << endl;

                    cmd_vel.linear.x = 0;
                    float actual_angular_vel = angular_step_size * 1.4;
                    if (fabs(angular_vel) >= actual_angular_vel)
                    {
                        cmd_vel.angular.z = angular_vel > 0 ? actual_angular_vel : -actual_angular_vel;
                    }
                    else
                    {
                        cmd_vel.angular.z = angular_vel;
                    }
                }
                else
                {
                    if (base_pixel == 255 && front_pixel == 255)    // wheelchair's whole footprint inside the collision zone, 
                    {
                        // change to manual control at a low speed
                        cout << "Wheelchair all inside collision zone, switching to manual control mode at a slow speed..." << endl;
                        cmd_vel.linear.x = joystick_input.x;
                        cmd_vel.angular.z = joystick_input.y;
                        if (joystick_input.x > 0.15)
                        {
                            cmd_vel.linear.x = 0.15;
                        }
                        else if (joystick_input.x < -0.15)
                        {
                            cmd_vel.linear.x = -0.15;
                        }

                        if (joystick_input.y > 0.15)
                        {
                            cmd_vel.angular.z = 0.15;
                        }
                        else if (joystick_input.y < -0.15)
                        {
                            cmd_vel.angular.z = -0.15;
                        }
                    }
                    else if (front_pixel == 255)     // wheelchair's front inside the collision zone.
                    {
                        // back the wheelchair
                        cout << "Wheelchair front inside collision zone, backing the wheelchair..." << endl;
                        cmd_vel.linear.x = - linear_step_size;
                        cmd_vel.angular.z = 0;
                    }
                    else    // wheelchair's back inside the collision zone or whole wheelchair outside collision zone
                    {
                        // turn in place as usual
                        cmd_vel.linear.x = 0;
                        float actual_angular_vel = angular_step_size * 1.4;
                        cmd_vel.angular.z = angle2turn > 0 ? actual_angular_vel : -actual_angular_vel;
                    }
                }
            }
        }
        break;
    }
    }
   
    // execute the action
    #ifdef DEBUG
    #endif

    cout << "Current velocity: linear v: " << current_vel.linear.x << ", angular w: " << current_vel.angular.z << endl;
    cout << "Joystick input: x: " << joystick_input.x << ", y: " << joystick_input.y << endl;
    cout << "Published Command: linear v: " << cmd_vel.linear.x << ", angular w: " << cmd_vel.angular.z << endl;
    cout << "User control weight is " << user_weight << endl;
    wheelchair_model_->PrintAction(action);
    cout << "Pure DWA: linear v: " << pdwa_v << ", angular w: " << pdwa_w << endl;
    cout << "Belief DWA: linear v: " << bdwa_v << ", angular w: " << bdwa_w << endl;
    // vel_publisher_.publish(cmd_vel);
    pub_cmd_vel = cmd_vel;
    PrintBelief();

    // tf2::Vector3 joystick_heading(0.55, 0.83, 0);
    // float v_follow = 0, w_follow = 0;
    // wheelchair_model_->FollowingVel(joystick_heading, 0.95, -0.04, v_follow, w_follow, 0.55, ModelParams::transition_time, true);
    // cout << "Test joystick input x = " << joystick_heading.getX() << ", y = " << joystick_heading.getY() << endl;
    // cout << "Test follow v = " << v_follow << ", w = " << w_follow << endl;

    //=================== Whether to wait for a few ms to execute the cmd_vel??? ======================//

    // ros::Duration(0.010).sleep();

    if (CheckDestination(current_pose, final_goal))
    {
        // terminate when the goal is reached
        cout << "Final goal has been reached, the task will be terminated soon!" << endl;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        // cout << "Published Command: linear v: " << cmd_vel.linear.x << ", angular w: " << cmd_vel.angular.z << endl;
        // vel_publisher_.publish(cmd_vel);
        pub_cmd_vel = cmd_vel;
        cancel_publisher_.publish(dummy_ID);
        footprint_polygon.polygon.points.clear();
        footprint_publisher_.publish(footprint_polygon);
        cmd_line.points.clear();
        pomdp_line.points.clear();
        ucmd_publisher_.publish(cmd_line);
        pomdp_publisher_.publish(pomdp_line);
        // cout << "Final goal: x = " << final_goal.position.x << ",y = " << final_goal.position.y << endl;
        reach_destination = true;
        // return false;
    }
    
    if (planner_type == "POMDP" || planner_type == "POMDPX")
    {
        // Observation part
        wheelchair_external_obs.wheelchair_pose = initial_wheelchair_local_status.agent_pose;
        wheelchair_external_obs.agent_pose_angle = initial_wheelchair_local_status.agent_pose_angle; //Do we need to calculate pose angle from pose here?
        wheelchair_external_obs.wheelchair_twist = current_vel;
        wheelchair_external_obs.joystick_signal.x = joystick_input.x;
        wheelchair_external_obs.joystick_signal.y = joystick_input.y;
        wheelchair_external_obs.joystick_signal.z = joystick_input.z;

        // wheelchair_model_->collision_index = CheckCollision();
        // wheelchair_state_->collision_idx = wheelchair_model_->collision_index;
        // if (wheelchair_model_->collision_index == 1)
        // {
        //     // terminate when a collision happens
        //     cout << "A COLLLSION HAS HAPPENED!" << endl;
        //     // cmd_vel.linear.x = 0;
        //     // cmd_vel.angular.z = 0;
        //     // // cout << "Published Command: linear v: " << cmd_vel.linear.x << ", angular w: " << cmd_vel.angular.z << endl;
        //     // vel_publisher_.publish(cmd_vel);
        //     // cancel_publisher_.publish(dummy_ID);
        //     // cmd_line.points.clear();
        //     // pomdp_line.points.clear();
        //     // ucmd_publisher_.publish(cmd_line);
        //     // pomdp_publisher_.publish(pomdp_line);
        //     // // cout << "Final goal: x = " << final_goal.position.x << ",y = " << final_goal.position.y << endl;
        //     // return true;
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
        // wheelchair_model_->lidar_points = lidar_points;

        /* Update the internal info */

        generateUserPath();
        updateModelInfo();

        if (Globals::config.useGPU)
        {
            wheelchair_model_->UpdateGPUModel();
        }
    }
    // cv::Mat scan_img;

    // plot_scan_.drawScan(scan_pointer, scan_img);

    // std::string filename = "/home/ray/DESPOT-LOG/" + std::to_string(img_count) + ".jpg";
    // cv::imwrite(filename, scan_img);

    // img_count ++;
    return false;
}

void GazeboWheelchairWorld::PrintState(const State& state, ostream& out) const
{
	const WheelchairState& wheelchair_state = static_cast<const WheelchairState&>(state);
	out << "Wheelchair position: x = " << wheelchair_state.wheelchair.agent_pose.position.x << "; y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
	out << "Wheelchair orientation: x = " << wheelchair_state.wheelchair.agent_pose.orientation.x << "; y = " << wheelchair_state.wheelchair.agent_pose.orientation.y 
		<< "; z = " << wheelchair_state.wheelchair.agent_pose.orientation.z << "; w = " << wheelchair_state.wheelchair.agent_pose.orientation.w << endl;
	out << "Goal door global position: x = " << goal_positions[wheelchair_state.path_idx].pose.position.x << "; y = " << goal_positions[wheelchair_state.path_idx].pose.position.y << endl;
}

void GazeboWheelchairWorld::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_scan)
{
    //Get static transform from lidar to base_link in case they are not in the same frame
    // if (lidar2baseTransform.header.frame_id != base_frame_id && msg_scan->header.frame_id != base_frame_id)
    // {
    //     ROS_INFO("LIDAR is not in base link frame and transform has not been found yet, finding transform");
    //     try
    //     {
    //         lidar2baseTransform = tf_buffer.lookupTransform(base_frame_id, msg_scan->header.frame_id, ros::Time(0), ros::Duration(tf_buffer_timeout));
    //         ROS_INFO("Transform found, all future scans received by shared_pomdp will be transformed before being used for collision checking");
    //     }
    //     catch (tf2::TransformException &Exception)
    //     {
    //         ROS_ERROR("LIDAR tranform could not be found, shared_pomdp may be incorrect");
    //         ROS_ERROR_STREAM(Exception.what());
    //     }
    // }
    // scan_pointer = msg_scan;

    // plot_ptr->drawScan(msg_scan, local_costmap);

    int scan_size = msg_scan->ranges.size();
    float x = 0, y = 0, row = 0, col = 0;
    temp_costmap = cv::Mat::zeros(ModelParams::costmap_rows, ModelParams::costmap_cols, CV_8UC1);
    
    for (int i = 0; i < scan_size; ++i)
    {
        if (msg_scan->ranges[i] < max_lidar_range && msg_scan->ranges[i] >= msg_scan->range_min)
        {
            x = msg_scan->ranges[i] * cos(msg_scan->angle_min + (double) i * msg_scan->angle_increment);
            y = msg_scan->ranges[i] * sin(msg_scan->angle_min + (double) i * msg_scan->angle_increment);

            // std::cout << "i = " << i << ", x = " << x << ", y = " << y << std::endl;

            row = round(-x / ModelParams::map_resolution + ModelParams::costmap_rows / 2);
            col = round(-y / ModelParams::map_resolution + ModelParams::costmap_cols / 2);

            if(col > ModelParams::costmap_cols - 1 || col < 0 || row > ModelParams::costmap_rows - 1 || row < 0)
            {
                continue;
            }
            else
            {
                temp_costmap.at<uint8_t>(row, col) = 255;
            }
        }
    }

    // cv::cvtColor(temp_costmap, temp_costmap, cv::COLOR_RGB2GRAY);
    // cv::threshold(temp_costmap, temp_costmap, 128, 255, cv::THRESH_BINARY_INV);

    // Dilation size on one side of obstacle is pixel/map_resolution/2
	// ie 20 pixels * 0.05m/px / 2 = 0.5m inflation on obstacles
	int morph_size = ModelParams::outer_radius * 2 / ModelParams::map_resolution;
	cv::Mat structure_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_size, morph_size));
	cv::dilate(temp_costmap, temp_costmap, structure_element);

    x_center = ModelParams::costmap_rows / 2;
    y_center = ModelParams::costmap_cols / 2;
    

    thread_mutex.lock();
    local_costmap = temp_costmap.clone();
    thread_mutex.unlock();

    // cv::imshow("plot", local_costmap);
    // cv::waitKey(1);

    // lidar_points.clear();
    // lidar_points.reserve(msg_scan->ranges.size());
    // for (int k = 0; k < msg_scan->ranges.size(); ++k)
    // {
    //     geometry_msgs::Point temp_point;
    //     temp_point.x = msg_scan->ranges[k] * cos(msg_scan->angle_min + k * msg_scan->angle_increment);
    //     temp_point.y = msg_scan->ranges[k] * sin(msg_scan->angle_min + k * msg_scan->angle_increment);

    //     //If transform header is not empty
    //     if (!lidar2baseTransform.header.frame_id.empty())
    //         tf2::doTransform<geometry_msgs::Point>(temp_point, temp_point, lidar2baseTransform);

    //     // if (fabs(temp_point.x) + fabs(temp_point.y) <= 0.4)
    //     // {
    //     //     cout << "temp_point.x " << temp_point.x << ", temp_point.y " << temp_point.y << endl;
    //     //     cout << "range " << msg_scan->ranges[k] << endl;
    //     // }
	// 	// else if (sqrtf(powf(temp_point.x, 2) + powf(temp_point.y, 2)) <= 0.4)
    //     // {
    //     //     cout << "temp_point.x " << temp_point.x << ", temp_point.y " << temp_point.y << endl;
    //     //     cout << "range " << msg_scan->ranges[k] << endl;
    //     // }
    //     lidar_points.emplace_back(std::move(temp_point));
    // }
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
    current_vel.linear.x = round(current_vel.linear.x * 100) / 100;
    current_vel.angular.z = round(current_vel.angular.z * 100) / 100;
    odom_receive = true;
}

// void GazeboWheelchairWorld::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map)
// {
//     // map.height = msg->info.height;
// 	// map.width = msg->info.width;
// 	// map.frame_id = msg->header.frame_id;
// 	// map.resolution = msg->info.resolution;
// 	// map.origin.position.x = msg->info.origin.position.x;
// 	// map.origin.position.y = msg->info.origin.position.y;
//     map_resolution = msg_map->info.resolution;
//     map_pose = msg_map->info.origin;

// 	// Dilate image
// 	local_costmap = cv::Mat(msg_map->data).reshape(0, msg_map->info.height);
// 	local_costmap.convertTo(local_costmap, CV_8UC1);

//     x_center = local_costmap.cols % 2 == 0 ? local_costmap.cols / 2 - 1 : (local_costmap.cols - 1) / 2;
//     y_center = local_costmap.rows % 2 == 0 ? local_costmap.rows / 2 - 1 : (local_costmap.rows - 1) / 2;

//     // for (int i = 0; i < local_costmap.rows; i++)
//     // {
//     //     for (int j = 0; j < local_costmap.cols; j++)
//     //     {
//     //         if (std::isnan(local_costmap.at<uint8_t>(i, j)))
//     //         {
//     //             cout << "There's nan value in the costmap." << endl;
//     //         }

//     //         cout << "Row " << i << ", col " << j << ", value = " << local_costmap.at<uint8_t>(i, j) << endl;
//     //     }
//     // }

//     // ROS_INFO_STREAM(local_costmap);

//     // std::string filename = "/home/ray/DESPOT-LOG/original_costmap_" + std::to_string(img_count) + ".jpg";
//     // cv::imwrite(filename, local_costmap);

//     //Dilation size on one side of obstacle is pixel/map_resolution/2
// 	//ie 20 pixels * 0.05m/px / 2 = 0.5m inflation on obstacles
// 	// int morph_size = ModelParams::dilation_radius * 2 / map_resolution;
// 	// cv::Mat structure_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_size, morph_size));
// 	// cv::dilate(local_costmap, local_costmap, structure_element);

// 	//Blur. Blur kernel must be odd number
// 	// int blur_size = ModelParams::blur_radius * 2 / map_resolution;
// 	// blur_size += (blur_size + 1) % 2;

// 	// cv::GaussianBlur(local_costmap, local_costmap, cv::Size(blur_size, blur_size), 3);
//     // filename = "/home/ray/DESPOT-LOG/Gaussian_costmap_" + std::to_string(img_count) + ".jpg";
//     // cv::imwrite(filename, local_costmap);

//     // cv::blur(local_costmap, local_costmap, cv::Size(blur_size, blur_size));
//     // filename = "/home/ray/DESPOT-LOG/Mean_costmap_" + std::to_string(img_count) + ".jpg";
//     // cv::imwrite(filename, local_costmap);

//     // cv::GaussianBlur(local_costmap, local_costmap, cv::Size(blur_size, blur_size), 3);
//     // filename = "/home/ray/DESPOT-LOG/Gaussian_costmap_" + std::to_string(img_count) + ".jpg";
//     // cv::imwrite(filename, local_costmap);
//     // img_count++;
//     costmap_receive = true;
// }

void GazeboWheelchairWorld::pdwaCallback(const geometry_msgs::Twist::ConstPtr &msg_pdwa)
{
    pdwa_v = msg_pdwa->linear.x;
    pdwa_w = msg_pdwa->angular.z;
    pdwa_receive = true;
}

void GazeboWheelchairWorld::bdwaCallback(const geometry_msgs::Twist::ConstPtr &msg_bdwa)
{
    bdwa_v = msg_bdwa->linear.x;
    bdwa_w = msg_bdwa->angular.z;
    bdwa_receive = true;    
}

void GazeboWheelchairWorld::weightCallback(const std_msgs::Float32::ConstPtr &msg_weight)
{
    user_weight = msg_weight->data;
    weight_receive = true;    
}

std::string GazeboWheelchairWorld::ReadConfigFile()
{
    std::string planner_type = "POMDP";
    ifstream config_file;
    config_file.open("/home/ray/scwheelchair_ws/src/wheelchair_pomdp/wheelchair_pomdp_GPU_version/config/wheelchair_pomdp.txt");
    if (!config_file.is_open())
    {
        cout << "Failed to open the config file, using default POMDP planner..." << endl;
        return planner_type;
    }
    else
    {
        getline(config_file, planner_type);
        cout << "Loaded the config file, using " << planner_type << " planner..." << endl;
    }
    config_file.close();
    return planner_type;
}

bool GazeboWheelchairWorld::CheckDestination(const geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
{
    float dist2goal = 0;
	dist2goal = sqrt(powf((current_pose.position.x - goal_pose.position.x), 2) + powf((current_pose.position.y - goal_pose.position.y), 2));
	if (dist2goal <= 0.5)
		return true;
	else
		return false;
}

float GazeboWheelchairWorld::CheckCollision()
{
    // tf2::Vector3 check_vector_front(ModelParams::base2front_dist, 0, 0);

	// check_vector_front = tf2::quatRotate(map_quaternion, check_vector_front);
    float x_front = ModelParams::base2front_dist;
    float y_front = 0;

    base_pixel = local_costmap.at<uint8_t>(y_center, x_center);

    int row_front = static_cast<int>(- x_front / ModelParams::map_resolution);
	int col_front = 0;

	front_pixel = local_costmap.at<uint8_t>(x_center + row_front, y_center + col_front);

    cout << "base_pixel =  " << base_pixel << endl;
    cout << "front_pixel =  " << front_pixel << endl;

    // std::string file_direct;

    // if (base_pixel == 255 || front_pixel == 255)
    // {
    //     file_direct = "CY";
    // }
    // else
    // {
    //     file_direct = "CN";
    // }


    // std::string filename = "/home/ray/DESPOT-LOG/" + std::to_string(img_count) + ".jpg";
    // cv::imwrite(filename, local_costmap);

    // img_count ++;

	if (base_pixel >= ModelParams::outer_pixel_thres || front_pixel >= ModelParams::outer_pixel_thres)
	{
		return 1;
	}
    else
    {
        return 0;
    }
	// else if (base_pixel < ModelParams::inner_pixel_thres && front_pixel < ModelParams::inner_pixel_thres)
	// {
	// 	return 0;
	// }
	// else
	// {
	// 	if (base_pixel < ModelParams::inner_pixel_thres)
	// 	{
	// 		return 0.5 * (static_cast<float>(front_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
	// 	}
	// 	else if (front_pixel < ModelParams::inner_pixel_thres)
	// 	{
	// 		return 0.5 * (static_cast<float>(base_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
	// 	}
	// 	else
	// 	{
	// 		float double_collision = 0;
	// 		double_collision = (static_cast<float>(base_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
	// 		double_collision += (static_cast<float>(front_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
	// 		return 0.5 * double_collision;
	// 	}
	// }
	// float r_collision = ModelParams::inner_radius;
	// float r_outer = ModelParams::outer_radius;
	// float temp_dist = 0;
	// float penalty_idx = 0;

	// for (const auto &point : lidar_points)
	// {
	// 	//https://stackoverflow.com/a/7227057
	// 	//Check through easy conditions first, getting distance with sqrt is computationally expensive
	// 	double dx = fabs(point.x);
	// 	double dy = fabs(point.y);

	// 	if (dx > r_outer || dy > r_outer)
	// 		continue;

	// 	if (dx + dy <= r_collision)
	// 	{
	// 		penalty_idx = 1;
	// 		return penalty_idx;
	// 	}
	// 	else
	// 	{
	// 		temp_dist = sqrtf(powf(dx, 2) + powf(dy, 2));
	// 		if (temp_dist <= r_collision)
	// 		{
	// 			penalty_idx = 1;
	// 			return penalty_idx;
	// 		}

	// 		if (temp_dist < r_outer)
	// 		{
	// 			penalty_idx = 1 - (temp_dist - r_collision) / (r_outer - r_collision);
	// 		}
	// 	}
	// }
	// return penalty_idx;
}

void GazeboWheelchairWorld::pathCallback(const voronoi_msgs_and_types::PathList &msg_path)
{
    path_list = msg_path;
    // check if the paths exist or have changed
    if (path_list.paths.size() == 0)
    {
        // ROS_ERROR_STREAM("Path info not received.");
        path_receive = false;
        // return;
    }
    else
    {
        path_receive = true;
    }
}

void GazeboWheelchairWorld::publishtimerCallback(const ros::TimerEvent &)
{
    // cout << "pub_cmd_vel x = " << pub_cmd_vel.linear.x << " y = " << pub_cmd_vel.angular.z << endl;
    geometry_msgs::Twist temp_twist = pub_cmd_vel;
    if (reach_destination)
    {
        // cout << "stop_count = " << stop_count << endl;
        pub_cmd_vel.linear.x = 0;
        pub_cmd_vel.angular.z = 0;
        stop_count++;
    }
    if (fabs(pub_cmd_vel.linear.x - old_cmd_vel.linear.x) > ModelParams::publish_interval * ModelParams::max_v_acceleration)
    {
        if (pub_cmd_vel.linear.x  > old_cmd_vel.linear.x)
        {
            temp_twist.linear.x = old_cmd_vel.linear.x + ModelParams::publish_interval * ModelParams::max_v_acceleration;
        }
        else
        {
            temp_twist.linear.x = old_cmd_vel.linear.x - ModelParams::publish_interval * ModelParams::max_v_acceleration;
        }
    }
    
    if (fabs(pub_cmd_vel.angular.z - old_cmd_vel.angular.z) > ModelParams::publish_interval * ModelParams::max_w_acceleration)
    {
        if (pub_cmd_vel.angular.z  > old_cmd_vel.angular.z)
        {
            temp_twist.angular.z = old_cmd_vel.angular.z + ModelParams::publish_interval * ModelParams::max_w_acceleration;
        }
        else
        {
            temp_twist.angular.z = old_cmd_vel.angular.z - ModelParams::publish_interval * ModelParams::max_w_acceleration;
        }
    }
    temp_twist.linear.x = round(temp_twist.linear.x * 1000) / 1000;
    temp_twist.angular.z = round(temp_twist.angular.z * 1000) / 1000;
    vel_publisher_.publish(temp_twist);
    if (stop_count > 20)
    {
        stop_wheelchair = true;
        cout << "Quitting DESPOT..." << endl;
    }
    old_cmd_vel = temp_twist;
}

void GazeboWheelchairWorld::updatetimerCallback(const ros::TimerEvent &)
{
    if (joystick_receive && odom_receive && path_receive && scan_receive)
    {
        /* The footprint of the wheelchair */

        geometry_msgs::Point32 temp_point;

        temp_point.x = 1.05;
        temp_point.y = 0.45;
        footprint_polygon.polygon.points.push_back(temp_point);
        temp_point.y = -0.45;
        footprint_polygon.polygon.points.push_back(temp_point);
        temp_point.x = -0.55;
        footprint_polygon.polygon.points.push_back(temp_point);
        temp_point.y = 0.45;
        footprint_polygon.polygon.points.push_back(temp_point);

        /* The footprint of a dynamic rectangle used to compute the clearance index */

        // int search_depth = ModelParams::search_depth - 3;
        // if (fabs(current_vel.angular.z) < 0.1)
        // {
        //     // straight movement
        //     // 1.5 is the length and the width of the wheelchair
        //     rotate_rect_length = fabs(current_vel.linear.x) * ModelParams::transition_time * search_depth + ModelParams::inflation_width;

        //     rect_center_x = current_vel.linear.x >= 0 ? 0.5 * (rotate_rect_length - ModelParams::inflation_width) : - 0.5 * (rotate_rect_length - ModelParams::inflation_width);

        //     rotate_rect_width = ModelParams::inflation_width;

        //     rect_center_y = 0;
        // }
        // else
        // {
        //     // curvilinear movement
        //     float rotate_R = fabs(current_vel.linear.x) / fabs(current_vel.angular.z);
        //     rotate_rect_length = rotate_R * sin(fabs(current_vel.angular.z) * ModelParams::transition_time * search_depth) + ModelParams::inflation_width;
        //     rotate_rect_width = rotate_R * (1 - cos(fabs(current_vel.angular.z) * ModelParams::transition_time * search_depth)) + ModelParams::inflation_width;

        //     rect_center_x = current_vel.linear.x >= 0 ? 0.5 * (rotate_rect_length - ModelParams::inflation_width) : - 0.5 * (rotate_rect_length - ModelParams::inflation_width);

        //     rect_center_y = current_vel.angular.z >= 0 ? 0.5 * (rotate_rect_width - ModelParams::inflation_width) : - 0.5 * (rotate_rect_width - ModelParams::inflation_width);
        // }

        // std::vector<cv::Point2f> rotate_vertices;
        // cv::Point2f temp_vertex;
        // geometry_msgs::Point32 temp_point;

        // temp_point.x = rect_center_x + 0.5 * rotate_rect_length;
        // temp_point.y = rect_center_y + 0.5 * rotate_rect_width;
        // temp_vertex.x = temp_point.x;
        // temp_vertex.y = temp_point.y;
        // footprint_polygon.polygon.points.push_back(temp_point);
        // rotate_vertices.push_back(temp_vertex);

        // temp_point.x = rect_center_x + 0.5 * rotate_rect_length;
        // temp_point.y = rect_center_y - 0.5 * rotate_rect_width;
        // temp_vertex.x = temp_point.x;
        // temp_vertex.y = temp_point.y;
        // footprint_polygon.polygon.points.push_back(temp_point);
        // rotate_vertices.push_back(temp_vertex);

        // temp_point.x = rect_center_x - 0.5 * rotate_rect_length;
        // temp_point.y = rect_center_y - 0.5 * rotate_rect_width;
        // temp_vertex.x = temp_point.x;
        // temp_vertex.y = temp_point.y;
        // footprint_polygon.polygon.points.push_back(temp_point);
        // rotate_vertices.push_back(temp_vertex);

        // temp_point.x = rect_center_x - 0.5 * rotate_rect_length;
        // temp_point.y = rect_center_y + 0.5 * rotate_rect_width;
        // temp_vertex.x = temp_point.x;
        // temp_vertex.y = temp_point.y;
        // footprint_polygon.polygon.points.push_back(temp_point);
        // rotate_vertices.push_back(temp_vertex);
        
        footprint_polygon.header.frame_id = base_frame_id;

        /* Generate the paths in the local frame */

        GazeboWheelchairWorld::generateGoal();

        /* Compute the clearance index based on the dynamic rectangle in costmap */

        // for (int i = 0; i < 4; ++i)
        // {
        //     tf2::Vector3 check_vector_center(rotate_vertices[i].x, rotate_vertices[i].y, 0);

        //     check_vector_center = tf2::quatRotate(map_quaternion, check_vector_center);

        //     temp_vertex.x = check_vector_center.getX();
        //     temp_vertex.y = check_vector_center.getY();

        //     int col_temp = temp_vertex.x >= 0 ? ceil(temp_vertex.x / map_resolution) : floor(temp_vertex.x / map_resolution);
        //     int row_temp = temp_vertex.y >= 0 ? ceil(temp_vertex.y / map_resolution) : floor(temp_vertex.y / map_resolution);

        //     temp_vertex.x = x_center + col_temp;
        //     temp_vertex.y = y_center + row_temp;

        //     rotate_vertices[i] = temp_vertex;
        //     // int base_pixel = local_costmap.at<uint8_t>(y_center + row_base, x_center + col_base);
        // }

        // // for (int i = 0; i < 4; i++)
        // // {
        // //     cv::line(local_costmap, rotate_vertices[i], rotate_vertices[(i + 1) % 4], cv::Scalar(255, 255, 255), 2);
        // // }

        // // std::string filename = "/home/ray/DESPOT-LOG/local_costmap_" + std::to_string(img_count) + ".jpg";
        // // cv::imwrite(filename, local_costmap);

        // cv::RotatedRect crop_rect = cv::minAreaRect(rotate_vertices);

        // rotate_vertices.clear();

        // float rect_angle = crop_rect.angle;
        // cv::Size rect_size = crop_rect.size;
        // if (crop_rect.angle < -45.0)
        // {
        //     rect_angle += 90.0;
        //     swap(rect_size.width, rect_size.height);
        // }

        // // get the rotation matrix
        // cv::Mat affine_mat = cv::getRotationMatrix2D(crop_rect.center, rect_angle, 1.0);
        // // perform the affine transformation
        // cv::Mat rotated_costmap, cropped_costmap;
        // cv::warpAffine(local_costmap, rotated_costmap, affine_mat, local_costmap.size(), cv::INTER_CUBIC);
        // // crop the resulting image
        // cv::getRectSubPix(rotated_costmap, rect_size, crop_rect.center, cropped_costmap);

        // cv::threshold(cropped_costmap, cropped_costmap, ModelParams::outer_pixel_thres, 100, cv::THRESH_BINARY);
        // float sum_cropped = cv::sum(cropped_costmap)[0];
        // float max_cropped = 100 * cropped_costmap.total();

        // clearance_idx = sum_cropped / max_cropped;

        // clearance_idx =  powf(clearance_idx, 0.5);

        // if (user_weight > 0.9)
        // {
        //     adapt_idx = 0.1;
        // }
        // else if (user_weight < 0.15)
        // {
        //     adapt_idx = 0.85;
        // }
        // else
        // {
        //     adapt_idx = 1 - user_weight;
        // }
        adapt_idx = 1 - user_weight;

        // user_control_weight.data = adapt_idx;

        // filename = "/home/ray/DESPOT-LOG/cropped_costmap_" + std::to_string(img_count) + ".jpg";
        // cv::imwrite(filename, cropped_costmap);

        // img_count++;

        /* Update the belief */
        // Initialize previous goal positions if pre_positions is empty
        if (pre_positions.size() == 0)
        {
            cout << "Belief space Initializing." << endl;
            pre_positions = goal_positions;
            // Initialize goal belief
            belief_goal.clear();
            belief_goal.resize(goal_positions.size(), 1 / static_cast<float>(goal_positions.size()));
            // Listen to the user at the beginning
            adapt_idx = 0.1;
            wheelchair_model_->re_sort = true;
        }
        else
        {
            // New paths are appended
            if (goal_positions.size() != pre_positions.size())
            {
                cout << "New paths are appended." << endl;
                int num_changed_goal = 0;
                int num_added_goal = 0;
                float sum_changed_goal = 0;

                num_added_goal = goal_positions.size() - pre_positions.size();
                belief_goal.resize(goal_positions.size(), 0);		// assign zero probabilities to the appended new paths
                // check if previous registered paths have changed
                for (int i = 0; i < pre_positions.size(); i++)
                {
                    if (pre_positions[i].header.seq == goal_positions[i].header.seq) // not changed
                    {
                        continue;
                    }
                    else // paths have been replaced
                    {
                        num_changed_goal++; // count the number of replaced paths
                        // sum_changed_goal += particles_[i]->weight; // sum of the probabilities over replaced paths
                        sum_changed_goal += belief_goal[i];
                        belief_goal[i] = 0; // assign zero probabilities to the replaced paths
                        // particles_[i]->weight = 0;
                    }
                }
                cout << num_added_goal << " path(s) have been added, " << num_changed_goal << " path(s) have been changed." << endl;
                // distribute the probabilites from replaced paths to the remaining paths evenly
                for (int i = 0; i < pre_positions.size(); i++)
                {
                    if (pre_positions[i].header.seq == goal_positions[i].header.seq)
                    {
                        // particles_[i]->weight += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
                        belief_goal[i] += sum_changed_goal / (pre_positions.size() - num_changed_goal);
                    }
                }
                wheelchair_model_->clipBelief(belief_goal);
                wheelchair_model_->normalize(belief_goal);

                // store the current path info for the next checking process
                pre_positions = goal_positions;
            }
            else // no new paths added, only replacement
            {
                // cout << "No paths added." << endl;
                int num_changed_goal = 0;
                float sum_changed_goal = 0;
                bool path_replaced = false;
                for (int i = 0; i < goal_positions.size(); i++)
                {
                    // check if previous registered paths have changed
                    if (pre_positions[i].header.seq == goal_positions[i].header.seq)
                    {
                        continue;
                    }
                    num_changed_goal++;
                    // sum_changed_goal += particles_[i]->weight;
                    sum_changed_goal += belief_goal[i];
                    belief_goal[i] = 0;
                    // particles_[i]->weight = 0;
                    path_replaced = true;
                }
                if (path_replaced)
                {
                    cout << num_changed_goal << " path(s) have been changed." << endl;
                    for (int i = 0; i < goal_positions.size(); i++)
                    {
                        if (pre_positions[i].header.seq == goal_positions[i].header.seq)
                        {
                            belief_goal[i] += sum_changed_goal / (pre_positions.size() - num_changed_goal);
                            // particles_[i]->weight += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
                        }
                    }
                    wheelchair_model_->clipBelief(belief_goal);
                    wheelchair_model_->normalize(belief_goal);
                }
                pre_positions = goal_positions;
            }
        }

        instant_goal = belief_goal;

        double sum_rotation = 0;
        double reward;
        // Update goal belief
        float Q_rotation = 0, Pi_rotation = 0;
        // cout << "Joystick input: x = " <<  wheelchair_obs.joystick_signal.x << ", y = " << wheelchair_obs.joystick_signal.y << endl;
        // cout << "Wheelchair velocity: v = " <<  wheelchair_obs.wheelchair_twist.linear.x << ", w = " << wheelchair_obs.wheelchair_twist.angular.z << endl;
        // cout << "Before belief update: " << endl;
        // for (int i = 0; i < belief_goal.size(); ++i)
        // {
        // 	cout << "Belief " << i + 1 << ": " << belief_goal[i] << endl;
        // }
        // cout << "Updating goal belief..." << endl;
        if (fabs(joystick_input.x) > 0.1 || fabs(joystick_input.y) > 0.1)
        {	
            for (int i = 0; i < goal_positions.size(); ++i)
            {
                // cout << "Updating belief of path " << i + 1 << "..." << endl;
                Q_rotation = wheelchair_model_->calRotationValue(joystick_input.x, joystick_input.y, goal_positions[i].pose.position);
                // cout << "Q_rotation " << Q_rotation << endl;
                Pi_rotation =  exp(- ModelParams::exp_temperature * Q_rotation);	// MaxEnt IOC
                // cout << "Pi_rotation " << Pi_rotation << endl;
                belief_goal[i] *= Pi_rotation;
                sum_rotation += belief_goal[i];

                Q_rotation = instantRotationValue(joystick_input.x, joystick_input.y, goal_positions[i].pose.position);
                Pi_rotation =  exp(- ModelParams::exp_temperature * Q_rotation);	// MaxEnt IOC
                instant_goal[i] = Pi_rotation;
                // cout << "Angle discrepancy between joystick input and goal " << i + 1 << " is " << Q_rotation * 180 / M_PI << endl;
            }
            for (int i = 0; i < belief_goal.size(); ++i)
            {
                belief_goal[i] /= sum_rotation;
            }
        }

        wheelchair_model_->clipBelief(belief_goal);
        wheelchair_model_->normalize(belief_goal);
        wheelchair_model_->normalize(instant_goal);

        std::vector<float>::iterator itMax = std::max_element(belief_goal.begin(), belief_goal.end());
        int most_likely_index = std::distance(belief_goal.begin(), itMax);
        most_likely_path.data = path_list.paths[most_likely_index].header.seq;

        itMax = std::max_element(instant_goal.begin(), instant_goal.end());
        instant_index = std::distance(instant_goal.begin(), itMax);

        goal_list.poses.resize(goal_positions.size());
        goal_list.header = goal_positions[0].header;
        for (int i = 0; i < goal_positions.size(); ++i)
        {
            goal_list.poses[i] = goal_positions[i].pose;
            goal_list.poses[i].position.z = belief_goal[i];
        }

        /* Visualization part */

        std::string output_txt;
        std::stringstream adapt_index_stream;
        adapt_index_stream << std::fixed << std::setprecision(2) << user_weight;
        output_txt = adapt_index_stream.str();
        double marker_lifetime = 2.0 * ModelParams::update_interval;
        adapt_txt.lifetime = ros::Duration(marker_lifetime);
        adapt_txt.text = output_txt;
        geometry_msgs::Point cmd_line_point, pomdp_line_point;
        float theta_cmd = 0, theta_pomdp = 0;

        for (int i = 0; i < 10; i++)
        {
            cmd_line_point.x += joystick_input.x * cos(theta_cmd) * ModelParams::planning_time;
            cmd_line_point.y += joystick_input.x * sin(theta_cmd) * ModelParams::planning_time;
            theta_cmd += joystick_input.y * ModelParams::planning_time;
            cmd_line.points.push_back(cmd_line_point);

            pomdp_line_point.x += cmd_vel.linear.x * cos(theta_pomdp) * ModelParams::planning_time;
            pomdp_line_point.y += cmd_vel.linear.x * sin(theta_pomdp) * ModelParams::planning_time;
            theta_pomdp += cmd_vel.angular.z * ModelParams::planning_time;
            pomdp_line.points.push_back(pomdp_line_point);
        }

        /* Publish other topics except cmd_vel */

        ucmd_publisher_.publish(cmd_line);
        pomdp_publisher_.publish(pomdp_line);
        // weight_publisher_.publish(user_control_weight);
        path_publisher_.publish(most_likely_path);
        footprint_publisher_.publish(footprint_polygon);
        adapt_publisher_.publish(adapt_txt);
        goal_publisher_.publish(goal_list);
        adapt_txt.text.clear();
        cmd_line.points.clear();
        pomdp_line.points.clear();
        footprint_polygon.polygon.points.clear();
    }
}

//Used to update weights of the dmps
// void GazeboWheelchairWorld::update_DMP_weights()
// {
//     int num_goals = goal_positions.size();
//     wheelchair_model_->dmp_init_variables.update_weights(num_goals, wheelchair_model_->intermediate_goal_list);
    
// }
void GazeboWheelchairWorld::printGoals()
{
    std::ofstream out_file;
    std::string out_file_name = "data/Voronoi_paths_step_" + std::to_string(step_no)+ ".txt";
    out_file.open(out_file_name);
    int num_goals = goal_positions.size();
    out_file << num_goals << std::endl;
    tf2::Quaternion goal_quat;
     int path_size;
    for(int i=0;i< num_goals;i++){

        out_file << goal_positions[i].pose.position.x << " " 
        << goal_positions[i].pose.position.y << std::endl;

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
    // out_file_name = "data/Lidar_" + std::to_string(step_no)+ ".txt";
    // out_file.open(out_file_name);
    // int num_lidar_points = wheelchair_model_->lidar_points.size();
    // out_file << num_lidar_points << std::endl;
    // for(int i=0;i< num_lidar_points;i++){
    //     out_file<< wheelchair_model_->lidar_points[i].x << " "
    //     << wheelchair_model_->lidar_points[i].y << std::endl;
    // }

    // out_file.close();
}
void GazeboWheelchairWorld::generateGoal()
{
    // geometry_msgs::Point point_marker;
    // waypoint_viz.points.clear();
    ex_intermediate_goals.paths.clear();
    ex_intermediate_goals.paths.resize(path_list.paths.size());
    temp_intermediate_goals.paths.clear();
    temp_intermediate_goals.paths.resize(path_list.paths.size());
    goal_positions.clear();
    goal_positions.resize(path_list.paths.size());
    belief_points.clear();
    belief_points.resize(path_list.paths.size());
    current_pose = getGlobalAgentPose().pose;

    // tf2::Vector3 map_vector(1, 0, 0), heading_vector(1, 0, 0);
    // tf2::Quaternion rotate_quat;
    // tf2::convert(map_pose.orientation, rotate_quat);
    // map_vector = tf2::quatRotate(rotate_quat, map_vector);
    // tf2::convert(current_pose.orientation, rotate_quat);
    // heading_vector = tf2::quatRotate(rotate_quat, heading_vector);

    // agent2map_yaw = heading_vector.angle(map_vector);

    // float cross_product = heading_vector.getX() * map_vector.getY() - heading_vector.getY() * map_vector.getX();
    // agent2map_yaw = (cross_product >= 0) ? agent2map_yaw : - agent2map_yaw;
    // map_quaternion.setRPY(0, 0, - agent2map_yaw);

    for (int i = 0; i < path_list.paths.size(); i++)
    {
        ex_intermediate_goals.paths[i] = GazeboWheelchairWorld::findIntermediateGoals(current_pose, path_list.paths[i]);
        ex_intermediate_goals.paths[i].header = path_list.paths[i].header;
        goal_positions[i].header = path_list.paths[i].header;
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
    for (int i = 0; i < ex_intermediate_goals.paths.size(); i++)
    {
        temp_intermediate_goals.paths[i].poses.resize(ex_intermediate_goals.paths[i].poses.size());
        for (int j = 0; j < ex_intermediate_goals.paths[i].poses.size(); j++)
        {
            tf2::doTransform<geometry_msgs::Pose>(ex_intermediate_goals.paths[i].poses[j].pose,
                temp_intermediate_goals.paths[i].poses[j].pose, map2localTransform);
            // waypoint_viz.points.push_back(wheelchair_model_->intermediate_goal_list.paths[i].poses[j].pose.position);
        }
        temp_intermediate_goals.paths[i].header.frame_id = base_frame_id;
        goal_positions[i].pose = temp_intermediate_goals.paths[i].poses[temp_intermediate_goals.paths[i].poses.size() - 1].pose;
        goal_positions[i].header.frame_id = base_frame_id;
        if (temp_intermediate_goals.paths[i].poses.size() > 6)
        {
            belief_points[i] = temp_intermediate_goals.paths[i].poses[6].pose.position;
        }
        else
        {
            belief_points[i] = goal_positions[i].pose.position;
        }
    }

    thread_mutex.lock();
    in_intermediate_goals.paths.resize(temp_intermediate_goals.paths.size());
    in_intermediate_goals = temp_intermediate_goals;
    thread_mutex.unlock();
    // update_DMP_weights();
    //printGoals();
    //step_no++;
}

void GazeboWheelchairWorld::generateUserPath()
{
    temp_user_path.header = in_intermediate_goals.paths[0].header;
    temp_user_path.poses.clear();
    geometry_msgs::PoseStamped moving_point;
    moving_point.header = temp_user_path.header;
    tf2::Vector3 projected_joystick(joystick_input.x * ModelParams::waypoint_dist / ModelParams::max_linear_speed,
        joystick_input.y * ModelParams::waypoint_dist / ModelParams::max_angular_speed, 0);
    float projected_length = projected_joystick.length();

    float x_check = 0;
    float y_check = 0;
    int col_check = 0;
    int row_check = 0;

    if (projected_length <= 0.2) // almost no user input, generate an empty path
    {
        temp_user_path.poses.push_back(moving_point);
    }
    else    // user has input
    {
        float num_steps = projected_length / 0.2;
        float increment_x = projected_joystick.getX() / num_steps;
        float increment_y = projected_joystick.getY() / num_steps;

        // first check collision
        // tf2::Vector3 map_joystick;
        // remap the joystick to map frame
        // map_joystick = tf2::quatRotate(map_quaternion, projected_joystick);
        float map_increment_x = projected_joystick.getX() / num_steps;
        float map_increment_y = projected_joystick.getY() / num_steps;
        for (int j = round(num_steps); j > 0; --j)
        {
            x_check = j * map_increment_x;
            y_check = j * map_increment_y;

            row_check = static_cast<int>(- x_check / ModelParams::map_resolution);
            col_check = static_cast<int>(- y_check / ModelParams::map_resolution);
            

            // collision happened
            if (local_costmap.at<uint8_t>(x_center + row_check, y_center + col_check) > ModelParams::outer_pixel_thres)
            {
                dummy_goal_collision = true;
                break;
            }
            dummy_goal_collision = false;
        }

        if (dummy_goal_collision)
        {
            // dummy goal in collision, change to existing paths
            temp_user_path = in_intermediate_goals.paths[instant_index];
        }
        else
        {
            // dummy goal not in collision, generate the path
            tf2::Vector3 point_direction;
            float yaw = 0;
            tf2::Quaternion point_quat;

            geometry_msgs::Point previous_point;
            previous_point.x = 0;
            previous_point.y = 0;
            moving_point.pose.position.x = 0;
            moving_point.pose.position.y = 0;
            for (int i = 0; i < round(num_steps); ++i)
            {
                moving_point.pose.position.x += increment_x;
                moving_point.pose.position.y += increment_y;
                point_direction.setValue(moving_point.pose.position.x - previous_point.x, moving_point.pose.position.y - previous_point.y, 0);
                yaw = point_direction.angle(tf2::Vector3(1, 0, 0));
                yaw = point_direction.getY() >= 0 ? yaw : - yaw;
                point_quat.setRPY(0, 0, yaw);
                tf2::convert(point_quat, moving_point.pose.orientation);
                temp_user_path.poses.push_back(moving_point);
                previous_point.x = moving_point.pose.position.x;
                previous_point.y = moving_point.pose.position.y;
            }
        }
    }

    thread_mutex.lock();
    user_path = temp_user_path;
    thread_mutex.unlock();

    if (joystick_input.x >= -0.1)
    {
        float theta_cmd = 0;
        int sample_number = 8;
        float sample_time = 1.6;
        float sample_interval = sample_time / static_cast<float>(sample_number);
        float cmd_line_x = 0, cmd_line_y = 0;
        // float check_width = 0.3;
        // tf2::Vector3 center2left(0, check_width, 0);
        // tf2::Vector3 center2right(0, -check_width, 0);
        // tf2::Quaternion projected_quat;
        // tf2::Vector3 projected_cmd(0, 0, 0);
        for (int i = 0; i < sample_number; i++)
        {
            cmd_line_x += joystick_input.x * cos(theta_cmd) * sample_interval;
            cmd_line_y += joystick_input.x * sin(theta_cmd) * sample_interval;
            theta_cmd += joystick_input.y * sample_interval;
            // projected_cmd.setX(cmd_line_x);
            // projected_cmd.setY(cmd_line_y);
            // projected_cmd.setZ(0);
            // projected_cmd = tf2::quatRotate(map_quaternion, projected_cmd);

            // x_check = projected_cmd.getX();
            // y_check = projected_cmd.getY();

            row_check = static_cast<int>(- cmd_line_x / ModelParams::map_resolution);
            col_check = static_cast<int>(- cmd_line_y / ModelParams::map_resolution);

            // col_check = x_check >= 0 ? ceil(x_check / map_resolution) : floor(x_check / map_resolution);
            // row_check = y_check >= 0 ? ceil(y_check / map_resolution) : floor(y_check / map_resolution);

            // collision happened
            if (local_costmap.at<uint8_t>(x_center + row_check, y_center + col_check) > ModelParams::outer_pixel_thres)
            {
                projected_cmd_collision = true;
                break;
            }
            projected_cmd_collision = false;

            // if (i >= sample_number - 3)
            // {
            //     projected_quat.setRPY(0, 0, theta_cmd);
            //     center2left = tf2::quatRotate(projected_quat, center2left);
            //     center2right = tf2::quatRotate(projected_quat, center2right);
            //     float left_x = cmd_line_x + center2left.getX();
            //     float left_y = cmd_line_y + center2left.getY();
            //     float right_x = cmd_line_x + center2right.getX();
            //     float right_y = cmd_line_y + center2right.getY();
            //     projected_cmd.setX(left_x);
            //     projected_cmd.setY(left_y);
            //     projected_cmd.setZ(0);
            //     projected_cmd = tf2::quatRotate(map_quaternion, projected_cmd);
            //     x_check = projected_cmd.getX();
            //     y_check = projected_cmd.getY();

            //     col_check = x_check >= 0 ? ceil(x_check / map_resolution) : floor(x_check / map_resolution);
            //     row_check = y_check >= 0 ? ceil(y_check / map_resolution) : floor(y_check / map_resolution);
            //     // collision happened
            //     if (local_costmap.at<uint8_t>(y_center + row_check, x_center + col_check) > ModelParams::outer_pixel_thres)
            //     {
            //         projected_cmd_collision = true;
            //         break;
            //     }

            //     projected_cmd.setX(right_x);
            //     projected_cmd.setY(right_y);
            //     projected_cmd.setZ(0);
            //     projected_cmd = tf2::quatRotate(map_quaternion, projected_cmd);
            //     x_check = projected_cmd.getX();
            //     y_check = projected_cmd.getY();

            //     col_check = x_check >= 0 ? ceil(x_check / map_resolution) : floor(x_check / map_resolution);
            //     row_check = y_check >= 0 ? ceil(y_check / map_resolution) : floor(y_check / map_resolution);
            //     // collision happened
            //     if (local_costmap.at<uint8_t>(y_center + row_check, x_center + col_check) > ModelParams::outer_pixel_thres)
            //     {
            //         projected_cmd_collision = true;
            //         break;
            //     }
            // }
        }
    }
    else
    {
        projected_cmd_collision = false;
    }

    // cout << "Collision check = " << dummy_goal_collision << endl;
    // cout << "Remapped goal index = " << instant_index + 1 << endl;
    // cout << "User path, size = "<< temp_user_path.poses.size() << endl;
	// for (int i = 0; i < user_path.poses.size(); ++i)
	// {
	// 	cout << user_path.poses[i].pose.position.x << " " << user_path.poses[i].pose.position.y << endl;
	// }
}

void GazeboWheelchairWorld::updateModelInfo()
{
    // wheelchair_model_->map_resolution = map_resolution;
    wheelchair_model_->x_center = x_center;
    wheelchair_model_->y_center = y_center;
    // wheelchair_model_->agent2map_yaw = agent2map_yaw;
    // wheelchair_model_->map_quaternion = map_quaternion;
    // update joystick input
    wheelchair_model_->external_joy_x = joystick_input.x;
    wheelchair_model_->external_joy_y = joystick_input.y;
    wheelchair_state_->joystick_x = joystick_input.x;
    wheelchair_state_->joystick_y = joystick_input.y;
    // update the velocities
    wheelchair_model_->current_wheelchair_status.agent_velocity = current_vel;
    wheelchair_state_->wheelchair.agent_velocity = current_vel;

    // update goal info
    wheelchair_model_->goal_positions.clear();
    wheelchair_model_->goal_positions = goal_positions;

    thread_mutex.lock();
    // update the costmap info
    wheelchair_model_->local_costmap = local_costmap.clone();
    // update path info
    wheelchair_model_->intermediate_goal_list.paths.clear();
    wheelchair_model_->intermediate_goal_list.paths.resize(path_list.paths.size());
    wheelchair_model_->intermediate_goal_list = in_intermediate_goals;
    wheelchair_model_->user_path = user_path;
    thread_mutex.unlock();
    wheelchair_model_->dummy_goal_collision = dummy_goal_collision;
    // wheelchair_model_->projected_cmd_collision = projected_cmd_collision;
    wheelchair_model_->instant_index = instant_index;
    
    // update executed velocities
    wheelchair_model_->v_execution = cmd_vel.linear.x;
    wheelchair_model_->w_execution = cmd_vel.angular.z;

    // update path belief info
    wheelchair_model_->belief_goal = belief_goal;

    // update dwa velocities
    if (bdwa_receive)
    {
        wheelchair_model_->belief_dwa_v = bdwa_v;
        wheelchair_model_->belief_dwa_w = bdwa_w;
    }
    else
    {
        wheelchair_model_->belief_dwa_v = 0;
        wheelchair_model_->belief_dwa_w = 0;
    }
    if (pdwa_receive)
    {
        wheelchair_model_->pure_dwa_v = pdwa_v;
        wheelchair_model_->pure_dwa_w = pdwa_w;
    }
    else
    {
        wheelchair_model_->pure_dwa_v = 0;
        wheelchair_model_->pure_dwa_w = 0;
    }
    // update adaptability index
    if (weight_receive)
    {
        wheelchair_model_->adapt_idx = adapt_idx;
    }
    else
    {
        wheelchair_model_->adapt_idx = 0.1;
    }
    // wheelchair_model_->adapt_idx = 0;
}

void GazeboWheelchairWorld::PrintBelief()
{
    cout << "Final Belief:" << endl;
	for (int i = 0; i < belief_goal.size(); ++i)
	{
		cout << "Goal " << i + 1 << ", position: x = " << goal_positions[i].pose.position.x << ", y = " << goal_positions[i].pose.position.y << ", belief = " << belief_goal[i] << endl;
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

float GazeboWheelchairWorld::instantRotationValue(float x_input, float y_input, geometry_msgs::Point goal_point) const
{
	tf2::Vector3 agent2goal(goal_point.x, goal_point.y, 0);
	tf2::Vector3 point2goal(x_input, y_input, 0);
	return agent2goal.angle(point2goal);
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

        acmlt_dist += GazeboWheelchairWorld::calDistance(msg_path.poses[i - 1].pose.position.x, msg_path.poses[i - 1].pose.position.y,
            msg_path.poses[i].pose.position.x, msg_path.poses[i].pose.position.y);

        if (i == msg_path.poses.size() - 1 || acmlt_dist >= ModelParams::waypoint_dist)
        {
            goal_direction.setValue(msg_path.poses[i].pose.position.x - msg_path.poses[i - 1].pose.position.x,
                msg_path.poses[i].pose.position.y - msg_path.poses[i - 1].pose.position.y, 0);
            yaw = goal_direction.getY() >= 0 ? goal_direction.angle(tf2::Vector3(1, 0, 0)) : - goal_direction.angle(tf2::Vector3(1, 0, 0));
            goal_quat.setRPY(0, 0, yaw);
            tf2::convert(goal_quat, intermediate_goal.pose.orientation);
            intermediate_goal_list.poses.push_back(intermediate_goal);
            break;
        }
        intermediate_goal_list.poses.push_back(intermediate_goal);
    }
    return intermediate_goal_list;
}

float GazeboWheelchairWorld::calDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(powf((x1 - x2), 2) + powf((y1 - y2), 2));
}