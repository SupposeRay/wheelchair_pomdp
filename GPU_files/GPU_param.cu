#include"GPU_param.h"

namespace Dvc_ModelParams {
	DEVICE double GOAL_TRAVELLED;
	DEVICE int N_PED_IN;
    DEVICE int N_PED_WORLD;

	DEVICE double VEL_MAX;
    DEVICE double NOISE_GOAL_ANGLE;
    DEVICE double CRASH_PENALTY;
    DEVICE double REWARD_FACTOR_VEL;
    DEVICE double REWARD_BASE_CRASH_VEL;
    DEVICE double BELIEF_SMOOTHING;
    DEVICE double NOISE_ROBVEL;
    DEVICE double COLLISION_DISTANCE;
    DEVICE bool USE_ZERO_VEL_CORRECTION;
    DEVICE double IN_FRONT_ANGLE_DEG;

    DEVICE double LASER_RANGE;

	DEVICE double pos_rln; // position resolution
	DEVICE double vel_rln; // velocity resolution

    DEVICE double PATH_STEP;

    DEVICE double GOAL_TOLERANCE;

    DEVICE double PED_SPEED;

	DEVICE bool debug;

	DEVICE double control_freq;
	DEVICE double AccSpeed;

    // deprecated params
	DEVICE double GOAL_REWARD;

    DEVICE int LIDAR_POINTS_SIZE;
    DEVICE int GOAL_POSITIONS_SIZE;

    // New added for wheelchair model	
	// POMDP model setting
	DEVICE bool using_probabilistic_model;		        // whether to introduce noise to the velocities in the step function
	DEVICE float noise_amplitude;				        // the noise amplitude when introducing noise
	DEVICE float backing_ratio;				        	// the proportion of max backing speed to max speed
	DEVICE float max_angular_speed;			        	// the max angular speed
	DEVICE float max_linear_speed;						// the max linear speed limit, acutal max linear speed is controlled by the joystick input
	DEVICE float facing_angle ;							// the angle threshold under which the wheelchair is considered facing the path

	// reward part
	DEVICE int excess_speed_penalty;				    // the penalty for exceeding max speed
	DEVICE int step_penalty;						    // the penalty for every step
	DEVICE int stationary_penalty;				        // the penalty for not moving when joystick input is non-zero
	DEVICE int user_following_reward;				    // the max reward for following joystick input
	DEVICE int collision_penalty;					    // the penalty for a collision
	DEVICE int inflation_basic_penalty;			        // the basic penalty for entering the inflation zone
	DEVICE int reaching_reward;					        // the reward for reaching the final goal
	DEVICE int inter_goal_reward;					    // the accumulative reward for reaching intermediate goals

	DEVICE float step_size;						// the number added/subtracted from the current velocities when applying an action

	DEVICE float transition_time;					// the rollout time per step, must be a multiple of planning_time
	DEVICE float max_v_acceleration;				// the maximum linear acceleration limited by motors
	DEVICE float max_w_acceleration ;

	DEVICE float weight_heading;					// the weightings of heading in DWA cost function, used for follow action
	DEVICE float weight_velocity ;					// the weightings of velocity in DWA cost function, used for follow action

	DEVICE float outer_pixel_thres ;					// the pixel value higher than which a collision is considered to happen
	DEVICE float inner_pixel_thres ;					// the pixel value higher than which a collision during rollout is considered to happen
	DEVICE float pixel_path ;						// the max pixel value on a costmap, where a collision is surely confirmed
	
	DEVICE float repsonse_time ;					// the response time for the real wheelchair

	DEVICE int inflation_high_penalty;				// the penalty increment for approaching the inflation zone
	DEVICE int inflation_max_penalty;			// the max penalty incurred to force DESPOT to slow down in the inflation zone
	//DEVICE extern int reaching_reward;					// the reward for reaching the final goal
	//DEVICE extern int inter_goal_reward;					// the accumulative reward for reaching intermediate goals
	DEVICE int receding_basic_penalty;				// the basic coefficient to penalize actions receding from paths

	DEVICE float reward_discount;					// the discount factor for rewards

	DEVICE int num_paths;							// the number of planned paths, should be indentical to the parameter in Voronoi planner
	DEVICE int num_adapt;							// the number of user adaptability modes: adaptable, non-adaptable
	DEVICE int num_normal_actions;					// the number of normal actions: linear+, linear-, angular+, angular-, keep, stop, following user
	DEVICE int num_simulation_m2g;					// the number of steps simulated in the move_to_goal actions


	
}

