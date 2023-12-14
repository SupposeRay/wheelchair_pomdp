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
	DEVICE float max_angular_speed;			        	// the max angular speed, max linear speed is controlled by the joystick input

	// reward part
	DEVICE int excess_speed_penalty;				    // the penalty for exceeding max speed
	DEVICE int step_penalty;						    // the penalty for every step
	DEVICE int stationary_penalty;				        // the penalty for not moving when joystick input is non-zero
	DEVICE int user_following_reward;				    // the max reward for following joystick input
	DEVICE int collision_penalty;					    // the penalty for a collision
	DEVICE int inflation_basic_penalty;			        // the basic penalty for entering the inflation zone
	DEVICE int reaching_reward;					        // the reward for reaching the final goal
	DEVICE int inter_goal_reward;					    // the accumulative reward for reaching intermediate goals
}

