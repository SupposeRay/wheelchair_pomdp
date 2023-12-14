
#ifndef GPUMODELPARAMS_H
#define GPUMODELPARAMS_H
#include <string>
#include <despot/GPUcore/CudaInclude.h>


namespace Dvc_ModelParams {

	DEVICE extern double GOAL_TRAVELLED;
	DEVICE extern int N_PED_IN;
    DEVICE extern int N_PED_WORLD;

	DEVICE extern double VEL_MAX;
    DEVICE extern double NOISE_GOAL_ANGLE;
    DEVICE extern double CRASH_PENALTY;
    DEVICE extern double REWARD_FACTOR_VEL;
    DEVICE extern double REWARD_BASE_CRASH_VEL;
    DEVICE extern double BELIEF_SMOOTHING;
    DEVICE extern double NOISE_ROBVEL;
    DEVICE extern double COLLISION_DISTANCE;
    DEVICE extern bool USE_ZERO_VEL_CORRECTION;
    DEVICE extern double IN_FRONT_ANGLE_DEG;

    DEVICE extern double LASER_RANGE;

	DEVICE extern double pos_rln; // position resolution
	DEVICE extern double vel_rln; // velocity resolution

    DEVICE extern double PATH_STEP;

    DEVICE extern double GOAL_TOLERANCE;

    DEVICE extern double PED_SPEED;

	DEVICE extern bool debug;

	DEVICE extern double control_freq;
	DEVICE extern double AccSpeed;

    // deprecated params
	DEVICE extern double GOAL_REWARD;

    DEVICE extern int LIDAR_POINTS_SIZE;
    DEVICE extern int GOAL_POSITIONS_SIZE; //stored in num_paths
    DEVICE extern int MAX_INTERMEDIATE_GOAL_SIZE; //Not being used

    // New added for wheelchair model	
	// POMDP model setting
	DEVICE extern bool using_probabilistic_model;	        	// whether to introduce noise to the velocities in the step function
	DEVICE extern float noise_amplitude;				    	// the noise amplitude when introducing noise
	DEVICE extern float backing_ratio;					        // the proportion of max backing speed to max speed
	DEVICE extern float max_angular_speed;			        	// the max angular speed, max linear speed is controlled by the joystick input

	// reward part
	DEVICE extern int excess_speed_penalty;				        // the penalty for exceeding max speed
	DEVICE extern int step_penalty;					        	// the penalty for every step
	DEVICE extern int stationary_penalty;				    	// the penalty for not moving when joystick input is non-zero
	DEVICE extern int user_following_reward;				    // the max reward for following joystick input
	DEVICE extern int collision_penalty;					    // the penalty for a collision
	DEVICE extern int inflation_basic_penalty;			        // the basic penalty for entering the inflation zone
	DEVICE extern int reaching_reward;					        // the reward for reaching the final goal
	DEVICE extern int inter_goal_reward;					    // the accumulative reward for reaching intermediate goals
};

#endif

