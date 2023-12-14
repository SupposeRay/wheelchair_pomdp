
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
	DEVICE extern float max_angular_speed;			        	// the max angular speed
	DEVICE extern float max_linear_speed;						// the max linear speed limit, acutal max linear speed is controlled by the joystick input
	// reward part
	DEVICE extern int excess_speed_penalty;				        // the penalty for exceeding max speed
	DEVICE extern int step_penalty;					        	// the penalty for every step
	DEVICE extern int stationary_penalty;				    	// the penalty for not moving when joystick input is non-zero
	DEVICE extern int user_following_reward;				    // the max reward for following joystick input
	DEVICE extern int collision_penalty;					    // the penalty for a collision
	DEVICE extern int inflation_basic_penalty;			        // the basic penalty for entering the inflation zone
	DEVICE extern int reaching_reward;					        // the reward for reaching the final goal
	DEVICE extern int inter_goal_reward;					    // the accumulative reward for reaching intermediate goals



	// New added for wheelchair model 12 Jan 2023
	// POMDP planner setting
	//DEVICE extern int num_particles = 100;						// the number of particles
    //DEVICE extern int planning_step = 800;						// the number of steps to plan, a negative number means infinite steps
    //DEVICE extern int max_default_policy_len;				// the max number of steps to simulate defaulty policy
    //DEVICE extern float planning_time;					// the planning time per step, freq = 1/planning_time
    //DEVICE extern bool enable_multithreading = false;			// whether to use multi-threaded CPU or not
    //DEVICE extern int num_thread = 20;							// the number of CPU threads
	//DEVICE extern bool use_GPU = false;							// whether to use GPU or not
    //DEVICE extern int verbosity_lvl = 2;						// the log file level
	DEVICE extern float reward_discount;					// the discount factor for rewards
	//DEVICE extern float memory_time = 3.0;						// the time duration to store previous user and robot actions
    // std::string file_directory = "/home/ray/DESPOT-LOG/test.txt";

	// external world setting
	DEVICE extern float transition_time;					// the rollout time per step, must be a multiple of planning_time
	DEVICE extern float max_v_acceleration;				// the maximum linear acceleration limited by motors
	DEVICE extern float max_w_acceleration ;				// the maximum angular acceleration limited by motors
	DEVICE extern float weight_heading;					// the weightings of heading in DWA cost function, used for follow action
	DEVICE extern float	weight_velocity ;					// the weightings of velocity in DWA cost function, used for follow action





	DEVICE extern float waypoint_dist;					// the distance between the agent and the waypoint
	// DEVICE extern float inner_radius = 0.32;						// the inflation radius inside which a collision is considered to happen
	DEVICE extern float inner_radius;
	// DEVICE extern float outer_radius = 0.35;					// the inflation radius inside which a penalty starts to be inflicted
	DEVICE extern float outer_radius ;
	DEVICE extern float blur_radius ;
	DEVICE extern float dilation_radius ;					// the bufferzone for collision
	DEVICE extern float outer_pixel_thres ;					// the pixel value higher than which a collision is considered to happen
	DEVICE extern float inner_pixel_thres ;					// the pixel value higher than which a collision during rollout is considered to happen
	DEVICE extern float pixel_path ;						// the max pixel value on a costmap, where a collision is surely confirmed
	DEVICE extern float repsonse_time ;					// the response time for the real wheelchair
	
	// POMDP model setting
	DEVICE extern int num_paths;							// the number of planned paths, should be indentical to the parameter in Voronoi planner
	DEVICE extern int num_adapt;							// the number of user adaptability modes: adaptable, non-adaptable
	DEVICE extern int num_normal_actions;					// the number of normal actions: linear+, linear-, angular+, angular-, keep, stop, following user
	DEVICE extern int num_simulation_m2g;					// the number of steps simulated in the move_to_goal actions
	//DEVICE extern float upper_bound;
	//DEVICE extern float lower_bound;	// the thresholds for clipping the probability
	DEVICE extern int num_int_observations; 				// the size of integer observation array used in GPU model
	DEVICE extern bool using_probabilistic_model;		// whether to introduce noise to the velocities in the step function
	DEVICE extern float noise_amplitude;					// the noise amplitude when introducing noise
	//DEVICE extern float exp_temperature ;					// the rate of updating path probabilities in MaxEnt IOC
	//DEVICE extern float backing_ratio = 0.5;					// the proportion of max backing speed to max speed
	//DEVICE extern float max_angular_speed = 1.0;				// the max angular speed, max linear speed is controlled by the joystick input
	DEVICE extern int num_points_direction ;					// the first several points along a specific path to define the direction of that path
	DEVICE extern float angle_receding ;				// the max angle between heading and path direction beyond which a receding penalty is inflicted
	
	DEVICE extern float step_size;						// the number added/subtracted from the current velocities when applying an action
	DEVICE extern float step_scale ;						// the proportion of step_size while the wheelchair is inside the collision zone
	DEVICE extern float facing_angle ;						// the angle threshold under which the wheelchair is considered facing the path

	// reward part
	//DEVICE extern int excess_speed_penalty;				// the penalty for exceeding max speed
	//DEVICE extern int step_penalty;						// the penalty for every step
	//DEVICE extern int stationary_penalty;					// the penalty for not moving when joystick input is non-zero
	//DEVICE extern int user_following_reward;				// the max reward for following joystick input
	//DEVICE extern int collision_penalty;				// the penalty for a severe collision
	//DEVICE extern int inflation_basic_penalty;			// the basic penalty for entering the inflation zone
	DEVICE extern int inflation_high_penalty;				// the penalty increment for approaching the inflation zone
	DEVICE extern int inflation_max_penalty;			// the max penalty incurred to force DESPOT to slow down in the inflation zone
	//DEVICE extern int reaching_reward;					// the reward for reaching the final goal
	//DEVICE extern int inter_goal_reward;					// the accumulative reward for reaching intermediate goals
	DEVICE extern int receding_basic_penalty;				// the basic coefficient to penalize actions receding from paths

	// DMPs part
	//DEVICE extern float dmp_cs_x = 1.0;							// the initial canonical dynamic system x value, going to 0 during the process
	//DEVICE extern float dmp_tau = 1.0;							// the spatial scaling term in DMPs, used for different speeds to reach the goal
	//DEVICE extern float dmp_error = 0.0;						// the error coupling term in DMPs
};

#endif

