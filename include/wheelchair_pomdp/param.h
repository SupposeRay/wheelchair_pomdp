
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include <string>
#include <math.h>
#include <vector>


namespace ModelParams {

	const double GOAL_TRAVELLED=20.0;
	const int N_PED_IN=6;
	const int N_PED_WORLD=200;

	extern double VEL_MAX;
	extern double NOISE_GOAL_ANGLE;
	extern double CRASH_PENALTY;
	extern double REWARD_FACTOR_VEL;
	extern double REWARD_BASE_CRASH_VEL;
	extern double BELIEF_SMOOTHING;
	extern double NOISE_ROBVEL;
	extern double COLLISION_DISTANCE;
	extern bool USE_ZERO_VEL_CORRECTION;

	extern double IN_FRONT_ANGLE_DEG;

	extern double LASER_RANGE;
	extern int DRIVING_PLACE;

	extern double NOISE_PED_POS;

	const double pos_rln=0.01; // position resolution
	const double vel_rln=0.001; // velocity resolution

	const double PATH_STEP = 0.05;

	const double GOAL_TOLERANCE = 0.5;

	const double PED_SPEED = 0;

	const bool debug=false;

	const double control_freq=3;
	const double AccSpeed=0.1;

	extern std::string rosns;
	extern std::string laser_frame;

    inline void init_params(bool in_simulation) {
        if(in_simulation) {
            rosns="";
            laser_frame="/laser_frame";
        } else {
            rosns="";
            laser_frame="/laser_frame";
        }
    }

    // deprecated params
    const double GOAL_REWARD = 100;

    const bool CPUDoPrint=false;

    const bool is_simulation=true;

	// New added for wheelchair model
	// POMDP planner setting
	const int num_particles = 100;						// the number of particles
    const int planning_step = 800;						// the number of steps to plan, a negative number means infinite steps
    const int max_default_policy_len = 5;				// the max number of steps to simulate defaulty policy
    const float planning_time = 0.3;					// the planning time per step, freq = 1/planning_time
    const bool enable_multithreading = false;			// whether to use multi-threaded CPU or not
    const int num_thread = 20;							// the number of CPU threads
	const bool use_GPU = false;							// whether to use GPU or not
    const int verbosity_lvl = 2;						// the log file level
	const float reward_discount = 0.95;					// the discount factor for rewards
	const float memory_time = 3.0;						// the time duration to store previous user and robot actions
	const int search_depth = 5;							// the maximum depth to expand a root node
    // std::string file_directory = "/home/ray/DESPOT-LOG/test.txt";

	// external world setting
	const float transition_time = 0.6;					// the rollout time per step, must be a multiple of planning_time
	const float max_v_acceleration = 2.0;				// the maximum linear acceleration limited by motors
	const float max_w_acceleration = 2.0;				// the maximum angular acceleration limited by motors
	const float std_v_acceleration = 1.0;				// the standard linear acceleration for normal actions
	const float std_w_acceleration = 1.0;				// the standard angular acceleration for normal actions
	const float weight_heading = 0.8;					// the weightings of heading in DWA cost function
	const float	weight_velocity = 0.2;					// the weightings of velocity in DWA cost function
	const float waypoint_dist = 2.0;					// the distance between the agent and the waypoint
	// const float inner_radius = 0.32;						// the inflation radius inside which a collision is considered to happen
	const float inner_radius = 0.45;
	// const float outer_radius = 0.35;					// the inflation radius inside which a penalty starts to be inflicted
	const float outer_radius = 0.55;
	const float blur_radius = 0.1;
	const float dilation_radius = 0.2;					// the bufferzone for collision
	const float collision_pixel = 75;					// the pixel value higher than which a collision is considered to happen
	const float outer_pixel_thres = 75;					// the pixel value higher than which a collision is considered to happen
	const float inner_pixel_thres = 74;					// the pixel value higher than which a collision during rollout is considered to happen
	const float pixel_path = 73.0;						// the max pixel value on a costmap, where a collision is surely confirmed
	const float repsonse_time = 0.2;					// the response time for the real wheelchair
	const float back2base_dist = 0.4;					// the distance between base_link and rear part of the wheelchair
	const float update_interval = 0.1;					// the update interval for the timer
	const float publish_interval = 0.01;				// the velocity publish interval for the timer
	const float inflation_width = 1.0;					// the inflation width around the wheelchair for dynmaic rectangle
	
	// POMDP model setting
	const int num_paths = 5;							// the number of planned paths, should be indentical to the parameter in Voronoi planner
	// const int num_M2G = 3;								// the number of move to goal actions used in POMDP planning
	const int num_adapt = 2;							// the number of user adaptability modes: adaptable, non-adaptable
	const int num_normal_actions = 7;					// the number of normal actions: linear+, linear-, angular+, angular-, keep, stop, following user
	const int num_simulation_m2g = 15;					// the number of steps simulated in the move_to_goal actions
	const float upper_bound = 0.9, lower_bound = 0.1;	// the thresholds for clipping the probability
	const int num_int_observations = 17; 				// the size of integer observation array used in GPU model
	const bool using_probabilistic_model = false;		// whether to introduce noise to the velocities in the step function
	const float noise_amplitude = 0.04;					// the noise amplitude when introducing noise
	const float exp_temperature = 0.3;					// the rate of updating path probabilities in MaxEnt IOC
	const float backing_ratio = 1.0;					// the proportion of max backing speed to max speed
	const float max_linear_speed = 1.0;					// the max linear speed limit, acutal max linear speed is controlled by the joystick input
	const float max_angular_speed = 1.0;				// the max angular speed
	const int num_points_direction = 3;					// the first several points along a specific path to define the direction of that path
	const float angle_receding = M_PI / 6;				// the max angle between heading and path direction beyond which a receding penalty is inflicted
	const float step_size = 0.2;						// the number added/subtracted from the current velocities when applying an action
	const float step_scale = 0.8;						// the proportion of step_size while the wheelchair is inside the collision zone
	const float facing_angle = 0.1;						// the angle threshold under which the wheelchair is considered facing the path
    const bool use_dwa_actions = true;
	const int num_dwa_actions = 2;
	const bool using_reaching_check = false;
	
	// reward part
	const int excess_speed_penalty = -100;				// the penalty for exceeding max speed
	const int step_penalty = -1;						// the penalty for every step
	const int stationary_penalty = -10;					// the penalty for not moving when joystick input is non-zero
	const int user_following_reward = 50; 				// the max reward for following joystick input
	const int collision_penalty = -5000;				// the penalty for a severe collision
	const int inflation_basic_penalty = -10;			// the basic penalty for entering the inflation zone
	const int inflation_high_penalty = -50;				// the penalty increment for approaching the inflation zone
	const int inflation_max_penalty = -1000;			// the max penalty incurred to force DESPOT to slow down in the inflation zone
	const int reaching_reward = 150;					// the reward for reaching the final goal
	const int inter_goal_reward = 10;					// the accumulative reward for reaching intermediate goals
	const int receding_basic_penalty = -1;				// the basic coefficient to penalize actions receding from paths

	// DMPs part
	const float dmp_cs_x = 1.0;							// the initial canonical dynamic system x value, going to 0 during the process
	const float dmp_tau = 1.0;							// the spatial scaling term in DMPs, used for different speeds to reach the goal
	const float dmp_error = 0.0;						// the error coupling term in DMPs

};

#endif

