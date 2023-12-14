
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include <string>
#include <math.h>


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
    const float planning_time = 0.15;					// the planning time per step, freq = 1/planning_time
    const bool enable_multithreading = true;			// whether to use multi-threaded CPU or not
    const int num_thread = 20;							// the number of CPU threads
	const bool use_GPU = false;							// whether to use GPU or not
    const int verbosity_lvl = 2;						// the log file level
	const float reward_discount = 0.95;					// the discount factor for rewards
    // std::string file_directory = "/home/ray/DESPOT-LOG/test.txt";

	// external world setting
	const float transition_time = 0.3;					// the rollout time per step, must be a multiple of planning_time
	const float waypoint_dist = 2.0;					// the distance between the agent and the waypoint
	const float inner_radius = 0.32;						// the inflation radius inside which a collision is considered to happen
	// const float inner_radius = 0.42;
	const float outer_radius = 0.35;					// the inflation radius inside which a penalty starts to be inflicted
	// const float outer_radius = 0.45;
	// const int GOAL_POSITIONS_SIZE = 10;
	
	// POMDP model setting
	const int num_paths = 5;							// the number of planned paths, should be indentical to the parameter in Voronoi planner
	const float upper_bound = 0.99, lower_bound = 0.01;	// the thresholds for clipping the probability
	const int num_int_observations = 17; 				// the size of integer observation array used in GPU model
	const bool using_probabilistic_model = false;		// whether to introduce noise to the velocities in the step function
	const float noise_amplitude = 0.04;					// the noise amplitude when introducing noise
	const float exp_temperature = 0.3;					// the rate of updating path probabilities in MaxEnt IOC
	const float backing_ratio = 0.5;					// the proportion of max backing speed to max speed
	const float max_angular_speed = 1.0;				// the max angular speed, max linear speed is controlled by the joystick input
	const int num_points_direction = 3;					// the first several points along a specific path to define the direction of that path
	const float angle_receding = M_PI / 6;				// the max angle between heading and path direction beyond which a receding penalty is inflicted

	// reward part
	const int excess_speed_penalty = -200;				// the penalty for exceeding max speed
	const int step_penalty = -1;						// the penalty for every step
	const int stationary_penalty = -50;					// the penalty for not moving when joystick input is non-zero
	const int user_following_reward = 100;				// the max reward for following joystick input
	const int collision_penalty = -1000;				// the penalty for a collision
	const int inflation_basic_penalty = -400;			// the basic penalty for entering the inflation zone
	const int reaching_reward = 300;					// the reward for reaching the final goal
	const int inter_goal_reward = 30;					// the accumulative reward for reaching intermediate goals
	const int receding_basic_penalty = -1;				// the basic coefficient to penalize actions receding from paths

};

#endif

