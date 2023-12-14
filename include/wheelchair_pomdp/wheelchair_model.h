#ifndef WHEELCHAIR_MODEL_H
#define WHEELCHAIR_MODEL_H
#include <numeric>
#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/core/particle_belief.h>
#include <despot/GPUinterface/GPUpomdp.h>
// #include <despot/util/coord.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
// #include <despot/solver/despot.h>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom Messages
#include <voronoi_msgs_and_types/PathList.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// shared_dwa

// #include "shared_dwa/shared_DWA_node.h"
#include "wheelchair_pomdp/param.h"
// #include "wheelchair_pomdp/wheelchair_dmp.h"
namespace despot
{

class WheelchairDSPOMDP;

// Particle Belief
class WheelchairParticleBelief: public ParticleBelief 
{
public:
	WheelchairParticleBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior = NULL, bool split = true);

	virtual void Update(ACT_TYPE action, OBS_TYPE obs);
		
	const WheelchairDSPOMDP* wheelchair_model_; 
};

struct WheelchairStruct
{
	// wheelchair pose
	geometry_msgs::Pose agent_pose;
	float agent_pose_angle; //Rotation around z axis, stored separately for easy transfer to GPU model

	// velocities
	geometry_msgs::Twist agent_velocity;

	 
};

struct GoalStruct
{
	float pos_x;
	float pos_y;
	float angle_z; //Orientation/angle
};

// struct ActionTuple
// {
// 	float agent_v;
// 	float agent_w;
// 	float joystick_x;
// 	float joystick_y;
// 	std::vector<geometry_msgs::Point> goal_positions;
// };

class ObservationClass
{
public:
    ObservationClass()
	{};

    ObservationClass(uint64_t obs_)
    {
        obs = obs_;
    }

    uint64_t GetHash() const
	{
        return obs;
    };

    // TODO: this function is probably wrong
    // take in hash map value and update obs
    void SetIntObs(uint64_t obs_) { 
        obs = obs_;
    };
private:
    uint64_t obs;  
};

class WheelchairObs : public ObservationClass
{
public:
	geometry_msgs::Pose wheelchair_pose;
	geometry_msgs::Twist wheelchair_twist;
	float agent_pose_angle; //Rotation around z axis, stored separately for easy transfer to GPU model

	bool continuous_obs = false;

	// discretized obs
	int joystick_obs;

	// continuous obs
	geometry_msgs::Point joystick_signal;

	// this function not very useful for now
	void getObsFromWheelchairStatus(despot::WheelchairStruct wheelchair_)
	{
		wheelchair_pose = wheelchair_.agent_pose;
		wheelchair_twist = wheelchair_.agent_velocity;
		agent_pose_angle = wheelchair_.agent_pose_angle;
		// goal_pose = state_.obj_pos;
	}
	void getObsFromInteger(uint64_t obs_wheelchair)
	{
		joystick_obs = obs_wheelchair % 100;
	}
};

/* =============================================================================
 * WheelchairState class
 * =============================================================================*/

class WheelchairState: public State
{
public:
	WheelchairStruct wheelchair;
	GoalStruct goal_details;
	float joystick_x, joystick_y;
	// float pure_dwa_v, pure_dwa_w;
	// float final_dwa_v, final_dwa_w;
	// user's desired path index, unknow part
	int path_idx;
	bool adaptability;
	// geometry_msgs::Point state_goal_point;
	// int state_goal_index;
	// float collision_idx;

	// number of intermediate goals reached
	int num_intermediate_goals;

	// the path info to each goal
	voronoi_msgs_and_types::PathList path2waypoint;

	//The path traversed by wheelchair. Used for contaction and interpolation
	nav_msgs::Path path_traversed;

	int action_length;
	int action_before;

	WheelchairState();
	WheelchairState(WheelchairStruct _wheelchair, int _path_idx, float _joystick_x = 0, float _joystick_y = 0);
	~WheelchairState();

	std::string text() const;
};

/* =============================================================================
 * WheelchairDSPOMDP class
 * =============================================================================*/

class WheelchairDSPOMDP: public DSPOMDP
{
protected:
	mutable MemoryPool<WheelchairState> memory_pool_;

	// std::vector<WheelchairState*> states_;

	// mutable std::vector<ValuedAction> mdp_policy_;

public:
	// action space
	/*
	linear_plus: linear.x + 0.5
	linear_minus: linear.x - 0.5
	angular_plus: angular.z + 0.5
	angular_minus: angular.z - 0.5
	ask: ask question
	*/
	// static const ACT_TYPE LINEAR_PLUS, LINEAR_MINUS, ANGULAR_PLUS, ANGULAR_MINUS, KEEP, STOP, FOLLOW;
	// enum {
	// 	A_linear_plus = 0,
	// 	A_linear_minus = 1,
	// 	A_angular_plus = 2,
	// 	A_angular_minus = 3,
	// 	A_ask_question = 4
	// };

	// observation space
	// enum {
	// 	OBS_NONE = 0,		// no joystick input
	// 	OBS_FRONT = 1,		// pushing forward
	// 	OBS_FR_L = 2,		// pushing front left
	// 	OBS_LEFT = 3,		// pushing left
	// 	OBS_FR_R = 4,		// pushing front right
	// 	OBS_RIGHT = 5,		// pushing right
	// };
	enum {
		OBS_FRONT = 0,		// pushing forward (-pi/16 ~ pi/16)
		OBS_FR_L = 1,		// pushing front left (pi/16 ~ 3pi/16)
		OBS_F_L = 2,		// pushing front left (3pi/16 ~ 5pi/16)
		OBS_F_LE = 3,		// pushing front left (5pi/16 ~ 7pi/16)
		OBS_LEFT = 4,		// pushing left (7pi/16 ~ 9pi/16)
		OBS_B_LE = 5,		// pushing back left (9pi/16 ~ 11pi/16)
		OBS_B_L = 6,		// pushing back left (11pi/16 ~ 13pi/16)
		OBS_BA_L = 7,		// pushing back left (13pi/16 ~ 15pi/16)
		OBS_BACK = 8,		// pushing back (15pi/16 ~ pi, -pi ~ -15pi/16)
		OBS_BA_R = 9,		// pushing back right (-15pi/16 ~ -13pi/16)
		OBS_B_R = 10,		// pushing back right (-13pi/16 ~ -11pi/16)
		OBS_B_RI = 11,		// pushing back right (-11pi/16 ~ -9pi/16)
		OBS_RIGHT = 12,		// pushing right (-9pi/16 ~ -7pi/16)
		OBS_F_RI = 13,		// pushing front right (-7pi/16 ~ -5pi/16)
		OBS_F_R = 14,		// pushing front right (-5pi/16 ~ -3pi/16)
		OBS_FR_R = 15,		// pushing front right (-3pi/16 ~ -pi/16)
		OBS_NONE = 16,		// no joystick input
	};

public:
	WheelchairDSPOMDP();
	WheelchairDSPOMDP(std::string config_file);

	// POMDP model to store the possible goal positions
	// vector of waypoints, vector size same as number of paths, data structure (x, y), used in CalAngleDiff (called in Step) and Update functions
	std::vector<geometry_msgs::PoseStamped> goal_positions;
	// previous goal positions to compare with the current, used in Update function
	// mutable std::vector<geometry_msgs::PoseStamped> pre_positions;
	// vector of intermediate goals, a series of points, data structure (x, y), used in ReachingCheck (called in Step) and Update functions
	mutable voronoi_msgs_and_types::PathList intermediate_goal_list;

	// the proability distribution of different goals
	std::vector<float> belief_goal;
	// the adaptability index
	mutable float adapt_idx;

	// a deque to store the history ActionTuple to predict user adapatability
	// mutable std::deque<ActionTuple> HRI_history;

	//DMP related variables
	// DMP_init_variables dmp_init_variables; //Used to contain constant variables of DMP

	// the current wheelchair status obtained from the external world, used for initialization
	WheelchairStruct current_wheelchair_status;
	// the current lidar points obtained from the external world, a series of points, data structure (x, y), used in CollisionCheck (called in Step) 
	std::vector<geometry_msgs::Point> lidar_points;

	// the current local costmap obtained from the external world, a matrix, data type uint8_t, used in CollisionCheck (called in Step)
	cv::Mat local_costmap;
	// map resolution [m/cell]
	// float map_resolution;
	// center cell
	int x_center = 0, y_center = 0;
	// the yaw between wheelchair heading and the costmap pose
	// float agent2map_yaw = 0;

	// store observation in hashvalue
	std::hash<std::string> obsHash;
	// using STL map here, should use HashTable if a lot of data
	mutable std::map<uint64_t, WheelchairObs> obsHashMap;

	mutable std::map<uint64_t, WheelchairObs> obsHashMap_execution;

	// the rollout time per step during tree search
    float transition_time = 0.3;

	// float variables to store the actual joystick input from the external world
	float external_joy_x = 0, external_joy_y = 0;
	
	mutable float initial_joy_x = 0, initial_joy_y = 0;

	mutable float collision_index = 0;

	//float variables to store dwa values
	float pure_dwa_v, pure_dwa_w;
	float belief_dwa_v, belief_dwa_w;

	// actual executed v and w
	float v_execution = 0, w_execution = 0;

	// updated position info afer step all particles in Update function
	// mutable float updated_position_x = 0, updated_position_y = 0;

	// updated heading info afer step all particles in Update function
	// mutable tf2::Vector3 updated_heading;

	// whether to re-sort the particles_
	mutable bool re_sort = false;

	// the quaternion to represent the rotation of the costmap with respect to the wheelchair local frame
	// mutable tf2::Quaternion map_quaternion;

	// the joystick length during the search
	mutable float initial_joystick_length = 0;

	// user's instantaneous desired path
	mutable nav_msgs::Path user_path;

	// the instant goal index
    int instant_index = 0;

	// if the instant joystick input is inside collision zone
	bool dummy_goal_collision = false;

	// if the short instant joystick input is inside collision zone
    // bool projected_cmd_collision = false;

	//===================== Essential Functions =====================//
	/* Returns total number of actions.*/
	int NumActions() const;

	/* Deterministic simulative model.*/
	bool Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const;
	
	// bool MoveToGoalStep(State& state, double& reward, OBS_TYPE& obs, float linear_x = 0, float angular_z = 0) const;

	// void MoveToGoalVel(WheelchairStruct& wheelchair_status, ACT_TYPE action, float& linear_x, float& angular_z) const;

	/* Functions related to beliefs and starting states.*/
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
	State* CreateStartState(std::string type = "DEFAULT") const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	/* Bound-related functions.*/
	double GetMaxReward() const;
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	ValuedAction GetBestAction() const;
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

	/* Memory management.*/
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	/* Display.*/
	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const;
	void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

	/* Collision check */
	float CollisionCheck(const WheelchairStruct& wheelchair_status) const;

	float M2G_CollisionCheck(const WheelchairStruct& wheelchair_status, const float& angle2turn) const;

	/* Reaching check */
	float ReachingCheck(WheelchairState& wheelchair_state, bool& reaching_goal) const;

	/* Reaching instant goal check */
	float InstantGoalReachingCheck(WheelchairState& wheelchair_state, bool& reaching_goal) const;

	/* Receding penalty */
	int RecedingPenalty(WheelchairState& wheelchair_state, tf2::Vector3& agent_heading) const;

	/* Shared control reward */
	float FollowUserReward(tf2::Vector3& follow_heading, float& v_after, float& w_after, float& follow_cost, int& action_depth) const;

	float M2G_FollowUserReward(tf2::Vector3& joystick_heading, tf2::Vector3& final_heading) const;

	/* Transition function */
	float TransitWheelchair(WheelchairState& wheelchair_state, int& transition_steps, float v_before, float v_after, float w_before, float w_after) const;

	/* Transition function for Update */
	bool TransitParticles(WheelchairState& wheelchair_state, float v_before, float v_after, float w_before, float w_after) const;

	/* Angle between wheelchair heading and direction to the goal */
	double CalAngleDiff(WheelchairStruct &wheelchair_status, tf2::Vector3& agent_heading, int &goal_idx, int point_idx = -1) const;

	float calRotationValue(float x_input, float y_input, geometry_msgs::Point goal_point) const;
	float calRotationActionCost(float angle_next) const;

	/* Convert observation into Hash value */
	void PrintObs(WheelchairObs& wheelchair_obs, std::ostream& out = std::cout) const;

	void PrintParticles(const std::vector<State*> particles, std::ostream& out = std::cout) const;
	
	void clipBelief(std::vector<despot::State *>& particles_vector) const;
	void normalize(std::vector<despot::State *>& particles_vector) const;
	void clipBelief(std::vector<float>& belief_vector) const;
	void normalize(std::vector<float>& belief_vector) const;

	// path contraction
	void ContractAndInterpolatePath(WheelchairStruct &wheelchair_status, nav_msgs::Path &path_traversed, nav_msgs::Path &original_path) const;

	// path interpolation
	void GenerateNewPath(WheelchairStruct &wheelchair_status, nav_msgs::Path &original_path) const;

	int TurningSteps(float& angle2turn, float& current_w, float& angular_vel, float time_span) const;

	float FollowingVel(tf2::Vector3 joystick_heading, float v_before, float w_before, float& v_follow, float& w_follow, float v_max, float time_span) const;

	//======================Functions for hyp-despot====================================
	int NumObservations() const
	{
		return std::numeric_limits<int>::max();
	}
	virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const ; //Implemented in GPU_wheelchair_model.cu
	
	virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const ; //Implemented in GPU_wheelchair_model.cu

	
	virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
													int num_particles, bool interleave,
													Dvc_RandomStreams* streams, int stream_pos,
													void* CUDAstream = NULL, int shift = 0) const ; //Implemented in GPU_wheelchair_model.cu
	

	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
												bool deepcopy) const ; //Implemented in GPU_wheelchair_model.cu
	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const; //Implemented in GPU_wheelchair_model.cu
	
	virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const; //Implemented in GPU_wheelchair_model.cu
	

	virtual void InitGPUModel(); //Implemented in GPU_Init.cu
	


	virtual void InitGPUUpperBound(std::string name,std::string particle_bound_name) const ; //Defined in GPU_Init.cu


	virtual void InitGPULowerBound(std::string name,std::string particle_bound_name) const; //Defined in GPU_Init.cu

	virtual void DeleteGPUModel(); //Defined in GPU_Init.cu
	
	virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name); //Defined in GPU_Init.cu

	virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) ; //Defined in GPU_Init.cu

	virtual void CreateMemoryPool() const ; //Implemented in GPU_wheelchair_model.cu
	
	virtual void DestroyMemoryPool(MEMORY_MODE mode) const ;  //Implemented in GPU_wheelchair_model.cu
	

	virtual int ParallelismInStep() const 
	{
			//std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
			return 1;
			//Used in car driving among peds to parallelize computation for peds. Retaining it here.
			//return ModelParams::N_PED_IN;

	}

	////======================Functions for hyp-despot-alpha====================================

	Dvc_State* GetPointerToParticleList(int offset, Dvc_State* full_list) const ; //Implemented in GPU_wheelchair_model.cu
	
	virtual void PrintGPUState(const Dvc_State* state, std::ostream& out = std::cout) const;

	/////==============================Additional fuctions for this model============================
	void UpdateGPUModel(); //Implemented in GPU_Init.cu. Used to update lidar points and goal positions after every step

};
} // namespace despot

#endif
