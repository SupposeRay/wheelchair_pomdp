#include "GPU_Init.h"

#include <despot/GPUconfig.h>
#include <despot/GPUcore/CudaInclude.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/GPUcore/GPUbuiltin_lower_bound.h>
#include <despot/GPUcore/GPUbuiltin_policy.h>


#include "GPU_wheelchair_model.h"
#include "GPU_WheelchairUpperBound.h"
#include "GPU_WheelchairLowerBound.h"

#include <despot/solver/Hyp_despot.h>

#include <wheelchair_pomdp/wheelchair_model.h>
#include <vector>
//#include <simulator.h>
using namespace despot;
using namespace std;

static Dvc_WheelchairPomdp* Dvc_pomdpmodel=NULL;
static Dvc_PedPomdpParticleLowerBound* b_smart_lowerbound=NULL; //Needed to calculate the value at lef nodes with default policy defined.
static Dvc_PedPomdpParticleUpperBound1* upperbound=NULL;
//static Dvc_PedPomdpSmartPolicy* smart_lowerbound=NULL;
static Dvc_PedPomdpDoNothingPolicy* do_nothing_lowerbound=NULL;

//static Dvc_COORD* tempGoals=NULL;
//static Dvc_COORD* tempPath=NULL;
static Dvc_3DCOORD* tempLidarPoints=NULL;
static Dvc_COORD* tempGoalPositions= NULL;
static Dvc_Path* tempIntermediateGoalList=NULL;
static int* tempLocalCostMap= NULL;
// static float tempExternalJoystick_x = 0;
// static float tempExternalJoystick_y = 0;

//void UpdateGPUGoals(DSPOMDP* Hst_model);
//void UpdateGPUPath(DSPOMDP* Hst_model);
void UpdateGPULidarPoints(DSPOMDP* Hst_model);
void UpdateGPUExternalJoystick(DSPOMDP* Hst_model);
void UpdateGPUGoalPositionsAndIntermediateGoalList(DSPOMDP* Hst_model);
void UpdateGPULocalCostmap(DSPOMDP* Hst_model);
void UpdateGPUAfterExcecuteAction(DSPOMDP* Hst_model);  

__global__ void PassPedPomdpFunctionPointers(Dvc_WheelchairPomdp* model)
{
	DvcModelStepIntObs_=&(model->Dvc_Step);
	DvcModelObsProbIntObs_=&(model->Dvc_ObsProbInt);
	DvcModelCopyNoAlloc_=&(model->Dvc_Copy_NoAlloc);
	DvcModelCopyToShared_=&(model->Dvc_Copy_ToShared);
	DvcModelGet_=&(model->Dvc_Get);
	DvcModelGetBestAction_=&(model->Dvc_GetBestAction);
	DvcModelGetMaxReward_=&(model->Dvc_GetMaxReward);
	DvcModelNumActions_ = &(model->NumActions);
	DvcModelGetCarVel_ = &(model->Dvc_GetCarVel);
	DvcModelPrintDvcState_ = &(model->Dvc_PrintDvcState);
}

__global__ void PassPedPomdpParams(	//double _in_front_angle_cos, 
		double _freq,
		double _transition_time,
		bool _using_probabilistic_model,
		float _noise_amplitude,
		float _backing_ratio,
		float _max_linear_speed,
		float _max_angular_speed,
		int _excess_speed_penalty,
		int _step_penalty,
		int _stationary_penalty,
		int _user_following_reward,
		int _collision_penalty,
		int _inflation_basic_penalty,
		int _reaching_reward,
		int _inter_goal_reward,
		float _step_size,
		float _max_v_acceleration,
		float _max_w_acceleration,
		float _weight_heading,
		float _weight_velocity,
		float _response_time,
		float _outer_pixel_thres,					
	    float _inner_pixel_thres,
		float _pixel_path,
		int _inflation_max_penalty,
		float _reward_discount,
		int _num_simulation_m2g,
		int _num_normal_actions,
		float _facing_angle,

		//Dvc_3DCOORD* _lidar_points,
		int _lidar_points_size,
		//Dvc_COORD* _goals, 
		//Dvc_COORD* _path, 
		//int pathsize,
		int goal_positions_size,
		double GOAL_TRAVELLED,
		int N_PED_IN,
		int N_PED_WORLD,
		double VEL_MAX,
		double NOISE_GOAL_ANGLE,
		double CRASH_PENALTY,
		double REWARD_FACTOR_VEL,
		double REWARD_BASE_CRASH_VEL,
		double BELIEF_SMOOTHING,
		double NOISE_ROBVEL,
		double COLLISION_DISTANCE,
		double IN_FRONT_ANGLE_DEG,
		double LASER_RANGE,
		double pos_rln, // position resolution
		double vel_rln, // velocity resolution
		double PATH_STEP,
		double GOAL_TOLERANCE,
		double PED_SPEED,
		bool debug,
		double control_freq,
		double AccSpeed,
		double GOAL_REWARD,
		bool USE_ZERO_VEL_CORRECTION)
{
	//Declared in GPU_wheelchair_model.h
	//in_front_angle_cos=_in_front_angle_cos;
	freq=_freq;
	Dvc_ModelParams::transition_time = _transition_time;
	//goals=_goals;
	//if(path == NULL) path=new Dvc_Path();
	//lidar_points = _lidar_points;
	Dvc_ModelParams::using_probabilistic_model = _using_probabilistic_model;
	Dvc_ModelParams::noise_amplitude = _noise_amplitude;
	Dvc_ModelParams::backing_ratio = _backing_ratio;
	Dvc_ModelParams::max_angular_speed = _max_angular_speed;
	Dvc_ModelParams::max_linear_speed = _max_linear_speed;
	Dvc_ModelParams::excess_speed_penalty = _excess_speed_penalty;
	Dvc_ModelParams::step_penalty = _step_penalty;
	Dvc_ModelParams::stationary_penalty = _stationary_penalty;
	Dvc_ModelParams::user_following_reward = _user_following_reward;
	Dvc_ModelParams::collision_penalty = _collision_penalty;
	Dvc_ModelParams::inflation_basic_penalty = _inflation_basic_penalty;
	Dvc_ModelParams::reaching_reward = _reaching_reward;
	Dvc_ModelParams::inter_goal_reward = _inter_goal_reward;
	Dvc_ModelParams::step_size = _step_size;
	Dvc_ModelParams::max_v_acceleration = _max_v_acceleration;
	Dvc_ModelParams::max_w_acceleration = _max_w_acceleration;
	Dvc_ModelParams::weight_velocity = _weight_velocity;
	Dvc_ModelParams::weight_heading = _weight_heading;
	Dvc_ModelParams::repsonse_time = _response_time;
	Dvc_ModelParams::outer_pixel_thres = _outer_pixel_thres;
	Dvc_ModelParams::inner_pixel_thres = _inner_pixel_thres;
	Dvc_ModelParams::pixel_path = _pixel_path;
	Dvc_ModelParams::inflation_max_penalty = _inflation_max_penalty;
	Dvc_ModelParams::reward_discount = _reward_discount;
	Dvc_ModelParams::num_simulation_m2g = _num_simulation_m2g;
	Dvc_ModelParams::num_normal_actions = _num_normal_actions;
	Dvc_ModelParams::facing_angle = _facing_angle;

	Dvc_ModelParams::LIDAR_POINTS_SIZE = _lidar_points_size;
	Dvc_ModelParams::GOAL_TRAVELLED =  GOAL_TRAVELLED ;
	Dvc_ModelParams::N_PED_IN = N_PED_IN  ;
	Dvc_ModelParams::N_PED_WORLD = N_PED_WORLD  ;
	Dvc_ModelParams::VEL_MAX =  VEL_MAX ;
	Dvc_ModelParams::NOISE_GOAL_ANGLE = NOISE_GOAL_ANGLE  ;
	Dvc_ModelParams::CRASH_PENALTY =  CRASH_PENALTY ;
	Dvc_ModelParams::REWARD_FACTOR_VEL = REWARD_FACTOR_VEL  ;
	Dvc_ModelParams::REWARD_BASE_CRASH_VEL =  REWARD_BASE_CRASH_VEL ;
	Dvc_ModelParams::BELIEF_SMOOTHING =   BELIEF_SMOOTHING;
	Dvc_ModelParams::NOISE_ROBVEL =NOISE_ROBVEL;
	Dvc_ModelParams::COLLISION_DISTANCE =  COLLISION_DISTANCE ;
	Dvc_ModelParams::IN_FRONT_ANGLE_DEG = IN_FRONT_ANGLE_DEG  ;
	Dvc_ModelParams::LASER_RANGE =  LASER_RANGE ;
	Dvc_ModelParams::pos_rln =  pos_rln ; // position resolution
	Dvc_ModelParams::vel_rln =  vel_rln ; // velocity resolution
	Dvc_ModelParams::PATH_STEP =  PATH_STEP ;
	Dvc_ModelParams::GOAL_TOLERANCE =  GOAL_TOLERANCE ;
	Dvc_ModelParams::PED_SPEED = PED_SPEED  ;
	Dvc_ModelParams::debug =  debug ;
	Dvc_ModelParams::control_freq =  control_freq ;
	Dvc_ModelParams::AccSpeed =  AccSpeed ;
	Dvc_ModelParams::GOAL_REWARD = GOAL_REWARD  ;
	Dvc_ModelParams::USE_ZERO_VEL_CORRECTION = USE_ZERO_VEL_CORRECTION;
	
	printf("Pass model to gpu\n");
	printf("Control frequency in GPU = %.3fHz\n", freq);
	printf("Transition time in GPU = %.3fs\n", Dvc_ModelParams::transition_time);

}

/*__global__ void UpdatePathKernel(Dvc_COORD* _path, int pathsize)
{
	if(path) {delete path; path=new Dvc_Path();}
	if(path==NULL) 	path=new Dvc_Path();

	path->size_=pathsize;
	path->pos_=0;
	path->way_points_=_path;
	printf("pass path to gpu %d\n", path);
}
*/
void WheelchairDSPOMDP::InitGPUModel(){
	WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(this);


	HANDLE_ERROR(cudaMallocManaged((void**)&Dvc_pomdpmodel, sizeof(Dvc_WheelchairPomdp)));

	PassPedPomdpFunctionPointers<<<1,1,1>>>(Dvc_pomdpmodel);
	HANDLE_ERROR(cudaDeviceSynchronize());

	//World related variables are being loaded to GPU here. Need to update it with voronoi path related variables and lidarpoints

	/* logd <<"Hst->world_model->path.size()= "<< Hst->world_model->path.size()<< endl;

	if(tempPath==NULL && Hst->world_model->path.size()>0){

		HANDLE_ERROR(cudaMallocManaged((void**)&tempPath, Hst->world_model->path.size()*sizeof(Dvc_COORD)));
	}
	if(tempGoals==NULL && Hst->world_model->goals.size()>0)
	HANDLE_ERROR(cudaMallocManaged((void**)&tempGoals,  Hst->world_model->goals.size()*sizeof(Dvc_COORD)));

	*/

	int lidar_points_size = Hst->lidar_points.size();
	/*if(tempLidarPoints == NULL && lidar_points_size > 0)
	{
		HANDLE_ERROR(cudaMallocManaged((void**)&tempLidarPoints,  lidar_points_size*sizeof(Dvc_3DCOORD)));
	}
	*/
	int goal_positions_size = ModelParams::num_paths;
	
	float control_freq = 1.0/Globals::config.time_per_move;
	
	

	PassPedPomdpParams<<<1,1,1>>>(
		//Hst->world_model->in_front_angle_cos, Hst->world_model->freq, tempGoals, tempPath,
		//Hst->world_model->path.size(),
		control_freq,
		ModelParams::transition_time,
		ModelParams::using_probabilistic_model,
		ModelParams::noise_amplitude,
		ModelParams::backing_ratio,
		ModelParams::max_linear_speed,
		ModelParams::max_angular_speed,
		ModelParams::excess_speed_penalty,
		ModelParams::step_penalty,
		ModelParams::stationary_penalty,
		ModelParams::user_following_reward,
		ModelParams::collision_penalty,
		ModelParams::inflation_basic_penalty,
		ModelParams::reaching_reward,
		ModelParams::inter_goal_reward,
		ModelParams::step_size,
		ModelParams::max_v_acceleration,
		ModelParams::max_w_acceleration,
		ModelParams::weight_heading,
		ModelParams::weight_velocity,
		ModelParams::repsonse_time,
		ModelParams::outer_pixel_thres,
		ModelParams::inner_pixel_thres,
		ModelParams::pixel_path,
		ModelParams::inflation_max_penalty,
		ModelParams::reward_discount,
		ModelParams::num_simulation_m2g,
		ModelParams::num_normal_actions,
		ModelParams::facing_angle,
		
		//tempLidarPoints,
		lidar_points_size,
		goal_positions_size,
		ModelParams::GOAL_TRAVELLED,
		ModelParams::N_PED_IN,
		ModelParams::N_PED_WORLD,
		ModelParams::VEL_MAX,
		ModelParams::NOISE_GOAL_ANGLE,
		ModelParams::CRASH_PENALTY,
		ModelParams::REWARD_FACTOR_VEL,
		ModelParams::REWARD_BASE_CRASH_VEL,
		ModelParams::BELIEF_SMOOTHING,
		ModelParams::NOISE_ROBVEL,
		ModelParams::COLLISION_DISTANCE,
		ModelParams::IN_FRONT_ANGLE_DEG,
		ModelParams::LASER_RANGE,
		ModelParams::pos_rln, // position resolution
		ModelParams::vel_rln, // velocity resolution
		ModelParams::PATH_STEP,
		ModelParams::GOAL_TOLERANCE,
		ModelParams::PED_SPEED,
		ModelParams::debug,
		ModelParams::control_freq,
		ModelParams::AccSpeed,
		ModelParams::GOAL_REWARD,
		ModelParams::USE_ZERO_VEL_CORRECTION);
	
	HANDLE_ERROR(cudaDeviceSynchronize());
	// cout << "Transition time " << transition_time << "s." << endl;
	// cout << "Control frequency control_freq " << control_freq << "Hz." << endl;
	//UpdateGPUGoals(Hst);
	//UpdateGPUPath(Hst);
	// UpdateGPULidarPoints(Hst);
	// UpdateGPUGoalPositionsAndIntermediateGoalList(Hst); 
	// UpdateGPUExternalJoystick(Hst); 
	// UpdateGPULocalCostmap(Hst);
	UpdateGPUAfterExcecuteAction(Hst);  
	HANDLE_ERROR(cudaDeviceSynchronize());

}

void WheelchairDSPOMDP::UpdateGPUModel(){
	WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(this);
	// UpdateGPULidarPoints(Hst);
	// UpdateGPUGoalPositionsAndIntermediateGoalList(Hst);
	// UpdateGPUExternalJoystick(Hst);
	UpdateGPUAfterExcecuteAction(Hst);    
	HANDLE_ERROR(cudaDeviceSynchronize());
}

__global__ void PassActionValueFuncs(
		Dvc_PedPomdpParticleUpperBound1* upperbound)
{
	DvcUpperBoundValue_ = &(upperbound->Value);
}

void WheelchairDSPOMDP::InitGPUUpperBound(string name,
		string particle_bound_name) const{
	HANDLE_ERROR(cudaMalloc((void**)&upperbound, sizeof(Dvc_PedPomdpParticleUpperBound1)));

	PassActionValueFuncs<<<1,1,1>>>(upperbound);

	HANDLE_ERROR(cudaDeviceSynchronize());
}



/*
__global__ void PassPedPomdpPolicyFuncPointers(Dvc_PedPomdpSmartPolicy* lowerbound)
{
	DvcDefaultPolicyAction_=&(lowerbound->Action);
	DvcLowerBoundValue_=&(lowerbound->Value);
}
*/
__global__ void PassPedPomdpPolicyFuncPointers(Dvc_PedPomdpDoNothingPolicy* lowerbound)
{
	DvcDefaultPolicyAction_=&(lowerbound->Action);
	DvcLowerBoundValue_=&(lowerbound->Value);
}

__global__ void PassPedPomdpPlbFuncPointers(Dvc_PedPomdpParticleLowerBound* b_lowerbound)
{
	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);
}


void WheelchairDSPOMDP::InitGPULowerBound(string name,
		string particle_bound_name) const{
	//if(name=="DONOTHING")
	{
		HANDLE_ERROR(cudaMallocManaged((void**)&do_nothing_lowerbound, sizeof(Dvc_PedPomdpDoNothingPolicy)));

		PassPedPomdpPolicyFuncPointers<<<1,1,1>>>(do_nothing_lowerbound);
	}
	//else
	//{
	//	HANDLE_ERROR(cudaMallocManaged((void**)&smart_lowerbound, sizeof(Dvc_PedPomdpSmartPolicy)));

	//	PassPedPomdpPolicyFuncPointers<<<1,1,1>>>(smart_lowerbound);
	//}
	HANDLE_ERROR(cudaDeviceSynchronize());

	HANDLE_ERROR(cudaMallocManaged((void**)&b_smart_lowerbound, sizeof(Dvc_PedPomdpParticleLowerBound)));

	PassPedPomdpPlbFuncPointers<<<1,1,1>>>(b_smart_lowerbound);

	HANDLE_ERROR(cudaDeviceSynchronize());
}




void WheelchairDSPOMDP::DeleteGPUModel()
{
	  HANDLE_ERROR(cudaFree(Dvc_pomdpmodel));

	  //if(tempGoals)HANDLE_ERROR(cudaFree(tempGoals));
	  //if(tempPath)HANDLE_ERROR(cudaFree(tempPath));
	  if(tempLidarPoints)HANDLE_ERROR(cudaFree(tempLidarPoints));
}

void WheelchairDSPOMDP::DeleteGPUUpperBound(string name,
		string particle_bound_name)
{
	  HANDLE_ERROR(cudaFree(upperbound));
}

void WheelchairDSPOMDP::DeleteGPULowerBound(string name,
		string particle_bound_name)
{
	  //if(smart_lowerbound)HANDLE_ERROR(cudaFree(smart_lowerbound));
	  if(do_nothing_lowerbound) HANDLE_ERROR(cudaFree(do_nothing_lowerbound));
	  if(b_smart_lowerbound)HANDLE_ERROR(cudaFree(b_smart_lowerbound));
}

/*__global__ void UpdateGoalKernel(Dvc_COORD* _goals)
{
	goals=_goals;
}
*/

__global__ void UpdateLidarPointsKernel(Dvc_3DCOORD* _lidar_points, int _lidar_points_size)
{

	Dvc_ModelParams::LIDAR_POINTS_SIZE = _lidar_points_size;
	lidar_points=_lidar_points;
	//for(int i=0;i<_lidar_points_size;i++)
	//{
	//	printf("(%d. %f , %f)",i,
	//	lidar_points[i].x, lidar_points[i].y);
	//}
	//printf("\n");
}


__global__ void UpdateGoalPositionsAndIntermediateGoalListKernel(Dvc_COORD* _goal_positions, Dvc_Path* _temp_intermediate_goal_list, int _num_path_size)
{
	Dvc_ModelParams::GOAL_POSITIONS_SIZE = _num_path_size;
	intermediate_goal_list = _temp_intermediate_goal_list;
	goal_positions = _goal_positions;
	// for(int i=0;i<_num_path_size;i++)
	// {
	// 	printf("(%d. %d )",i,intermediate_goal_list[i].size_);
	// 	printf("(%d. %f,, %f )",i,goal_positions[i].x,goal_positions[i].y );

	// 	for(int j=0;j<intermediate_goal_list[i].size_;j++)
	// 	{
	// 		printf("(%d. %f,, %f )",j,intermediate_goal_list[i].way_points_[j].x,intermediate_goal_list[i].way_points_[j].y );
	// 	}
	// 	printf("\n");
	// }
	// printf("\n");

}

__global__ void UpdateExternalJoystickKernel(float _external_joy_x, float _external_joy_y)
{
	external_joy_x = _external_joy_x;
	external_joy_y = _external_joy_y;
}

__global__ void UpdateLocalCostmapKernel(int* tempLocalCostMap, int _local_costmap_rows, int _local_costmap_cols)
{
	local_costmap_cols = _local_costmap_cols;
	local_costmap_rows = _local_costmap_rows;
	local_costmap_data = tempLocalCostMap;
	// map_resolution = _map_resolution;
	x_center = local_costmap_cols % 2 == 0 ? (local_costmap_cols / 2) - 1 : (local_costmap_cols - 1) / 2;
    y_center = local_costmap_rows % 2 == 0 ? (local_costmap_rows / 2) - 1 : (local_costmap_rows - 1) / 2;



}

__global__ void UpdateAfterExecuteAction(Dvc_3DCOORD* _lidar_points, int _lidar_points_size,
Dvc_COORD* _goal_positions, Dvc_Path* _temp_intermediate_goal_list, int _num_path_size,
float _external_joy_x, float _external_joy_y,
int* _tempLocalCostMap, int _local_costmap_rows, int _local_costmap_cols)
{
	Dvc_ModelParams::LIDAR_POINTS_SIZE = _lidar_points_size;
	lidar_points=_lidar_points;
	
	Dvc_ModelParams::GOAL_POSITIONS_SIZE = _num_path_size;
	intermediate_goal_list = _temp_intermediate_goal_list;
	goal_positions = _goal_positions;

	external_joy_x = _external_joy_x;
	external_joy_y = _external_joy_y;

	// agent2map_yaw = _agent2map_yaw;

	local_costmap_cols = _local_costmap_cols;
	local_costmap_rows = _local_costmap_rows;
	local_costmap_data = _tempLocalCostMap;
	// map_resolution = _map_resolution;

	x_center = local_costmap_cols % 2 == 0 ? (local_costmap_cols / 2) - 1 : (local_costmap_cols - 1) / 2;
    y_center = local_costmap_rows % 2 == 0 ? (local_costmap_rows / 2) - 1 : (local_costmap_rows - 1) / 2;



}
/*void UpdateGPUGoals(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU){
		WheelchairPomdp* Hst =static_cast<WheelchairPomdp*>(Hst_model);
		if(tempGoals)HANDLE_ERROR(cudaFree(tempGoals));

		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "goal list size: " << Hst->world_model->goals.size()<< endl;
		HANDLE_ERROR(cudaMallocManaged((void**)&tempGoals,  Hst->world_model->goals.size()*sizeof(Dvc_COORD)));


		for(int i=0;i<Hst->world_model->goals.size();i++){
			tempGoals[i].x=Hst->world_model->goals[i].x;
			tempGoals[i].y=Hst->world_model->goals[i].y;
		}
		UpdateGoalKernel<<<1,1,1>>>(tempGoals);
		HANDLE_ERROR(cudaDeviceSynchronize());
	}

}



void UpdateGPUPath(DSPOMDP* Hst_model)
{

	if(Globals::config.useGPU){
		PedPomdp* Hst =static_cast<PedPomdp*>(Hst_model);

		if(tempPath)HANDLE_ERROR(cudaFree(tempPath));
		HANDLE_ERROR(cudaMallocManaged((void**)&tempPath, Hst->world_model->path.size()*sizeof(Dvc_COORD)));

		for(int i=0;i<Hst->world_model->path.size();i++){
			tempPath[i].x=Hst->world_model->path[i].x;
			tempPath[i].y=Hst->world_model->path[i].y;
		}

		UpdatePathKernel<<<1,1,1>>>(tempPath,Hst->world_model->path.size());
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
	//exit(-1);
}
*/

void UpdateGPUAfterExcecuteAction(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU)
	{
		WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(Hst_model);
		if(tempLidarPoints)HANDLE_ERROR(cudaFree(tempLidarPoints));

		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "lidar points list size: " << Hst->lidar_points.size()<< endl;
		if(Hst->lidar_points.size() > 0)
		{
			HANDLE_ERROR(cudaMallocManaged((void**)&tempLidarPoints,  Hst->lidar_points.size()*sizeof(Dvc_3DCOORD)));


			for(int i=0;i<Hst->lidar_points.size();i++){
				tempLidarPoints[i].x=Hst->lidar_points[i].x;
				tempLidarPoints[i].y=Hst->lidar_points[i].y;
				tempLidarPoints[i].z=Hst->lidar_points[i].z;
			}
			//UpdateLidarPointsKernel<<<1,1,1>>>(tempLidarPoints, Hst->lidar_points.size());
			//HANDLE_ERROR(cudaDeviceSynchronize());
		}





		int goal_positions_size = ModelParams::num_paths;
		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "Goal positions size: " << goal_positions_size << endl;
		if(goal_positions_size > 0)
		{
			if(tempIntermediateGoalList)
			{
				for(int i = 0; i < goal_positions_size; i++)
				{
					if(tempIntermediateGoalList[i].way_points_)
					{
						HANDLE_ERROR(cudaFree(tempIntermediateGoalList[i].way_points_));
					}
				}

				HANDLE_ERROR(cudaFree(tempIntermediateGoalList));
			}
			if(tempGoalPositions)
			{
				HANDLE_ERROR(cudaFree(tempGoalPositions));
			}
			

			
			HANDLE_ERROR(cudaMallocManaged((void**)&tempGoalPositions,  goal_positions_size*sizeof(Dvc_COORD)));
			
			HANDLE_ERROR(cudaMallocManaged((void**)&tempIntermediateGoalList,  goal_positions_size*sizeof(Dvc_Path)));


			int path_size;
			tf2::Quaternion goal_quat;
			for(int i=0;i<goal_positions_size;i++){

				tempGoalPositions[i].x = Hst->goal_positions[i].pose.position.x;
				tempGoalPositions[i].y = Hst->goal_positions[i].pose.position.y;

				path_size = Hst->intermediate_goal_list.paths[i].poses.size();
				tempIntermediateGoalList[i].size_ = path_size;
				tempIntermediateGoalList[i].way_points_ = NULL;
				HANDLE_ERROR(cudaMallocManaged((void**)&tempIntermediateGoalList[i].way_points_,  path_size*sizeof(Dvc_3DCOORD)));
				for(int j=0; j < path_size; j++)
				{
					tempIntermediateGoalList[i].way_points_[j].x = Hst->intermediate_goal_list.paths[i].poses[j].pose.position.x;
					tempIntermediateGoalList[i].way_points_[j].y = Hst->intermediate_goal_list.paths[i].poses[j].pose.position.y;
					tf2::convert(Hst->intermediate_goal_list.paths[i].poses[j].pose.orientation, goal_quat);
					tempIntermediateGoalList[i].way_points_[j].z = tf2::getYaw(goal_quat);

				}
				
			}
			//UpdateGoalPositionsAndIntermediateGoalListKernel<<<1,1,1>>>(tempGoalPositions, tempIntermediateGoalList, goal_positions_size);
			//HANDLE_ERROR(cudaDeviceSynchronize());
		}



		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "Joystick input: x = " << Hst->external_joy_x << ", y = " << Hst->external_joy_y << endl;
		//UpdateExternalJoystickKernel<<<1,1,1>>>(temp_external_joy);
		//HANDLE_ERROR(cudaDeviceSynchronize());

		if(tempLocalCostMap)HANDLE_ERROR(cudaFree(tempLocalCostMap));

		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "local cost map rows,  cols: " << Hst->local_costmap.rows << " " <<  Hst->local_costmap.cols << endl;
		int local_costmap_rows = Hst->local_costmap.rows;
		int local_costmap_cols = Hst->local_costmap.cols;
		int local_costmap_size = local_costmap_rows*local_costmap_cols;
		if(local_costmap_size > 0)
		{
			HANDLE_ERROR(cudaMallocManaged((void**)&tempLocalCostMap,  local_costmap_size*sizeof(int)));


			//tempLocalCostMap = Hst->local_costmap.data();
			//int*local_costmap_data_  = &Hst->local_costmap.at<int>(0);
			int* local_costmap_data_ = (int*)(Hst->local_costmap.data);
			for(int i=0;i<local_costmap_size;i++){
				tempLocalCostMap[i] = local_costmap_data_[i];
			}
			UpdateLocalCostmapKernel<<<1,1,1>>>(tempLocalCostMap, local_costmap_rows, local_costmap_cols);

		}
		
		
		UpdateAfterExecuteAction<<<1,1,1>>>(tempLidarPoints, Hst->lidar_points.size(), tempGoalPositions, tempIntermediateGoalList, 
		goal_positions_size, Hst->external_joy_x, Hst->external_joy_y, 
		tempLocalCostMap, local_costmap_rows, local_costmap_cols);
		HANDLE_ERROR(cudaDeviceSynchronize());
	}
}

void UpdateGPULidarPoints(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU){
		WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(Hst_model);
		if(tempLidarPoints)HANDLE_ERROR(cudaFree(tempLidarPoints));

		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "lidar points list size: " << Hst->lidar_points.size()<< endl;
		if(Hst->lidar_points.size() > 0)
		{
			HANDLE_ERROR(cudaMallocManaged((void**)&tempLidarPoints,  Hst->lidar_points.size()*sizeof(Dvc_3DCOORD)));


			for(int i=0;i<Hst->lidar_points.size();i++){
				tempLidarPoints[i].x=Hst->lidar_points[i].x;
				tempLidarPoints[i].y=Hst->lidar_points[i].y;
				tempLidarPoints[i].z=Hst->lidar_points[i].z;
			}
			UpdateLidarPointsKernel<<<1,1,1>>>(tempLidarPoints, Hst->lidar_points.size());
			HANDLE_ERROR(cudaDeviceSynchronize());
		}
	}

}

void UpdateGPUExternalJoystick(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU){
		
		WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(Hst_model);
		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "Joystick input: x = " << Hst->external_joy_x << ", y = " << Hst->external_joy_y << endl;
		UpdateExternalJoystickKernel<<<1,1,1>>>(Hst->external_joy_x, Hst->external_joy_y);
		HANDLE_ERROR(cudaDeviceSynchronize());

	}
}

void UpdateGPUGoalPositionsAndIntermediateGoalList(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU){
		
		WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(Hst_model);
		int goal_positions_size = ModelParams::num_paths;
		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "Goal positions size: " << goal_positions_size << endl;
		if(goal_positions_size > 0)
		{
			if(tempIntermediateGoalList)
			{
				for(int i = 0; i < goal_positions_size; i++)
				{
					if(tempIntermediateGoalList[i].way_points_)
					{
						HANDLE_ERROR(cudaFree(tempIntermediateGoalList[i].way_points_));
					}
				}

				HANDLE_ERROR(cudaFree(tempIntermediateGoalList));
			}
			if(tempGoalPositions)
			{
				HANDLE_ERROR(cudaFree(tempGoalPositions));
			}
			

			
			HANDLE_ERROR(cudaMallocManaged((void**)&tempGoalPositions,  goal_positions_size*sizeof(Dvc_COORD)));
			
			HANDLE_ERROR(cudaMallocManaged((void**)&tempIntermediateGoalList,  goal_positions_size*sizeof(Dvc_Path)));


			int path_size;
			tf2::Quaternion goal_quat;
			for(int i=0;i<goal_positions_size;i++){

				tempGoalPositions[i].x = Hst->goal_positions[i].pose.position.x;
				tempGoalPositions[i].y = Hst->goal_positions[i].pose.position.y;

				path_size = Hst->intermediate_goal_list.paths[i].poses.size();
				tempIntermediateGoalList[i].size_ = path_size;
				tempIntermediateGoalList[i].way_points_ = NULL;
				HANDLE_ERROR(cudaMallocManaged((void**)&tempIntermediateGoalList[i].way_points_,  path_size*sizeof(Dvc_3DCOORD)));
				for(int j=0; j < path_size; j++)
				{
					tempIntermediateGoalList[i].way_points_[j].x = Hst->intermediate_goal_list.paths[i].poses[j].pose.position.x;
					tempIntermediateGoalList[i].way_points_[j].y = Hst->intermediate_goal_list.paths[i].poses[j].pose.position.y;
					tf2::convert(Hst->intermediate_goal_list.paths[i].poses[j].pose.orientation, goal_quat);
					tempIntermediateGoalList[i].way_points_[j].z = tf2::getYaw(goal_quat);

				}
				
			}
			UpdateGoalPositionsAndIntermediateGoalListKernel<<<1,1,1>>>(tempGoalPositions, tempIntermediateGoalList, goal_positions_size);
			HANDLE_ERROR(cudaDeviceSynchronize());
		}
	}

}

void UpdateGPULocalCostmap(DSPOMDP* Hst_model)
{
	if(Globals::config.useGPU){
		WheelchairDSPOMDP* Hst =static_cast<WheelchairDSPOMDP*>(Hst_model);
		if(tempLocalCostMap)HANDLE_ERROR(cudaFree(tempLocalCostMap));

		cout << __FUNCTION__ << "@" << __LINE__ << endl;
		cout << "local cost map rows,  cols: " << Hst->local_costmap.rows << " " <<  Hst->local_costmap.cols << endl;
		int local_costmap_rows = Hst->local_costmap.rows;
		int local_costmap_cols = Hst->local_costmap.cols;
		int local_costmap_size = local_costmap_rows*local_costmap_cols;
		if(local_costmap_size > 0)
		{
			HANDLE_ERROR(cudaMallocManaged((void**)&tempLocalCostMap,  local_costmap_size*sizeof(int)));


			//tempLocalCostMap = Hst->local_costmap.data();
			//int*local_costmap_data_  = &Hst->local_costmap.at<int>(0);
			int* local_costmap_data_ = (int*)Hst->local_costmap.data;
			for(int i=0;i<local_costmap_size;i++){
				tempLocalCostMap[i] = local_costmap_data_[i];
			}
			UpdateLocalCostmapKernel<<<1,1,1>>>(tempLocalCostMap, local_costmap_rows, local_costmap_cols);
			HANDLE_ERROR(cudaDeviceSynchronize());
		}
	}

}