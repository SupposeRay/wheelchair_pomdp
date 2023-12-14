#ifndef GPUWheelchairPomdp_H
#define GPUWheelchairPomdp_H

#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUinterface/GPUlower_bound.h>

#include <despot/GPUutil/GPUcoord.h>
#include <despot/GPUcore/CudaInclude.h>
#include "GPU_param.h"
#include "wheelchair_pomdp/wheelchair_model.h"
#include "GPU_Path.h"
using namespace despot;

/* =============================================================================
 * Dvc_Observation class
 * =============================================================================*/

/*class ObservationClass
{
public:
    ObservationClass()
	{};

    ObservationClass(uint64_t obs_)
    {
        obs = obs_;
    }
private:
    uint64_t obs;  
};

class Dvc_WheelchairObs : public ObservationClass
*/
/* =============================================================================
 * Dvc_PomdpObs class
 * =============================================================================*/
struct Dvc_WheelchairObs
{
public:
	int obs_pos_x;
	int obs_pos_y;
	int obs_angle_z;
	int obs_vel_v;
	int obs_vel_w;

	// discretised into steps (1, LARGE_STEP)
	int joystick_obs;

	DEVICE Dvc_WheelchairObs(int* obs)
	{
		obs_pos_x = obs[1];
		obs_pos_y = obs[2];
		obs_angle_z = obs[3];
		obs_vel_v = obs[4];
		obs_vel_w = obs[5];
		joystick_obs = obs[6];
	}

	/*void getObsFromWheelchairStatus(Dvc_WheelchairStruct wheelchair_)
	{
		obs_pos_x = round(wheelchair_.pos_x * 100) / 100;
		obs_pos_y = round(wheelchair_.pos_y * 100) / 100;
		obs_angle_z = round(wheelchair_.angle_z * 100) / 100;
		obs_vel_v = round(wheelchair_.vel_v * 100) / 100;
		obs_vel_w = round(wheelchair_.vel_w * 100) / 100;
	}*/
};

/* =============================================================================
 * Dvc_PomdpState class
 * =============================================================================*/
struct Dvc_PedStruct {
	DEVICE Dvc_PedStruct() {
		vel = Dvc_ModelParams::PED_SPEED;
	}
	DEVICE Dvc_PedStruct(Dvc_COORD a, int b, int c) {
		pos = a;
		goal = b;
		id = c;
		vel = Dvc_ModelParams::PED_SPEED;
	}
	Dvc_COORD pos; //pos
	int goal;  //goal
	int id;   //id
	double vel;
};
struct Dvc_WheelchairStruct { //Equivalent to WheelchairStruct in CPU model
	float pos_x;
	float pos_y;
	float angle_z; //Orientation/angle
	float vel_v;
	float vel_w;
	//float dist_travelled;
	
};

struct Dvc_GoalStruct { //Equivalent to WheelchairStruct in CPU model
	float pos_x;
	float pos_y;
	float angle_z; //Orientation/angle
	//float vel_v;
	//float vel_w;
	//float dist_travelled;
	
};

//class WheelchairState;
class Dvc_PomdpState: public Dvc_State { //Equivalent to WheelchairState in CPU model
public:
	Dvc_WheelchairStruct wheelchair;
	Dvc_GoalStruct goal;  //Not needed if we have path_idx
	int path_idx; //Used to get the intermediate waypoints for a given goal
	int num; //Used as number of peds. Set to 0. Not used in our current code.
	Dvc_PedStruct* peds/*[Dvc_ModelParams::N_PED_IN]*/;

	DEVICE Dvc_PomdpState();

	DEVICE Dvc_PomdpState(const Dvc_PomdpState& src);

	DEVICE void Init_peds()
	{
		if (peds == NULL)
		{
			peds = new Dvc_PedStruct[Dvc_ModelParams::N_PED_IN];
			memset((void*)peds, 0, Dvc_ModelParams::N_PED_IN * sizeof(Dvc_PedStruct));
		}
	}

	DEVICE Dvc_PomdpState& operator=(const Dvc_PomdpState& other) // copy assignment
	{
		if (this != &other) { // self-assignment check expected
			// storage can be reused
			num = other.num;

			goal.pos_x = other.goal.pos_x;
			goal.pos_y = other.goal.pos_y;
			goal.angle_z= other.goal.angle_z;
			//car.dist_travelled = other.car.dist_travelled;
			//car.pos = other.car.pos;
			//car.vel = other.car.vel;
			wheelchair.pos_x = other.wheelchair.pos_x;
			wheelchair.pos_y = other.wheelchair.pos_y;
			wheelchair.angle_z = other.wheelchair.angle_z;
			wheelchair.vel_v = other.wheelchair.vel_v;
			wheelchair.vel_w = other.wheelchair.vel_w;
			

			for (int i = 0; i < num; i++)
			{
				peds[i].goal = other.peds[i].goal;
				peds[i].id = other.peds[i].id;
				peds[i].pos.x = other.peds[i].pos.x;
				peds[i].pos.y = other.peds[i].pos.y;
				peds[i].vel = other.peds[i].vel;
			}
		}
		return *this;
	}


	DEVICE ~Dvc_PomdpState()
	{
	}

	HOST static void CopyMainStateToGPU(Dvc_PomdpState* Dvc, int scenarioID, const WheelchairState*);
	HOST static void CopyPedsToGPU(Dvc_PomdpState* Dvc, int NumParticles, bool deep_copy = true);
	HOST static void ReadMainStateBackToCPU(const Dvc_PomdpState*, WheelchairState*);
	HOST static void ReadPedsBackToCPU(const Dvc_PomdpState* Dvc, std::vector<State*>, bool deep_copy = true);

};
class Dvc_WheelchairPomdp: public Dvc_DSPOMDP {

public:
	DEVICE Dvc_WheelchairPomdp(/*int size, int obstacles*/);
	DEVICE ~Dvc_WheelchairPomdp();

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

	DEVICE static bool Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	                            int* obs);
	DEVICE static float Dvc_ObsProbInt(int* obs, Dvc_State& state, int action);
	DEVICE static int NumActions();
	DEVICE static float ObsProb(OBS_TYPE obs, const Dvc_State& state, int action);
	DEVICE static int Dvc_NumObservations();

	DEVICE Dvc_State* Allocate(int state_id, double weight) const;
	DEVICE static Dvc_State* Dvc_Get(Dvc_State* particles, int pos);
	DEVICE static float Dvc_GetCarVel(Dvc_State* particles, int pos);
	DEVICE static Dvc_State* Dvc_Alloc( int num);
	DEVICE static Dvc_State* Dvc_Copy(const Dvc_State* particle, int pos);
	DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des = true);
	DEVICE static void Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des = true);
	DEVICE static void Dvc_Free(Dvc_State* particle);

	DEVICE static Dvc_ValuedAction Dvc_GetBestAction();

	DEVICE static float Dvc_GetMaxReward() {return Dvc_ModelParams::GOAL_REWARD;};

	DEVICE static int Dvc_ReachingCheck(const Dvc_PomdpState& wheelchair_state) ;

	DEVICE static float Dvc_CollisionCheck(const Dvc_WheelchairStruct& wheelchair_status) ;

	DEVICE static float Dvc_TransitWheelchair(Dvc_WheelchairStruct& wheelchair_status) ;
	
	/* Angle between wheelchair heading and direction to the goal */
	DEVICE static float Dvc_CalAngleDiff(const Dvc_WheelchairStruct& wheelchair_status, int &goal_idx);

	DEVICE static void Dvc_PrintObs(Dvc_WheelchairObs& wheelchair_obs, long& obs_int) ;
	DEVICE static void Dvc_PrintDvcState(Dvc_State* particle);
	enum {
		LINEAR_PLUS, LINEAR_MINUS, ANGULAR_PLUS, ANGULAR_MINUS, KEEP
	};
};


//World model parameters from CPU
//Commenting the ones used in Car Drive
//DEVICE extern Dvc_Path* path;
//DEVICE extern Dvc_COORD* goals;
DEVICE extern double freq;
DEVICE extern float transition_time;

//DEVICE extern double in_front_angle_cos;


//Adding the ones used in wheelchair model
DEVICE extern Dvc_3DCOORD* lidar_points;
DEVICE extern Dvc_COORD* goal_positions;
DEVICE extern Dvc_Path* intermediate_goal_list;
DEVICE extern float external_joy_x;
DEVICE extern float external_joy_y;


//DEVICE extern std::vector<long> ObsMap;

#endif
