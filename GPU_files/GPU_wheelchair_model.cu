#include "GPU_wheelchair_model.h"
#include <despot/GPUcore/thread_globals.h>

#include <wheelchair_pomdp/wheelchair_model.h>
#include <despot/util/coord.h>
#include <driver_types.h>
#include <stddef.h>
#include "despot/GPUutil/GPUmemorypool.h"
#include "despot/GPUutil/GPUrandom.h"

#include "GPU_WheelchairUpperBound.h"


#define THREADDIM 128
using namespace std;

using namespace despot;
using namespace Globals;


static GPU_MemoryPool<Dvc_PomdpState>* gpu_mainstate_pool_=NULL;
static GPU_MemoryPool<Dvc_PedStruct>* gpu_ped_pool_=NULL;

static Dvc_PedStruct **Dvc_tempPeds=NULL;
static Dvc_PedStruct **Hst_tempPeds=NULL;

static float** Dvc_temp_weight=NULL;
static int** Hst_temp_IDs=NULL;
static float** Hst_temp_weight=NULL;
static Dvc_PomdpState** Hst_temp_mainstates=NULL;
static Dvc_PomdpState* Managed_rootnode_particles=NULL;


//DEVICE Dvc_Path* path=NULL;
DEVICE Dvc_COORD* goals=NULL;
DEVICE double freq=0;
// DEVICE float transition_time=0;
DEVICE double in_front_angle_cos=0;
DEVICE Dvc_3DCOORD* lidar_points=NULL;
DEVICE Dvc_COORD* goal_positions=NULL;
DEVICE Dvc_Path* intermediate_goal_list=NULL;
DEVICE float external_joy_x = 0;
DEVICE float external_joy_y = 0;
DEVICE float agent2map_yaw = 0;
DEVICE float map_resolution = 0;
DEVICE int* local_costmap_data = NULL;
DEVICE int local_costmap_rows = 0;
DEVICE int local_costmap_cols = 0;
DEVICE int x_center = 0;
DEVICE int y_center = 0;



using namespace despot;
/* ==============================================================================
 * Dvc_PomdpState class
 * ==============================================================================*/

DEVICE Dvc_PomdpState::Dvc_PomdpState():num(0), peds(NULL)
{
	contracted_path.size = 0;
	path_traversed.size = 0;
}

DEVICE Dvc_PomdpState::Dvc_PomdpState(const Dvc_PomdpState& src)
{
	*this=src;
}


/**
 * CopyPeds_to_Particles kernel:
 * Copy pedestrian states in a combined source list (in contigeous memory) to destination particles
 * This is for copying back to CPU
 */

__global__ void CopyPeds_to_Particles(Dvc_PomdpState* dvc_particles,  Dvc_PedStruct* src)
{
	int scenarioID=blockIdx.x;
	int ped_id=threadIdx.x;

	Dvc_PomdpState* Dvc_i=dvc_particles+scenarioID;
	Dvc_PedStruct* Src_i=src+scenarioID*Dvc_ModelParams::N_PED_IN;

	if(ped_id<Dvc_i->num)
		Dvc_i->peds[ped_id]=Src_i[ped_id];



}


/**
 * CopyPeds_to_list kernel:
 * Copy pedestrian states in particles into a combined contigoues memory list
 * This is for copying back to CPU
 */

__global__ void CopyPeds_to_list(const Dvc_PomdpState* particles,  Dvc_PedStruct* peds_list)
{
	int scenarioID=blockIdx.x;
	int ped_id=threadIdx.x;
	const Dvc_PomdpState* Dvc_i = particles + scenarioID;
	Dvc_PedStruct* Des_i = peds_list + scenarioID * Dvc_ModelParams::N_PED_IN;

	if(ped_id<Dvc_i->num)
		Des_i[ped_id]=Dvc_i->peds[ped_id];
}

HOST void Dvc_PomdpState::CopyMainStateToGPU(Dvc_PomdpState* dvc_particles, int scenarioID, const WheelchairState* hst_particle)
{
	//dvc_particles[scenarioID].car.dist_travelled=hst_particle->car.dist_travelled;

	dvc_particles[scenarioID].wheelchair.pos_x=hst_particle->wheelchair.agent_pose.position.x;
	dvc_particles[scenarioID].wheelchair.pos_y=hst_particle->wheelchair.agent_pose.position.y;
	dvc_particles[scenarioID].wheelchair.angle_z=hst_particle->wheelchair.agent_pose_angle;
	
	
	// dvc_particles[scenarioID].goal.pos_x=hst_particle->goal_details.pos_x;
	// dvc_particles[scenarioID].goal.pos_y=hst_particle->goal_details.pos_y;
	// dvc_particles[scenarioID].goal.angle_z=hst_particle->goal_details.angle_z;

	dvc_particles[scenarioID].wheelchair.vel_v=hst_particle->wheelchair.agent_velocity.linear.x ;
	dvc_particles[scenarioID].wheelchair.vel_w=hst_particle->wheelchair.agent_velocity.angular.z ;
	
	dvc_particles[scenarioID].path_idx=hst_particle->path_idx;
	dvc_particles[scenarioID].joystick_x=hst_particle->joystick_x;
	dvc_particles[scenarioID].joystick_y=hst_particle->joystick_y;

	dvc_particles[scenarioID].path_traversed.size = hst_particle->path_traversed.poses.size();
	
	for (int i = 0; i < dvc_particles[scenarioID].path_traversed.size; i ++)
	{
		dvc_particles[scenarioID].path_traversed.poses[i].x = hst_particle->path_traversed.poses[i].pose.position.x;
		dvc_particles[scenarioID].path_traversed.poses[i].y = hst_particle->path_traversed.poses[i].pose.position.y;
	}

	dvc_particles[scenarioID].adaptability = hst_particle->adaptability;
	// dvc_particles[scenarioID].collision_idx = hst_particle->collision_idx;

	dvc_particles[scenarioID].num_intermediate_goals = hst_particle->num_intermediate_goals;
	dvc_particles[scenarioID].num= 0; //hst_particle->num; Making it zero. When we add dynamic scene, this should be set back to hst_particle->num



	dvc_particles[scenarioID].weight=hst_particle->weight;
	dvc_particles[scenarioID].state_id=hst_particle->state_id;
	dvc_particles[scenarioID].scenario_id=hst_particle->scenario_id;
	
	

	// cout << "Path index in CopyMainStateToGPU: " << dvc_particles[scenarioID].path_idx << endl;


	//Removing copying of peds for now
	/*
	int Data_block_size=ModelParams::N_PED_IN;

	if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
	{
		memcpy((void*)(Hst_tempPeds[GetCurrentStream()]+Data_block_size*scenarioID),
				(const void*)hst_particle->peds,
				Data_block_size*sizeof(Dvc_PedStruct));
	}
	else
	{
		memcpy((void*)(Hst_tempPeds[0]+Data_block_size*scenarioID),
				(const void*)hst_particle->peds,
				Data_block_size*sizeof(Dvc_PedStruct));
	}
	*/
}
HOST void Dvc_PomdpState::CopyPedsToGPU(Dvc_PomdpState* dvc_particles, int NumParticles, bool deep_copy)
{
	if(deep_copy)
	{
		int Data_size=NumParticles*ModelParams::N_PED_IN;
		dim3 grid1(NumParticles,1);dim3 threads1(ModelParams::N_PED_IN,1);
		if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
		{
			HANDLE_ERROR(cudaMemcpyAsync((void*)Dvc_tempPeds[GetCurrentStream()],
					(const void*)Hst_tempPeds[GetCurrentStream()],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyHostToDevice,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]));
			logd << "dvc_particles=" << dvc_particles<< ",Dvc_tempPeds[i]=" << Dvc_tempPeds[GetCurrentStream()] <<", GetCurrentStream()="<< GetCurrentStream()<< endl;

			CopyPeds_to_Particles<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]>>>
					(dvc_particles,Dvc_tempPeds[GetCurrentStream()]);
		}
		else
		{
			HANDLE_ERROR(cudaMemcpy((void*)Dvc_tempPeds[0],
					(const void*)Hst_tempPeds[0],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyHostToDevice));

			logd << "dvc_particles=" << dvc_particles<< ",Dvc_tempPeds[0]=" << Dvc_tempPeds[0]<< endl;
			CopyPeds_to_Particles<<<grid1, threads1>>>(dvc_particles,Dvc_tempPeds[0]);
		}

		//HANDLE_ERROR( cudaDeviceSynchronize());

	}
}

HOST void Dvc_PomdpState::ReadMainStateBackToCPU(const Dvc_PomdpState* dvc_particles, WheelchairState* hst_particle)
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy((void*)Hst_temp_mainstates[ThreadID], (const void*)dvc_particles, sizeof(Dvc_PomdpState), cudaMemcpyDeviceToHost));
	//hst_particle->car.dist_travelled=Hst_temp_mainstates[ThreadID]->car.dist_travelled;
	//hst_particle->car.pos=Hst_temp_mainstates[ThreadID]->car.pos;
	//hst_particle->car.vel=Hst_temp_mainstates[ThreadID]->car.vel;
	hst_particle->wheelchair.agent_pose.position.x = Hst_temp_mainstates[ThreadID]->wheelchair.pos_x;
	hst_particle->wheelchair.agent_pose.position.y = Hst_temp_mainstates[ThreadID]->wheelchair.pos_y;
	hst_particle->wheelchair.agent_pose_angle = Hst_temp_mainstates[ThreadID]->wheelchair.angle_z;
	
	
	// hst_particle->goal_details.pos_x = Hst_temp_mainstates[ThreadID]->goal.pos_x;
	// hst_particle->goal_details.pos_y = Hst_temp_mainstates[ThreadID]->goal.pos_y;
	// hst_particle->goal_details.angle_z = Hst_temp_mainstates[ThreadID]->goal.angle_z;

	hst_particle->wheelchair.agent_velocity.linear.x = Hst_temp_mainstates[ThreadID]->wheelchair.vel_v;
	hst_particle->wheelchair.agent_velocity.angular.z = Hst_temp_mainstates[ThreadID]->wheelchair.vel_w;


	hst_particle->path_idx=Hst_temp_mainstates[ThreadID]->path_idx;
	hst_particle->joystick_x=Hst_temp_mainstates[ThreadID]->joystick_x;
	hst_particle->joystick_y=Hst_temp_mainstates[ThreadID]->joystick_y;


	hst_particle->path_traversed.poses.clear();
	
	for (int i = 0; i < Hst_temp_mainstates[ThreadID]->path_traversed.size; i ++)
	{
		geometry_msgs::PoseStamped pose2add;
		pose2add.pose.position.x = Hst_temp_mainstates[ThreadID]->path_traversed.poses[i].x;
		pose2add.pose.position.y = Hst_temp_mainstates[ThreadID]->path_traversed.poses[i].y;
		
		hst_particle->path_traversed.poses.push_back(pose2add);
	}

	hst_particle->adaptability = Hst_temp_mainstates[ThreadID]->adaptability;
	// hst_particle->collision_idx = Hst_temp_mainstates[ThreadID]->collision_idx;

	hst_particle->num_intermediate_goals = Hst_temp_mainstates[ThreadID]->num_intermediate_goals;
	//dvc_particles[scenarioID].num= 0; //hst_particle->num; Making it zero. When we add dynamic scene, this should be set back to hst_particle->num
	

	//hst_particle->num=Hst_temp_mainstates[ThreadID]->num;
	hst_particle->weight=Hst_temp_mainstates[ThreadID]->weight;
	hst_particle->state_id=Hst_temp_mainstates[ThreadID]->state_id;
	hst_particle->scenario_id=Hst_temp_mainstates[ThreadID]->scenario_id;
	
	
	// cout << "Path index in readstatetoCPU: " << hst_particle->path_idx << endl;
}

HOST void Dvc_PomdpState::ReadPedsBackToCPU(const Dvc_PomdpState* dvc_particles,
		std::vector<State*> hst_particles, bool deep_copy)
{
	if(deep_copy)
	{
		int ThreadID=0;
		if(Globals::config.use_multi_thread_)
			ThreadID=Globals::MapThread(this_thread::get_id());

		int NumParticles=hst_particles.size();
		int Data_size=NumParticles*ModelParams::N_PED_IN;
		dim3 grid1(NumParticles,1);dim3 threads1(ModelParams::N_PED_IN,1);
		if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
		{
			CopyPeds_to_list<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]>>>
					(dvc_particles,Dvc_tempPeds[ThreadID]);
			HANDLE_ERROR(cudaMemcpyAsync((void*)Hst_tempPeds[ThreadID],
					(const void*)Dvc_tempPeds[ThreadID],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyDeviceToHost,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]));
			cudaStreamSynchronize(((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]);
		}
		else
		{
			CopyPeds_to_list<<<grid1, threads1>>>(dvc_particles,Dvc_tempPeds[0]);
			HANDLE_ERROR(cudaMemcpy((void*)Hst_tempPeds[0],
					(const void*)Dvc_tempPeds[0],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyDeviceToHost));
		}


		int Data_block_size=ModelParams::N_PED_IN;

		for(int i=0;i<NumParticles;i++)
		{
			/*
			//Commenting this out as wheelchair state does not have peds
			WheelchairState* car_state=static_cast<WheelchairState*>(hst_particles[i]);

			
			
			if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
			{
				memcpy((void*)car_state->peds,
						(const void*)(Hst_tempPeds[ThreadID]+Data_block_size*i),
						Data_block_size*sizeof(Dvc_PedStruct));
			}
			else
			{
				memcpy((void*)car_state->peds,
						(const void*)(Hst_tempPeds[0]+Data_block_size*i),
						Data_block_size*sizeof(Dvc_PedStruct));
			}*/
		}
	}
}


__global__ void CopyParticles(Dvc_PomdpState* des,Dvc_PomdpState* src,
		float* weight,int* particle_IDs,int num_particles,
		Dvc_RandomStreams* streams, int stream_pos
		)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;

	if(pos==0)
	{
		weight[0]=0;
		if(streams) streams->position_=stream_pos;
	}
	if(pos < num_particles)
	{

		int scenarioID=particle_IDs[pos];
		Dvc_PomdpState* src_i=src+scenarioID;//src is a full length array for all particles
		Dvc_PomdpState* des_i=des+pos;//des is short, only for the new partition

		//des_i->car.dist_travelled=src_i->car.dist_travelled;
		//des_i->car.pos=src_i->car.pos;
		//des_i->car.vel=src_i->car.vel;
		des_i->wheelchair.pos_x = src_i->wheelchair.pos_x;
		des_i->wheelchair.pos_y = src_i->wheelchair.pos_y;
		des_i->wheelchair.angle_z = src_i->wheelchair.angle_z;
		des_i->wheelchair.vel_v = src_i->wheelchair.vel_v;
		des_i->wheelchair.vel_w = src_i->wheelchair.vel_w;

		//Goal copying

		// des_i->goal.pos_x = src_i->goal.pos_x;
		// des_i->goal.pos_y = src_i->goal.pos_y;
		// des_i->goal.angle_z= src_i->goal.angle_z;
		des_i->path_idx = src_i->path_idx;
		des_i->joystick_x = src_i->joystick_x;
		des_i->joystick_y = src_i->joystick_y;
		des_i->path_traversed.size = src_i->path_traversed.size;
		for(int i=0;i< des_i->path_traversed.size;i++)
		{
			des_i->path_traversed.poses[i] =  src_i->path_traversed.poses[i];
		}
		des_i->contracted_path.size = src_i->contracted_path.size;
		for(int i=0;i< des_i->contracted_path.size;i++)
		{
			des_i->contracted_path.poses[i] =  src_i->contracted_path.poses[i];
		}

		des_i->adaptability = src_i->adaptability;

		des_i->collision_idx = src_i->collision_idx;

		des_i->num_intermediate_goals = src_i->num_intermediate_goals;

		des_i->num=src_i->num;
		des_i->weight=src_i->weight;
		des_i->state_id=src_i->state_id;
		des_i->scenario_id=src_i->scenario_id;

		for(int i=0;i<src_i->num;i++)
		{
			des_i->peds[i].goal=src_i->peds[i].goal;
			des_i->peds[i].id=src_i->peds[i].id;
			des_i->peds[i].pos.x=src_i->peds[i].pos.x;
			des_i->peds[i].pos.y=src_i->peds[i].pos.y;
			des_i->peds[i].vel=src_i->peds[i].vel;
		}

		if(!Dvc_config->track_alpha_vector)
		{
			//Accumulate weight of the particles
			atomicAdd(weight, des_i->weight);
		}
	}
}

void WheelchairDSPOMDP::CreateMemoryPool() const
{
	if(gpu_mainstate_pool_==NULL)
		gpu_mainstate_pool_=new GPU_MemoryPool<Dvc_PomdpState>;
	if(gpu_ped_pool_==NULL)
		gpu_ped_pool_=new GPU_MemoryPool<Dvc_PedStruct>;
}

void WheelchairDSPOMDP::DestroyMemoryPool(MEMORY_MODE mode) const
{
	switch(mode)
	{
		case DESTROY:
			if(gpu_mainstate_pool_){delete gpu_mainstate_pool_;gpu_mainstate_pool_=NULL;}
			if(gpu_ped_pool_){delete gpu_ped_pool_;gpu_ped_pool_=NULL;}
			break;
		case RESET:
			if(gpu_mainstate_pool_ ){ gpu_mainstate_pool_->ResetChuncks();};
			if(gpu_ped_pool_ ){ gpu_ped_pool_->ResetChuncks();};
			break;
	}
}
__global__ void LinkPeds(Dvc_PomdpState* state, Dvc_PedStruct* peds_memory, int numParticles)
{
	for(int i=0;i<numParticles;i++)
	{
		state[i].peds=peds_memory+i*Dvc_ModelParams::N_PED_IN;
	}
}

Dvc_State* WheelchairDSPOMDP::AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const
{
	clock_t start=clock();
	dim3 grid((numParticles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	int num_threads=1;

	if(Globals::config.use_multi_thread_)
	{
		num_threads = Globals::config.NUM_THREADS;
	}

	Dvc_PedStruct* node_particle_peds;
	switch(mode)
	{
	case INIT:

		CreateMemoryPool();

		/* Intermediate pedestrian container for copying pedestrians in host particles to device particles */
		if(Dvc_tempPeds == NULL && Hst_tempPeds == NULL){
			Dvc_tempPeds=new Dvc_PedStruct*[num_threads];
			Hst_tempPeds=new Dvc_PedStruct*[num_threads];
			for(int i=0;i<num_threads;i++)
			{
				HANDLE_ERROR(cudaMalloc((void**)&Dvc_tempPeds[i],numParticles*ModelParams::N_PED_IN*sizeof(Dvc_PedStruct) ));
				HANDLE_ERROR(cudaHostAlloc((void**)&Hst_tempPeds[i],numParticles*ModelParams::N_PED_IN*sizeof(Dvc_PedStruct),0 ));
			}
		}

		cout<<"numParticles="<<numParticles<<endl;

		if(particles_for_all_actions[0] == NULL){
			particles_for_all_actions[0]=new Dvc_State*[num_threads];
			//Allocate pedestrian memory separately
			Dvc_PedStruct*  peds_tmp=gpu_ped_pool_->Allocate((NumActions()*num_threads)*numParticles*ModelParams::N_PED_IN);

			for(int i=0;i<num_threads;i++)
			{
				HANDLE_ERROR(cudaMalloc((void**)&particles_for_all_actions[0][i],
						NumActions()*numParticles*sizeof(Dvc_PomdpState)));
				//Link pre-allocated pedestrian memory 
				LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>
						(static_cast<Dvc_PomdpState*>(particles_for_all_actions[0][i]),
						peds_tmp+(NumActions()*i)*numParticles*ModelParams::N_PED_IN,
						NumActions()*numParticles);
			}
			//Record the ped memory used by the pre-allocated lists
			//never reuse these memory for vnode particles
			gpu_ped_pool_->RecordHead();
		}

		/*Intermediate memory for copying particle IDs to device memory 
		cudaHostAlloc enables the copying to interleave with kernel executions*/
		Hst_temp_IDs=new int*[num_threads];
		for(int i=0;i<num_threads;i++)
		{
			if(Globals::config.track_alpha_vector)
			{
				cudaHostAlloc(&Hst_temp_IDs[i],(2+ Globals::config.num_scenarios + Globals::config.num_obs)*NumActions()*sizeof(int),0);
			}
			else
			{
				cudaHostAlloc(&Hst_temp_IDs[i],numParticles*sizeof(int),0);
			}
		}

		/*Intermediate memory for copying weights to device memory. 
		cudaHostAlloc enables the copying to interleave with kernel executions*/

		Hst_temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			cudaHostAlloc(&Hst_temp_weight[i],1*sizeof(float),0);

		Dvc_temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			HANDLE_ERROR(cudaMalloc(&Dvc_temp_weight[i], sizeof(float)));


		/*Intermediate memory for copying main memory of particle (everything except pedestrians) from device back to host
		cudaHostAlloc enables the copying to interleave with kernel executions*/
		Hst_temp_mainstates=new Dvc_PomdpState*[num_threads];

		for(int i=0;i<num_threads;i++)
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_temp_mainstates[i],1*sizeof(Dvc_PomdpState),0));

		/* No node particle allocated */
		return NULL;

	case ALLOC_ROOT:

		/*Intermediate managed memory for root node particles.
		 * Managed memory enables data copying between CPU and GPU without launching memcpy (which is expensive)
		 */
		HANDLE_ERROR(cudaMallocManaged((void**)&Managed_rootnode_particles, numParticles*sizeof(Dvc_PomdpState)));

		node_particle_peds = gpu_ped_pool_->Allocate(numParticles*ModelParams::N_PED_IN);

		/* Link pedestrian lists to the main memory of particles */
		LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>(Managed_rootnode_particles, node_particle_peds, numParticles);
		HANDLE_ERROR(cudaDeviceSynchronize());
		return Managed_rootnode_particles;

	case ALLOC:

		/* Allocate vnode particles: main memory and the pedestrian lists */
		Dvc_PomdpState* vnode_particles = gpu_mainstate_pool_->Allocate(numParticles);
		Dvc_PedStruct* vnode_particle_peds = gpu_ped_pool_->Allocate(numParticles*ModelParams::N_PED_IN);

		/* Link pedestrian lists to the main memory of particles */
		LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>(vnode_particles, vnode_particle_peds, numParticles);
		HANDLE_ERROR(cudaDeviceSynchronize());
		return vnode_particles;
	};


	return NULL;
}


void WheelchairDSPOMDP::CopyGPUParticlesFromParent(Dvc_State* des,Dvc_State* src,int src_offset,
		int* dvc_particle_IDs,int num_particles,bool interleave,
		Dvc_RandomStreams* streams, int stream_pos,
		void* cudaStream, int shift) const
{
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	if(num_particles<THREADDIM)
	{
		grid.x=1;grid.y=1;threads.x=num_particles;
	}

	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	if(cudaStream)
	{
		CopyParticles<<<grid, threads,0, *(cudaStream_t*)cudaStream>>>(static_cast<Dvc_PomdpState*>(des),
				static_cast<Dvc_PomdpState*>(src)+src_offset,Dvc_temp_weight[(ThreadID+shift)%Globals::config.NUM_THREADS],
				dvc_particle_IDs,num_particles, streams,stream_pos);
		if(!interleave)
			;
	}
	else
	{
		CopyParticles<<<grid, threads,0, 0>>>(static_cast<Dvc_PomdpState*>(des),
				static_cast<Dvc_PomdpState*>(src)+src_offset,Dvc_temp_weight[ThreadID],
				dvc_particle_IDs,num_particles, streams,stream_pos);
		if(!interleave)
			HANDLE_ERROR(cudaDeviceSynchronize());
	}
}


Dvc_State* WheelchairDSPOMDP::GetPointerToParticleList(int offset,  Dvc_State* full_list) const
{
	return static_cast<Dvc_PomdpState*>(full_list)+ offset;
}

void WheelchairDSPOMDP::PrintGPUState(const Dvc_State* state, std::ostream& out) const
{
	const Dvc_PomdpState* dvc_wheelchair_particles = static_cast<const Dvc_PomdpState*>(state);
	std::cout << "Dvc_Particles, path index = " << dvc_wheelchair_particles->path_idx << ", position x = " << dvc_wheelchair_particles->wheelchair.pos_x <<
		", y = " << dvc_wheelchair_particles->wheelchair.pos_y << ", theta = " << dvc_wheelchair_particles->wheelchair.angle_z << ", velocity v = " <<
		dvc_wheelchair_particles->wheelchair.vel_v << ", w = " << dvc_wheelchair_particles->wheelchair.vel_w << std::endl;
} 
Dvc_State* WheelchairDSPOMDP::CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles, bool deep_copy) const
	//dvc_particles: managed device memory storing particles
	// deep_copy: option on whether to copy list objects in side particles
{

	auto start = Time::now();


	for (int i=0;i<particles.size();i++)
	{
		const WheelchairState* src=static_cast<const WheelchairState*>(particles[i]);
		Dvc_PomdpState::CopyMainStateToGPU(static_cast<Dvc_PomdpState*>(dvc_particles),src->scenario_id,src);
	}
	//Not copying peds as there are no peds for now
	//Dvc_PomdpState::CopyPedsToGPU(static_cast<const Dvc_PomdpState*>(dvc_particles),particles.size());

	return dvc_particles;
}

void WheelchairDSPOMDP::CopyParticleIDsToGPU( int* Dvc_ptr, const std::vector<int>& particleIDs, void *cudaStream) const
{
	if(cudaStream)
	{
		int ThreadID=Globals::MapThread(this_thread::get_id());
		memcpy(Hst_temp_IDs[ThreadID],particleIDs.data(),particleIDs.size()*sizeof(int));

		HANDLE_ERROR(cudaMemcpyAsync(Dvc_ptr,Hst_temp_IDs[ThreadID],particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice,*(cudaStream_t*)cudaStream));
	}
	else
	{
		logd << "Dvc_ptr = "<< Dvc_ptr << " particleIDs.size() = " << particleIDs.size()<< " cudaStream = "<< cudaStream<< endl;
		HANDLE_ERROR(cudaMemcpy(Dvc_ptr,particleIDs.data(),particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice));
	}
}


void WheelchairDSPOMDP::DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_for_all_actions ) const
{
	int num_threads=1;

	switch (mode){
	case DESTROY:

		if(Globals::config.use_multi_thread_)
		{
			num_threads=Globals::config.NUM_THREADS;
		}
		for(int i=0;i<num_threads;i++)
		{
			if(particles_for_all_actions[i]!=NULL)
				{HANDLE_ERROR(cudaFree(particles_for_all_actions[i]));particles_for_all_actions[i]=NULL;}
		}
		if(particles_for_all_actions)delete [] particles_for_all_actions;particles_for_all_actions=NULL;
		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(Hst_temp_IDs[i]);
		}
		delete [] Hst_temp_IDs;
		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(Hst_temp_weight[i]);
		}
		delete [] Hst_temp_weight;
		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_temp_weight[i]);
		}
		delete [] Dvc_temp_weight;

		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_tempPeds[i]);
			cudaFreeHost(Hst_tempPeds[i]);
			cudaFreeHost(Hst_temp_mainstates[i]);
		}

		delete [] Dvc_tempPeds;
		delete [] Hst_tempPeds;
		delete [] Hst_temp_mainstates;
		break;
	case RESET:
		HANDLE_ERROR(cudaFree(static_cast<Dvc_PomdpState*>(Managed_rootnode_particles)));

		break;
	};

	DestroyMemoryPool(mode);
}


DEVICE float Dvc_PedPomdpParticleUpperBound1::Value(
		const Dvc_State* particles, int scenarioID, Dvc_History& history) {

	return Dvc_ModelParams::GOAL_REWARD / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
}


//TODO: Modify this according to Step function of wheelchair POMDP model
DEVICE bool Dvc_WheelchairPomdp::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	int* obs) {

	//threadIdx.x is the particle
	//threadIdx.y is number of peds
	//blockIdx.x is number of actions
	//copy contents, link cells to existing ones
// 	__shared__ int iscollision[32];

// 	if(FIX_SCENARIO==1 || GPUDoPrint)
// 		if(GPUDoPrint && wheechairpomdp_state.scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
// 			printf("(GPU) Before step: scenario=%d \n", wheelchairpomdp_state.scenario_id);
// 			printf("action= %d\n ",action);
// 			printf("Before step:\n");
// 			//int pos=pedpomdp_state.car.pos;
// 			//printf("car_pox= %d ",pos);
// 			//printf("trav_dist=%f\n",pedpomdp_state.car.dist_travelled);
// 			printf("car_vel= %f\n",pedpomdp_state.car.vel_x);

// 			for(int i=0;i<pedpomdp_state.num;i++)
// 			{
// 				printf("ped %d pox_x= %f pos_y=%f\n",i,
// 						pedpomdp_state.peds[i].pos.x,pedpomdp_state.peds[i].pos.y);
// 			}
// 		}

	
	Dvc_PomdpState& wheelchairpomdp_state = static_cast<Dvc_PomdpState&>(state);
	Dvc_WheelchairStruct& wheelchairpomdp_status = wheelchairpomdp_state.wheelchair;
	int& goal_point_idx = wheelchairpomdp_state.path_idx;

	// check whether the goal is reached
	bool reaching_goal = false;
	// check how many steps required for transition (especially for stop action)
	int transition_steps = 1;

	Eigen::Quaternionf map_quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	* Eigen::AngleAxisf(-agent2map_yaw, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf wheelchair_quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	* Eigen::AngleAxisf(wheelchairpomdp_status.angle_z, Eigen::Vector3f::UnitZ());
	
	Eigen::Vector3f agent_heading(1,0,0);
	agent_heading = wheelchair_quat.matrix()*agent_heading;

	Eigen::Vector3f joystick_heading(wheelchairpomdp_state.joystick_x, wheelchairpomdp_state.joystick_y, 0);
	//if (wheelchairpomdp_state.joystick_x == 0 && wheelchairpomdp_state.joystick_y == 0)
	//	{joystick_heading[0] = 0.0001;}

	float angle_diff;
	//Dvc_GoalStruct& wheelchair_goal = wheelchairpomdp_state.goal;
	// if (GPUDoPrint)
	// {
	// 	printf("Before step: \n\
	// 	GPU idx = %d.\n\
	// 	Block idx.x = %d, y = %d, thread idx.x = %d, y = %d.\n\
	// 	Scenario id = %d.\n\
	// 	Particle weight = %f, state_id = %d.\n\
	// 	Position: x = %f, y = %f, theta = %f.\n\
	// 	Velocity: v = %f, w = %f.\n\
	// 	Path index = %d.\n\
	// 	Goal Position: x = %f, y = %f\n\
	// 	Action = %d\n",
	// 	blockDim.x * blockIdx.x + threadIdx.x,
	// 	blockIdx.x, blockIdx.y, threadIdx.x, threadIdx.y,
	// 	wheelchairpomdp_state.scenario_id,
	// 	wheelchairpomdp_state.weight, wheelchairpomdp_state.state_id,
	// 	wheelchairpomdp_state.wheelchair.pos_x, wheelchairpomdp_state.wheelchair.pos_y, wheelchairpomdp_state.wheelchair.angle_z,
	// 	wheelchairpomdp_state.wheelchair.vel_v, wheelchairpomdp_state.wheelchair.vel_w,
	// 	wheelchairpomdp_state.path_idx,
	// 	goal_positions[wheelchairpomdp_state.path_idx].x, goal_positions[wheelchairpomdp_state.path_idx].y,
	// 	action);
	// }

	// printf("Joystick input: x = %.3f, y = %.3f\n", external_joy_x, external_joy_y);
	//long obs_integer;
	//Dvc_WheelchairObs wheelchair_obs;

	// float max_linear_speed = fabs(external_joy_x);
	float max_linear_speed = external_joy_x > 0 ? fabs(external_joy_x) : Dvc_ModelParams::backing_ratio * fabs(external_joy_x);

	float v_follow = 0, w_follow = 0;
	float v_before = roundf(wheelchairpomdp_state.wheelchair.vel_v*10)/10; //round(wheelchairpomdp_status.vel_v * 100) / 100;
	float w_before = roundf(wheelchairpomdp_state.wheelchair.vel_w*10)/10; //round(wheelchairpomdp_status.vel_w * 100) / 100;
	
	float v_after = v_before;
	float w_after = w_before;

	float follow_cost = Dvc_FollowingVel(joystick_heading, v_before, w_before, v_follow, w_follow, max_linear_speed);

	float scale_factor = 1.0; //wheelchair_state.collision_idx > 0 ? ModelParams::step_scale : 1.0; This part of CPU code not being used
	
	switch (action)
	{
	case 0:	// linear acceleration
	{
		if (v_before >= Dvc_ModelParams::max_linear_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
			v_after = max_linear_speed;
		}
		else
		{
			v_after = v_before + Dvc_ModelParams::step_size * scale_factor; // Increase the linear speed by Dvc_ModelParams::step_size
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 1:	// linear deceleration
	{
		if (v_before <= - Dvc_ModelParams::max_linear_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
			v_after = - max_linear_speed;
		}
		else
		{
			v_after = v_before - Dvc_ModelParams::step_size * scale_factor; // Decrease the linear speed by Dvc_ModelParams::step_size
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 2:	// angular acceleration
	{
		if (w_before >= Dvc_ModelParams::max_angular_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
			w_after = Dvc_ModelParams::max_angular_speed;
		}
		else
		{
			w_after = w_before + Dvc_ModelParams::step_size * scale_factor; // Increase the angular speed by Dvc_ModelParams::step_size
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 3: // angular deceleration
	{
		if (w_before <= - Dvc_ModelParams::max_angular_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
			w_after = - Dvc_ModelParams::max_angular_speed;
		}
		else
		{
			w_after = w_before - Dvc_ModelParams::step_size * scale_factor; // Decrease the angular speed by Dvc_ModelParams::step_size
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 4:	// keep
	{
		reward = Dvc_ModelParams::step_penalty; // Penalize the step
		break;
	}
	case 5:	// stop
	{
		// stop the wheelchair
		v_after = 0.0;
		w_after = 0.0;
		if (fabs(v_before) > 0.2)
		{
			reward = Dvc_ModelParams::step_penalty + Dvc_ModelParams::stationary_penalty * (fabs(v_before) - 0.2);
		}
		else
		{
			reward = Dvc_ModelParams::step_penalty;
		}
		break;
	}
	case 6:	// follow
	{
		v_after = v_follow;
		w_after = w_follow;

		reward = Dvc_ModelParams::step_penalty; // Penalize the step
		break;
	}
	default:	// move to goals
	{
		bool stop_wheechair = false;
		reward = 0;
		double reward_discount = 1;
		int point_along_path = 0;	// the next point along the contracted path
		Eigen::Vector3f agent2path(0, 0, 0);

		if (fabs(v_before) != 0)
		{
			// cout << "Stop the wheelchair because it's moving " << endl;
			stop_wheechair = true;
			v_after = 0.0;
			w_after = 0.0;
			if (fabs(v_before) > Dvc_ModelParams::step_size)
			{
				reward += Dvc_ModelParams::step_penalty + Dvc_ModelParams::stationary_penalty * (fabs(v_before) - 0.2);
			}
			else
			{
				reward += Dvc_ModelParams::step_penalty;
			}

			// cout << "stop moving v_before = " << v_before << ", v_after = " << v_after << endl;
			// cout << "stop moving w_before = " << w_before << ", w_after = " << w_after << endl;

			//GenerateNewPath(wheelchair_status, wheelchair_path.paths[action - ModelParams::num_normal_actions]);
			Dvc_GenerateNewPath(wheelchairpomdp_status, wheelchairpomdp_state.path_traversed);
			float transition_reward = Dvc_TransitWheelchair(wheelchairpomdp_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
			reward += transition_reward;

			// if (transition_reward <= ModelParams::collision_penalty)
			// {
			// 	// a collision happened
			// 	return true;
			// }

			// compute the reward
			if (wheelchairpomdp_state.adaptability)	// follow the path
			{
				reward += Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
				// Reach the final goal
				if (reaching_goal)
				{
					return true;	// Only terminate the task when a final goal is reached
				}
			}
			else	// follow the user
			{
				reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
			}
			// cout << "Finish path contraction for move to goal after stopping the wheelchair" << action - ModelParams::num_normal_actions << endl;
		}

		if (wheelchairpomdp_state.path_traversed.size > 0)
		{
			// cout << "Contracting path..." << endl;
			// only contract the path when new points are added to the current path
			Dvc_ContractAndInterpolatePath(wheelchairpomdp_status, wheelchairpomdp_state.path_traversed, wheelchairpomdp_state.contracted_path, action - Dvc_ModelParams::num_normal_actions, map_quat);
		}
		point_along_path = 0;
		int num2simulate = stop_wheechair ? Dvc_ModelParams::num_simulation_m2g - 1 : Dvc_ModelParams::num_simulation_m2g;
		
		// cout << "Simulating " << num2simulate << " steps for move to goal " << action - ModelParams::num_normal_actions << endl;
		for (int i = 0; i < num2simulate;)
		{
			reward_discount = stop_wheechair ? powf(Dvc_ModelParams::reward_discount, i + transition_steps) : powf(ModelParams::reward_discount, i);
			// wheechair is not moving, turn in place to face towards the contracted path
			//agent2path.setValue(wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.x
			//	- wheelchair_status.agent_pose.position.x,
			//	wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.y
			//	- wheelchair_status.agent_pose.position.y, 0);
			agent2path(0) = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].x
			- wheelchairpomdp_status.pos_x;
			agent2path(1) = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].y
			- wheelchairpomdp_status.pos_y;
			agent2path(2) = 0;

			while (agent2path.norm() <= 0.1)
			{
				point_along_path = point_along_path + 1;
				// agent2path.setValue(wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.x
				// 	- wheelchair_status.agent_pose.position.x,
				// 	wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.y
				// 	- wheelchair_status.agent_pose.position.y, 0);
				agent2path(0) = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].x
				- wheelchairpomdp_status.pos_x;
				agent2path(1) = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].y
				- wheelchairpomdp_status.pos_y;
				agent2path(2) = 0;
			}
			// the angle between current heading and the direction facing the path

			// tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);

			// the current wheelchair heading
			//agent_heading.setValue(1, 0, 0);
			//agent_heading = tf2::quatRotate(wheelchair_quat, agent_heading);
			//float angle2turn = agent_heading.angle(agent2path);
			agent_heading(0) = 1; agent_heading(1) = 0; agent_heading(2) = 0; 
			agent_heading = wheelchair_quat.matrix()*agent_heading;
			float angle2turn = acosf(agent_heading.dot(agent2path)/(agent_heading.norm() * agent2path.norm()));//joystick_heading.angle(velocity_heading) / M_PI;
			
			

			// cout << "angle2turn = " << angle2turn << endl;

			// the wheelchair is facing the path
			// the wheelchair is facing the path
			if (angle2turn <= Dvc_ModelParams::facing_angle)
			{
				// the wheelchair is facing the path, but still turning, stop the wheelchair
				if (fabs(w_before) != 0 && i == 0)
				{
					// cout << "Stop the wheelchair because it's turning " << endl;
					v_after = 0.0;
					w_after = 0.0;
					reward += Dvc_ModelParams::step_penalty;
					// cout << "stop turning v_before = " << v_before << ", v_after = " << v_after << endl;
					// cout << "stop turning w_before = " << w_before << ", w_after = " << w_after << endl;

					float transition_reward = Dvc_TransitWheelchair(wheelchairpomdp_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
					reward += transition_reward * reward_discount;

					// if (transition_reward <= ModelParams::collision_penalty)
					// {
					// 	// a collision happened
					// 	return true;
					// }

					// compute the reward
					if (wheelchairpomdp_state.adaptability)	// follow the path
					{
						reward += reward_discount * Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
						// Reach the final goal
						if (reaching_goal)
						{
							return true;	// Only terminate the task when a final goal is reached
						}
					}
					else	// follow the user
					{
						reward += reward_discount * Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
					}
					i++;
				}

				// the wheelchair is facing the path, and not turning, move along the path
				else
				{
					// cout << "Move the wheelchair along the path " << endl;
					wheelchairpomdp_status.pos_x = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].x ;
					//wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.x;
					wheelchairpomdp_status.pos_y = intermediate_goal_list[action - Dvc_ModelParams::num_normal_actions].way_points_[point_along_path].y ;
					//wheelchair_path.paths[action - ModelParams::num_normal_actions].poses[point_along_path].pose.position.y;

					wheelchairpomdp_status.vel_v = 0;
					wheelchairpomdp_status.vel_w = 0;
					// max linear speed increment is max_acceleration * transition_time, namely 1 * 0.3 = 0.3
					// count how many steps are required for reaching the next point along the path
					// area of one rectangle is a*t^2
					float base_area = Dvc_ModelParams::max_v_acceleration * Dvc_ModelParams::transition_time * Dvc_ModelParams::transition_time;

					// stop after the moving
					v_after = 0;
					w_after = 0;

					// 2 steps are required: linear+, linear-
					if (agent2path.norm() <= base_area)
					{
						// compute the reward
						for (int j = 0; j < 2; ++j)
						{
							float extra_discount = powf(Dvc_ModelParams::reward_discount, j);
							// step penalty
							reward += Dvc_ModelParams::step_penalty * reward_discount * extra_discount;
							// follow user reward
							if (!wheelchairpomdp_state.adaptability)
							{
								// v_after = 0.5 * ModelParams::max_v_acceleration * ModelParams::transition_time;
								// w_after = 0;
								reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * reward_discount * extra_discount;
							}
						}

						// follow the path reward
						if (wheelchairpomdp_state.adaptability)
						{
							reward_discount *= powf(Dvc_ModelParams::reward_discount, 1);	// 1 more step
							reward += reward_discount * Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal);
							// Reach the final goal
							if (reaching_goal)
							{
								return true;	// Only terminate the task when a final goal is reached
							}
						}

						i = i + 2;	// 2 steps
					}

					// 3 steps are required: linear+, keep, linear-
					else if (agent2path.norm() <= 2 * base_area)
					{
						// compute the reward
						for (int j = 0; j < 3; ++j)
						{
							float extra_discount = powf(Dvc_ModelParams::reward_discount, j);
							// step penalty
							reward += Dvc_ModelParams::step_penalty * reward_discount * extra_discount;
							// follow user reward
							if (!wheelchairpomdp_state.adaptability)
							{
								// v_after = 0.6667 * ModelParams::max_v_acceleration * ModelParams::transition_time;
								// w_after = 0;
								reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * reward_discount * extra_discount;
							}
						}

						// follow the path reward
						if (wheelchairpomdp_state.adaptability)
						{
							reward_discount *= powf(Dvc_ModelParams::reward_discount, 2);	// 2 more steps
							reward += reward_discount * Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal);
							// Reach the final goal
							if (reaching_goal)
							{
								return true;	// Only terminate the task when a final goal is reached
							}
						}
						
						i = i + 3;	// 3 steps
					}

					else
					{
						// compute the reward
						for (int j = 0; j < 4; ++j)
						{
							float extra_discount = powf(Dvc_ModelParams::reward_discount, j);
							// step penalty
							reward += Dvc_ModelParams::step_penalty * reward_discount * extra_discount;
							// follow user reward
							if (!wheelchairpomdp_state.adaptability)
							{
								// v_after = 0.75 * ModelParams::max_v_acceleration * ModelParams::transition_time;
								// w_after = 0;
								reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * reward_discount * extra_discount;
							}
						}

						// follow the path reward
						if (wheelchairpomdp_state.adaptability)
						{
							reward_discount *= powf(ModelParams::reward_discount, 3);	// 3 more steps
							reward += reward_discount * Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal);
							// Reach the final goal
							if (reaching_goal)
							{
								return true;	// Only terminate the task when a final goal is reached
							}
						}
						
						i = i + 4;	// 4 steps
					}
					point_along_path++;	// move to the next point

				}

			}

			// the wheelchair needs to turn to face the path
			else
			{
				// cout << "Turn the wheelchair to face the path" << endl;
				//float cross_product = agent_heading.getX() * agent2path.getY() - agent_heading.getY() * agent2path.getX();
				float cross_product = agent_heading(0) * agent2path(1) - agent_heading(1) * agent2path(0);
				
				angle2turn = (cross_product >= 0)? angle2turn : -angle2turn;

				int case_num = 0;

				int steps2turn = Dvc_TurningSteps(angle2turn, w_before, case_num);

				// stop after the turning
				v_after = 0;
				w_after = 0;
			
				// after turning, update the wheelchair status
				// the orientation now is aligned with angle2path
				//wheelchair_status.agent_pose_angle += angle2turn;
				wheelchairpomdp_status.angle_z += angle2turn;
				//wheelchair_quat.setRPY(0, 0, wheelchair_status.agent_pose_angle);
				//tf2::convert(wheelchair_quat, wheelchair_status.agent_pose.orientation);
				wheelchair_quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  									* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  								* Eigen::AngleAxisf(wheelchairpomdp_status.angle_z, Eigen::Vector3f::UnitZ());




				wheelchairpomdp_status.vel_v = 0;
				wheelchairpomdp_status.vel_w = 0;
				
				// tf2::Quaternion joystick_quat;
				// joystick_quat.setRPY(0, 0, - angle2turn);
				// joystick_heading.setValue(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
				// joystick_heading = tf2::quatRotate(joystick_quat, joystick_heading);
				// wheelchair_state.joystick_x = round(joystick_heading.getX() * 100) / 100;
				// wheelchair_state.joystick_y = round(joystick_heading.getY() * 100) / 100;

				// compute the reward
				for (int j = 0; j < steps2turn; ++j)
				{
					float extra_discount = powf(Dvc_ModelParams::reward_discount, j);
					// step penalty
					reward += Dvc_ModelParams::step_penalty * reward_discount * extra_discount;
					// follow user reward
					if (!wheelchairpomdp_state.adaptability)
					{
						v_after = 0;
						w_after = 0;
						reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * reward_discount * extra_discount;
					}
				}
				// no moving, so the reaching check is not needed

				i = i + steps2turn;
			}

		}
		return true;
	}
	}

	// Penalize the linear velocities that exceed the joystick input
	if (fabs(v_after) > max_linear_speed)
	{
		reward += fabs(v_after) * Dvc_ModelParams::excess_speed_penalty / Dvc_ModelParams::step_size;
	}
	// add white noise to the current velocity
	// according to the log file, the velocity diff can be as high as 0.04, so the tolerance will be ±0.04
	if (Dvc_ModelParams::using_probabilistic_model)
	{
		wheelchairpomdp_state.wheelchair.vel_v += (rand_num - 0.5) * 2 * Dvc_ModelParams::noise_amplitude;
		wheelchairpomdp_state.wheelchair.vel_w += (rand_num - 0.5) * 2 * Dvc_ModelParams::noise_amplitude;
	}
	
	// Before trainsition, add the current position to the path if the wheelchair moves (velocity is non-zero)
	if (v_before != 0 || v_after != 0)
	{
		Dvc_GenerateNewPath(wheelchairpomdp_status, wheelchairpomdp_state.path_traversed);
		//for (int i = 0; i < ModelParams::num_paths; ++i)
		//{
			//GenerateNewPath(wheelchair_status, wheelchair_path.paths[i]);
			// ContractAndInterpolatePath(wheelchair_status, wheelchair_path.paths[i], map_quat);
		//}
	}

	// Transit to the next state with a penalty_idx [0, 1] as a return value

	// cout << "action = " << action << endl;
	// cout << "normal v_before = " << v_before << ", v_after = " << v_after << endl;
	// cout << "normal w_before = " << w_before << ", w_after = " << w_after << endl;
	float transition_reward = Dvc_TransitWheelchair(wheelchairpomdp_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
	reward += transition_reward;


	if (wheelchairpomdp_state.adaptability)	// follow the path
	{
		reward += Dvc_ReachingCheck(wheelchairpomdp_state, reaching_goal) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
		// Reach the final goal
		if (reaching_goal)
		{
			return true;	// Only terminate the task when a final goal is reached
		}
	}
	else	// follow the user
	{
		reward += Dvc_FollowUserReward(v_follow, w_follow, v_after, w_after) * powf(Dvc_ModelParams::reward_discount, transition_steps - 1);
	}
	/* THE SHARED CONTROL PART */

	// if (external_joy_x != 0 || external_joy_y != 0)
	// {
	// 	if (wheelchairpomdp_status.vel_v == 0 && wheelchairpomdp_status.vel_w == 0)
	// 	{
	// 		reward += Dvc_ModelParams::stationary_penalty;
	// 	}
	// 	else
	// 	{
	// 		Eigen::Vector3f joystick_direction(external_joy_x, external_joy_y, 0);
	// 		Eigen::Vector3f moving_direction(wheelchairpomdp_status.pos_x, wheelchairpomdp_status.pos_y, 0);
	// 		float angle_user_robot = acosf(joystick_direction.dot(moving_direction)/(joystick_direction.norm() * moving_direction.norm()));
	// 		reward += (0.5 * M_PI -  angle_user_robot) * Dvc_ModelParams::user_following_reward * 2 / M_PI;
	// 	}
	// }

	//Not needed as this is done in CalAngleDiff
	//wheelchair_quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	//								* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	//							* Eigen::AngleAxisf(wheelchairpomdp_status.angle_z, Eigen::Vector3f::UnitZ());

	// the current wheelchair heading
	//agent_heading(0) = 1; agent_heading(1) = 0; agent_heading(2) = 0; //.setValue(1, 0, 0);
	//agent_heading = wheelchair_quat.matrix() * agent_heading; //::quatRotate(wheelchair_quat, agent_heading);
	
	
	
	// Compute the reward after transition

	


	// if (penalty_idx == 1) // A severe collision happens
	// {
	// 	reward += Dvc_ModelParams::collision_penalty;
	// 	return true;	// Only terminate the task when a severe collision occurs
	// }
	// else if (penalty_idx != 0)	// Transition succeeds with mild collision
	// {
	// 	// -100 basic penalty for mild collision plus an extra penalty based on the distance to obstacles
	// 	reward += (Dvc_ModelParams::collision_penalty - Dvc_ModelParams::inflation_basic_penalty) * penalty_idx + Dvc_ModelParams::inflation_basic_penalty;
	// }
	// // printf("Reward after TransitWheelchair: %d.\n", reward);
	// // else penalty_idx == 0, transition succeeds with no collision, no basic penalty added
	// angle_diff = Dvc_CalAngleDiff(wheelchairpomdp_status, goal_point_idx);
	// // Do the reaching check after transition
	// int num_reached_goals = Dvc_ReachingCheck(wheelchairpomdp_state);
	// // Reach the final goal
	// if (num_reached_goals == intermediate_goal_list[wheelchairpomdp_state.path_idx].size_)
	// {
	// 	reward += Dvc_ModelParams::reaching_reward;
	// 	return true;	// Only terminate the task when a final goal is reached
	// }
	// // Reach the intermediate goals or not reach any goal
	// else
	// {
	// 	reward += Dvc_ModelParams::inter_goal_reward * num_reached_goals;	// Reward a +10 for each intermediate goal reached
	// }
	// printf("Reward after ReachingCheck: %d.\n", reward);
	// if (GPUDoPrint)
	// {
	// 	printf("After step: \n\
	// 	GPU idx = %d.\n\
	// 	Block idx.x = %d, y = %d, thread idx.x = %d, y = %d.\n\
	// 	Scenario id = %d.\n\
	// 	Particle weight = %f, state_id = %d.\n\
	// 	Position: x = %f, y = %f, theta = %f.\n\
	// 	Velocity: v = %f, w = %f.\n\
	// 	Path index = %d.\n\
	// 	Goal Position: x = %f, y = %f\n\
	// 	Num of reached goals = %d\n\
	// 	Action = %d\n\
	// 	Reward = %f\n",
	// 	blockDim.x * blockIdx.x + threadIdx.x,
	// 	blockIdx.x, blockIdx.y, threadIdx.x, threadIdx.y,
	// 	wheelchairpomdp_state.scenario_id,
	// 	wheelchairpomdp_state.weight, wheelchairpomdp_state.state_id,
	// 	wheelchairpomdp_state.wheelchair.pos_x, wheelchairpomdp_state.wheelchair.pos_y, wheelchairpomdp_state.wheelchair.angle_z,
	// 	wheelchairpomdp_state.wheelchair.vel_v, wheelchairpomdp_state.wheelchair.vel_w,
	// 	wheelchairpomdp_state.path_idx,
	// 	goal_positions[wheelchairpomdp_state.path_idx].x, goal_positions[wheelchairpomdp_state.path_idx].y,
	// 	num_reached_goals,
	// 	action,
	// 	reward);
	// }

	// printf("After step: Action = %d, Reward = %d\n", action, reward);

// Observation part
//wheelchair_obs.getObsFromWheelchairStatus(wheelchairpomdp_status);
// Obtain the angle between wheelchair heading and the vector pointing from wheelchair position to goal door
////float agent2goal_x = wheelchairpomdp_state.goal.pos_x - wheelchairpomdp_state.wheelchair.pos_x;
//float agent2goal_y = wheelchairpomdp_state.goal.pos_y - wheelchairpomdp_state.wheelchair.pos_y;
//float agent2goal_angle = atan2f(agent2goal_y, agent2goal_x);

//float angle_diff = agent2goal_angle - (wheelchairpomdp_state.wheelchair.angle_z * M_PI / 180);
	
	// Observation part
	int joystick_obs;

	if (wheelchairpomdp_state.adaptability)
	{

		angle_diff = Dvc_CalAngleDiff(wheelchairpomdp_status, goal_point_idx);

		angle_diff = angle_diff >= 0 ? angle_diff : angle_diff + 2 * M_PI;
		int direction_index = roundf(angle_diff * 8 / M_PI);
		direction_index = direction_index == 16 ? 0 : direction_index;
		// intermediate goal is in the right direction
		if (direction_index == 0)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_FRONT;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_FR_L;
			}
			else
			{
				joystick_obs = OBS_FR_R;
			}
		}
		// intermediate goal is in the front right direction
		else if (direction_index == 1)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_FR_L;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_FRONT;
			}
			else
			{
				joystick_obs = OBS_F_L;
			}
		}
		// intermediate goal is in the front direction
		else if (direction_index == 2)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_F_L;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_FR_L;
			}
			else
			{
				joystick_obs = OBS_F_LE;
			}	
		}	
		// intermediate goal is in the front left direction
		else if (direction_index == 3)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_F_LE;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_F_L;
			}
			else
			{
				joystick_obs = OBS_LEFT;
			}
			
		}
		else if(direction_index == 4)	// intermediate goal is in the left direction (7pi/16 ~ 9pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_LEFT;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_F_LE;
			}
			else
			{
				joystick_obs = OBS_B_LE;
			}
			
		}
		else if(direction_index == 5)	// intermediate goal is in the back left direction (9pi/16 ~ 11pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_B_LE;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_LEFT;
			}
			else
			{
				joystick_obs = OBS_B_L;
			}
			
		}
		else if(direction_index == 6)	// intermediate goal is in the back left direction (11pi/16 ~ 13pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_B_L;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_B_LE;
			}
			else
			{
				joystick_obs = OBS_BA_L;
			}
			
		}
		else if(direction_index == 7)	// intermediate goal is in the back left direction (13pi/16 ~ 15pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_BA_L;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_B_L;
			}
			else
			{
				joystick_obs = OBS_BACK;
			}
			
		}
		else if(direction_index == 8)	// intermediate goal is in the back direction (15pi/16 ~ pi, -pi ~ -15pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_BACK;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_BA_L;
			}
			else
			{
				joystick_obs = OBS_BA_R;
			}
			
		}
		else if(direction_index ==9)	// intermediate goal is in the back right direction (-15pi/16 ~ -13pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_BA_R;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_BACK;
			}
			else
			{
				joystick_obs = OBS_B_R;
			}
			
		}
		else if(direction_index ==10)	// intermediate goal is in the back right direction (-13pi/16 ~ -11pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_B_R;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_BA_R;
			}
			else
			{
				joystick_obs = OBS_B_RI;
			}
			
		}
		else if(direction_index == 11)	// intermediate goal is in the back right direction (-11pi/16 ~ -9pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_B_RI;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_B_R;
			}
			else
			{
				joystick_obs = OBS_RIGHT;
			}
			
		}
		else if(direction_index == 12)	// intermediate goal is in the right direction (-9pi/16 ~ -7pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_RIGHT;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_B_RI;
			}
			else
			{
				joystick_obs = OBS_F_RI;
			}
			
		}
		else if(direction_index == 13)	// intermediate goal is in the front right direction (-7pi/16 ~ -5pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_F_RI;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_RIGHT;
			}
			else
			{
				joystick_obs = OBS_F_R;
			}
			
		}
		else if(direction_index == 14)	// intermediate goal is in the front right direction (-5pi/16 ~ -3pi/16)
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_F_R;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_F_RI;
			}
			else
			{
				joystick_obs = OBS_FR_R;
			}
			
		}		
		else if(direction_index == 15)		// intermediate goal is in the front right direction (-3pi/16 ~ -pi/16)// angle_diff >= 0.375 * M_PI
		{
			if (rand_num < 0.7)
			{
				joystick_obs = OBS_FR_R;
			}
			else if (rand_num < 0.85)
			{
				joystick_obs = OBS_F_R;
			}
			else
			{
				joystick_obs = OBS_FRONT;
			}
		}
	}
	else
	{
		if (wheelchairpomdp_state.joystick_x == 0 && wheelchairpomdp_state.joystick_y == 0)
			{joystick_heading(0) = 0.0001;}
		Eigen::Vector3f velocity_heading(1,0,0);
		angle_diff = acosf(joystick_heading.dot(velocity_heading)/(joystick_heading.norm() * velocity_heading.norm()))/M_PI;//joystick_heading.angle(velocity_heading) / M_PI;
//joystick_heading.angle(tf2::Vector3(1, 0, 0));
		angle_diff = (joystick_heading(1) >= 0)? angle_diff : 2 * M_PI - angle_diff;
		
		int direction_index = roundf(angle_diff * 8 / M_PI);
		direction_index = direction_index == 16 ? 0 : direction_index;
		joystick_obs = direction_index;
	}

// Updates hashmap:
// takes hash value as key and stores wheelchair_obs as value in map
// First check if hash value is already stored
//Dvc_PrintObs(wheelchair_obs, obs_integer);

/*if (std::find(ObsMap.begin(), ObsMap.end(), obs_integer) == ObsMap.end())
{
	ObsMap.push_back(obs_integer);
}*/

//Update int* obs
	if(obs!=NULL) //This check is necessary because default policy sends in null obs to step function.
	{
	int obs_array_counter = 0;
	obs[obs_array_counter++] = 6; //Number of values in obs array
	obs[obs_array_counter++] = lroundf(wheelchairpomdp_state.wheelchair.pos_x / Dvc_ModelParams::pos_rln);
	obs[obs_array_counter++] = lroundf(wheelchairpomdp_state.wheelchair.pos_y / Dvc_ModelParams::pos_rln);
	obs[obs_array_counter++] = lroundf(wheelchairpomdp_state.wheelchair.angle_z / Dvc_ModelParams::pos_rln);
	obs[obs_array_counter++] = lroundf(wheelchairpomdp_state.wheelchair.vel_v / Dvc_ModelParams::pos_rln);
	obs[obs_array_counter++] = lroundf(wheelchairpomdp_state.wheelchair.vel_w / Dvc_ModelParams::pos_rln);
	obs[obs_array_counter++] = joystick_obs;
	}
	

	return false;
	//	return true;
	//else
	//	return false;
// 	reward = 0;

// 	unsigned long long int Temp=INIT_QUICKRANDSEED;

// 	/* Termination checking */
// 	if(threadIdx.y==0)
// 	{
// 		// Terminate upon reaching goal
// 		if (pedpomdp_state.car.dist_travelled > Dvc_ModelParams::GOAL_TRAVELLED-1e-4
// 				|| pedpomdp_state.car.pos >= path->size_-1) {
// 			reward = Dvc_ModelParams::GOAL_REWARD;
// 			terminal= true;
// 		}
// 	}

// 	/* Collision checking */
// 	iscollision[threadIdx.x]=false;
// 	__syncthreads();

// 	if(!terminal)
// 	{
// 		const int car = pedpomdp_state.car.pos;
// 		const Dvc_COORD& car_pos = path->way_points_[car];
// 		const Dvc_COORD& forward_pos = path->way_points_[path->forward(car, 1.0)];

// 		if(threadIdx.y<pedpomdp_state.num){
// 			const Dvc_COORD& pedpos = pedpomdp_state.peds[threadIdx.y].pos;
// 			bool collide_ped=false;
// 			float HNx = forward_pos.x - car_pos.x, // car direction
// 						 HNy = forward_pos.y - car_pos.y;
// 			float HMx = pedpos.x - car_pos.x,
// 						 HMy = pedpos.y - car_pos.y;


// /// car geomery for golfcart
// /*			double car_width = 0.87,
// 			car_length = 1.544;

// 			double safe_margin = 0.92, side_safe_margin = 0.4, back_safe_margin = 0.33,
// 				 side_margin = car_width / 2.0 + side_safe_margin,
// 				 front_margin = car_length/2.0 + safe_margin,
// 				 back_margin = car_length/2.0 + back_safe_margin;
// */
// /// end golfcart

// /// car geomery for audi r8
// 			/*double car_width = 2.0,
// 			 car_length = 4.4;

// 			 double safe_margin = 0.8, side_safe_margin = 0.35, back_safe_margin = 0.2,
// 			 side_margin = car_width / 2.0 + side_safe_margin,
// 			 front_margin = 3.6 + safe_margin,
// 			 back_margin = 0.8 + back_safe_margin;*/
// /// end audi r8


// /// car geometry for pomdp car
// 			double car_width = 1.2,
// 						 car_length = 2.2;
// 			double safe_margin = 0.3,
// 				 side_margin = car_width / 2.0 + safe_margin,
// 				 front_margin = safe_margin,
// 				 back_margin = car_length + safe_margin;
// /// end pomdp car


// 			float HLx = - HNy, // direction after 90 degree anticlockwise rotation
// 						 HLy = HNx;

// 			float HM_HN = HMx * HNx + HMy * HNy, // HM . HN
// 						 HN_HN = HNx * HNx + HNy * HNy; // HN . HN
// 			if (HM_HN >= 0 && HM_HN * HM_HN > HN_HN * front_margin * front_margin)
// 				collide_ped = false;
// 			else if (HM_HN <= 0 && HM_HN * HM_HN > HN_HN * back_margin * back_margin)
// 				collide_ped = false;
// 			else
// 			{
// 			    float HM_HL = HMx * HLx + HMy * HLy, // HM . HL
// 						 HL_HL = HLx * HLx + HLy * HLy; // HL . HL
// 			    collide_ped= HM_HL * HM_HL <= HL_HL * side_margin * side_margin;
// 			}
// 			atomicOr(iscollision+threadIdx.x, collide_ped);
// 		}
// 	}
// 	__syncthreads(); // Synchronize the block to wait for collision checking with all peds (parallelized in the Y dimemsion) to finish.

// 	if(threadIdx.y==0 && !terminal)
// 	{

// 		/* Terminate if collision is detected */
// 		if(pedpomdp_state.car.vel > 0.001 && iscollision[threadIdx.x] ) { /// collision occurs only when car is moving
// 		    reward= Dvc_ModelParams::CRASH_PENALTY *
// 		    		(pedpomdp_state.car.vel * pedpomdp_state.car.vel +
// 		    				Dvc_ModelParams::REWARD_BASE_CRASH_VEL);

// 		    if(action == ACT_DEC) reward += 0.1;

// 			terminal= true;
// 		}

// 		/* Compute reward */
// 		if(!terminal)
// 		{
// 			// Smoothness penalty
// 			reward += (action == ACT_DEC || action == ACT_ACC) ? -0.1 : 0.0;

// 			reward += Dvc_ModelParams::REWARD_FACTOR_VEL *
// 					(pedpomdp_state.car.vel - Dvc_ModelParams::VEL_MAX) / Dvc_ModelParams::VEL_MAX;

// 			float acc = (action == ACT_ACC) ? Dvc_ModelParams::AccSpeed :
// 				((action == ACT_CUR) ?  0 : (-Dvc_ModelParams::AccSpeed));

// 			/* State transition: car */
// 			float dist = pedpomdp_state.car.vel / freq;
// 			int nxt = path->forward(pedpomdp_state.car.pos, dist);
// 			pedpomdp_state.car.pos = nxt;
// 			pedpomdp_state.car.dist_travelled += dist;

// 			const float N = Dvc_ModelParams::NOISE_ROBVEL;
// 			if (N>0) {
// 				if(FIX_SCENARIO!=1 && !GPUDoPrint)
// 					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);

// 				float prob = rand_num;
// 				if (prob > N) {
// 					pedpomdp_state.car.vel += acc / freq;
// 				}
// 			} else {
// 				pedpomdp_state.car.vel += acc / freq;
// 			}
// 			pedpomdp_state.car.vel = max(min(pedpomdp_state.car.vel, Dvc_ModelParams::VEL_MAX), 0.0);
// 		}
// 	}
// 	__syncthreads();


// 	if(!terminal)
// 	{
// 		/* State transition: peds */
// 		if(threadIdx.y<pedpomdp_state.num)
// 		{
// 			int i=0;
// 			while(i<threadIdx.y)
// 			{
// 				if(FIX_SCENARIO!=1 && !GPUDoPrint)
// 					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
// 				i++;
// 			}
// 			if(threadIdx.y!=0 && FIX_SCENARIO!=1 && !GPUDoPrint)
// 				rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);

// 			const Dvc_COORD& goal = goals[pedpomdp_state.peds[threadIdx.y].goal];
// 			if (abs(goal.x+1)<1e-5 && abs(goal.y+1)<1e-5) {  //stop intention, ped doesn't move
// 				;
// 			}
// 			else
// 			{
// 				// Straightline model with Gussian noise on directions
// 				Dvc_Vector goal_vec(goal.x - pedpomdp_state.peds[threadIdx.y].pos.x, goal.y - pedpomdp_state.peds[threadIdx.y].pos.y);
// 				float a = goal_vec.GetAngle();
// 				float noise = sqrt(-2 * log(rand_num));

// 				if(FIX_SCENARIO!=1 && !GPUDoPrint)
// 					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
// 				noise *= cos(2 * M_PI * rand_num)* Dvc_ModelParams::NOISE_GOAL_ANGLE;
// 				a += noise;

// 				Dvc_Vector move(a, pedpomdp_state.peds[threadIdx.y].vel/freq, 0);
// 				pedpomdp_state.peds[threadIdx.y].pos.x += move.dw;
// 				pedpomdp_state.peds[threadIdx.y].pos.y += move.dh;
// 			}
// 		}
// 	}
// 	__syncthreads();


// 	if(threadIdx.y==0 && obs!=NULL)//for each particle in the thread block
// 	{
// 		/* generate observations by descretizing the observable part of the state */
// 		if(!terminal)
// 		{
// 			int i=0;
// 			obs[i++]=2+2*pedpomdp_state.num;
// 			obs[i++] = int(pedpomdp_state.car.pos);
// 			obs[i++] = int((pedpomdp_state.car.vel+1e-5) / Dvc_ModelParams::vel_rln);
// 			for(int j = 0; j < pedpomdp_state.num; j ++) {
// 				obs[i++] = int(pedpomdp_state.peds[j].pos.x / Dvc_ModelParams::pos_rln);
// 				obs[i++] = int(pedpomdp_state.peds[j].pos.y / Dvc_ModelParams::pos_rln);
// 			}
// 		}
// 		else
// 		{
// 			int i=0;
// 			obs[i++]=0;
// 			obs[i++] = 0;
// 			obs[i++] = 0;
// 			for(int j = 0; j < pedpomdp_state.num; j ++) {
// 				obs[i++] = 0;
// 				obs[i++] = 0;
// 			}
// 		}
// 	}

// 	if(!terminal && GPUDoPrint && pedpomdp_state.scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
// 		printf("(GPU) After step: scenario=%d \n", pedpomdp_state.scenario_id);
// 		printf("rand=%f, action=%d \n", rand_num, action);
// 		printf("After step:\n");
// 		printf("Reward=%f\n",reward);
// 		int pos=pedpomdp_state.car.pos;
// 		printf("car pox= %d ",pos);
// 		printf("dist=%f\n",pedpomdp_state.car.dist_travelled);
// 		printf("car vel= %f\n",pedpomdp_state.car.vel);
// 		for(int i=0;i<pedpomdp_state.num;i++)
// 		{
// 			printf("ped %d pox_x= %f pos_y=%f\n",i,
// 					pedpomdp_state.peds[i].pos.x,pedpomdp_state.peds[i].pos.y);
// 		}
// 	}
//	return terminal;
}

DEVICE int Dvc_WheelchairPomdp::NumActions() {
	return 5;
}


//TODO: Modify this function based on wheelchair pomdp model
DEVICE float Dvc_WheelchairPomdp::Dvc_ObsProbInt(int* obs, Dvc_State& state, int action)
{
	Dvc_PomdpState& wheelchairpomdp_state = static_cast<Dvc_PomdpState&>(state);
	Dvc_WheelchairStruct& wheelchairpomdp_status = wheelchairpomdp_state.wheelchair;
	//Dvc_GoalStruct& wheelchair_goal = wheelchairpomdp_state.goal;
	int goal_point_idx = wheelchairpomdp_state.path_idx;
	
	
	Dvc_WheelchairObs wheelchair_obs(obs);
	//std::string obs_string = std::to_string(*obs);
	// joystick input is the first integer
	//wheelchair_obs.joystick_obs = std::stoi(obs_string[0]);


	// Obtain the angle between wheelchair heading and the vector pointing from wheelchair position to goal door
	//double agent2goal_x = wheelchair_goal.pos_x - wheelchairpomdp_status.pos_x;
	//double agent2goal_y = wheelchair_goal.pos_y - wheelchairpomdp_status.pos_y;
	//double agent2goal_angle = atan2(agent2goal_y, agent2goal_x);

	//double angle_diff = agent2goal_angle - wheelchairpomdp_status.angle_z * M_PI / 180;
	float angle_diff = Dvc_CalAngleDiff(wheelchairpomdp_status, goal_point_idx);

	// goal door is in the right direction
	// if (angle_diff <= -0.375 * M_PI)
	// {
	// 	if (wheelchair_obs.joystick_obs == OBS_RIGHT)
	// 	{
	// 		return 15;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FR_R)
	// 	{
	// 		return 5;
	// 	}
	// 	else
	// 	{
	// 		return 1;
	// 	}
	// }
	// // goal door is in the front right direction
	// else if (angle_diff > -0.375 * M_PI && angle_diff <= -0.125 * M_PI)
	// {
	// 	if (wheelchair_obs.joystick_obs == OBS_FR_R)
	// 	{
	// 		return 10;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FRONT)
	// 	{
	// 		return 5;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_RIGHT)
	// 	{
	// 		return 5;
	// 	}
	// 	else
	// 	{
	// 		return 1;
	// 	}
	// }
	// // goal door is in the front direction
	// else if (angle_diff > -0.125 * M_PI && angle_diff < 0.125 * M_PI)
	// {
	// 	if (wheelchair_obs.joystick_obs == OBS_FRONT)
	// 	{
	// 		return 10;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FR_L)
	// 	{
	// 		return 5;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FR_R)
	// 	{
	// 		return 5;
	// 	}
	// 	else
	// 	{
	// 		return 1;
	// 	}
	// }
	// // goal door is in the front left direction
	// else if (angle_diff >= 0.125 * M_PI && angle_diff < 0.375 * M_PI)
	// {
	// 	if (wheelchair_obs.joystick_obs == OBS_FR_L)
	// 	{
	// 		return 10;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_LEFT)
	// 	{
	// 		return 5;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FRONT)
	// 	{
	// 		return 5;
	// 	}
	// 	else
	// 	{
	// 		return 1;
	// 	}
	// }
	// // goal door is in the left direction
	// else		// angle_diff >= 0.375 * M_PI
	// {
	// 	if (wheelchair_obs.joystick_obs == OBS_LEFT)
	// 	{
	// 		return 15;
	// 	}
	// 	else if (wheelchair_obs.joystick_obs == OBS_FR_L)
	// 	{
	// 		return 5;
	// 	}
	// 	else
	// 	{
	// 		return 1;
	// 	}
	// }

	angle_diff = angle_diff >= 0 ? angle_diff : angle_diff + 2 * M_PI;

	int direction_index = roundf(angle_diff * 8 / M_PI);

	direction_index = direction_index == 16 ? 0 : direction_index;

	if(direction_index == 0)	// intermediate goal is in the front direction (-pi/16 ~ pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_FRONT)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FR_L)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FR_R)
		{
			return 5;
		}
		else
		{
			return 1;
		}
		
	}

	else if(direction_index == 1)	// intermediate goal is in the front left direction (pi/16 ~ 3pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_FR_L)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FRONT)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_F_L)
		{
			return 5;
		}
		else
		{
			return 1;
		}
	}

	else if(direction_index == 2) // intermediate goal is in the front left direction (3pi/16 ~ 5pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_F_L)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FR_L)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_F_LE)
		{
			return 5;
		}
		else
		{
			return 1;
		}
		
	}
	else if(direction_index == 3)	// intermediate goal is in the front left direction (5pi/16 ~ 7pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_F_LE)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_F_L)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_LEFT)
		{
			return 5;
		}
		else
		{
			return 1;
		}
		
	}
	else if(direction_index == 4)	// intermediate goal is in the left direction (7pi/16 ~ 9pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_LEFT)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_F_LE)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_B_LE)
		{
			return 5;
		}
		else
		{
			return 1;
		}
	}
	else if(direction_index == 5)	// intermediate goal is in the back left direction (9pi/16 ~ 11pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_B_LE)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_LEFT)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_B_L)
		{
			return 5;
		}
		else
		{
			return 1;
		}
		
	}
	else if(direction_index == 6)	// intermediate goal is in the back left direction (11pi/16 ~ 13pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_B_L)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_B_LE)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_BA_L)
		{
			return 5;
		}
		else
		{
			return 1;
		}
	}
	else if(direction_index == 7)	// intermediate goal is in the back left direction (13pi/16 ~ 15pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_BA_L)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_B_L)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_BACK)
		{
			return 5;
		}
		else
		{
			return 1;
		}
	}
	else if(direction_index == 8)	// intermediate goal is in the back direction (15pi/16 ~ pi, -pi ~ -15pi/16)
	{
		if (wheelchair_obs.joystick_obs == OBS_BACK)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_BA_L)
		{
			return 5;
		}
		else if (wheelchair_obs.joystick_obs == OBS_BA_R)
		{
			return 5;
		}
		else
		{
			return 1;
		}
	}
	else if(direction_index == 9)	// intermediate goal is in the back right direction (-15pi/16 ~ -13pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_BA_R)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_BACK)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_B_R)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}
		else if(direction_index == 10)	// intermediate goal is in the back right direction (-13pi/16 ~ -11pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_B_R)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_BA_R)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_B_RI)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}
		else if(direction_index==11)	// intermediate goal is in the back right direction (-11pi/16 ~ -9pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_B_RI)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_B_R)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_RIGHT)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}
		else if(direction_index == 12)	// intermediate goal is in the right direction (-9pi/16 ~ -7pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_RIGHT)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_B_RI)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_F_RI)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}
		else if(direction_index == 13)	// intermediate goal is in the front right direction (-7pi/16 ~ -5pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_F_RI)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_RIGHT)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_F_R)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}
		else if(direction_index == 14)	// intermediate goal is in the front right direction (-5pi/16 ~ -3pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_F_R)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_F_RI)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_FR_R)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}

		else if(direction_index == 15)	// intermediate goal is in the front right direction (-3pi/16 ~ -pi/16)
		{
			if (wheelchair_obs.joystick_obs == OBS_FR_R)
			{
				return 15;
			}
			else if (wheelchair_obs.joystick_obs == OBS_F_R)
			{
				return 5;
			}
			else if (wheelchair_obs.joystick_obs == OBS_FRONT)
			{
				return 5;
			}
			else
			{
				return 1;
			}
			
		}

	//const PomdpState& state = static_cast<const PomdpState&>(s);
	// Dvc_PomdpState& pedpomdp_state = static_cast<Dvc_PomdpState&>(state);//copy contents, link cells to existing ones
	// 	//PrintState(state);
	// 	float prob = 1.0;
	// 	float b = 0.0;
	// 	for (int j = 0; j < pedpomdp_state.num; j ++) {
	// 	  b = b + ((obs[2*j + 3]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.x )*((obs[2*j + 3]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.x );
	// 	  b = b + ((obs[2*j + 4]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.y )*((obs[2*j + 4]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.y );
	// 	  //std::cout << j << " obs vec " << obs[2*j + 2]<< "," << obs[2*j + 3] << ")b= " << b<< std::endl;
	// 	}
	// 	float stddev = 1.0;
	// 	b = - b / (2.0* stddev*stddev);
	// 	//std::cout << "b= " << b << std::endl;
	// 	return expf(b);
}
DEVICE void Dvc_WheelchairPomdp::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_PomdpState* src_i= static_cast<const Dvc_PomdpState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_PomdpState* des_i= static_cast<Dvc_PomdpState*>(des)+pos;
	des_i->weight=src_i->weight;
	des_i->scenario_id=src_i->scenario_id;
	des_i->num=src_i->num;
	
	
	//des_i->car.dist_travelled=src_i->car.dist_travelled;
	//des_i->car.pos=src_i->car.pos;
	//des_i->car.vel=src_i->car.vel;
	des_i->wheelchair.pos_x = src_i->wheelchair.pos_x;
	des_i->wheelchair.pos_y = src_i->wheelchair.pos_y;
	des_i->wheelchair.angle_z = src_i->wheelchair.angle_z;
	des_i->wheelchair.vel_v = src_i->wheelchair.vel_v;
	des_i->wheelchair.vel_w = src_i->wheelchair.vel_w;

	//Goal copying
	// des_i->goal.pos_x = src_i->goal.pos_x;
	// des_i->goal.pos_y = src_i->goal.pos_y;
	// des_i->goal.angle_z = src_i->goal.angle_z;
	des_i->path_idx = src_i->path_idx;
	des_i->joystick_x = src_i->joystick_x;
	des_i->joystick_y = src_i->joystick_y;
	
	des_i->path_traversed.size = src_i->path_traversed.size;
	for(int i=0;i< des_i->path_traversed.size;i++)
	{
		des_i->path_traversed.poses[i] =  src_i->path_traversed.poses[i];
	}
	des_i->contracted_path.size = src_i->contracted_path.size;
	for(int i=0;i< des_i->contracted_path.size;i++)
	{
		des_i->contracted_path.poses[i] =  src_i->contracted_path.poses[i];
	}

	des_i->adaptability = src_i->adaptability;

	des_i->collision_idx = src_i->collision_idx;

	des_i->num_intermediate_goals = src_i->num_intermediate_goals;

	for(int i=0;i< des_i->num;i++)
	{
		des_i->peds[i].vel=src_i->peds[i].vel;
		des_i->peds[i].pos.x=src_i->peds[i].pos.x;
		des_i->peds[i].pos.y=src_i->peds[i].pos.y;
		des_i->peds[i].goal=src_i->peds[i].goal;
		des_i->peds[i].id=src_i->peds[i].id;
	}
}

DEVICE void Dvc_WheelchairPomdp::Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_PomdpState* src_i= static_cast<const Dvc_PomdpState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_PomdpState* des_i= static_cast<Dvc_PomdpState*>(des)+pos;
	des_i->weight=src_i->weight;
	des_i->scenario_id=src_i->scenario_id;
	des_i->num=src_i->num;

	//des_i->car.dist_travelled=src_i->car.dist_travelled;
	//des_i->car.pos=src_i->car.pos;
	//des_i->car.vel=src_i->car.vel;
	des_i->wheelchair.pos_x = src_i->wheelchair.pos_x;
	des_i->wheelchair.pos_y = src_i->wheelchair.pos_y;
	des_i->wheelchair.angle_z = src_i->wheelchair.angle_z;
	des_i->wheelchair.vel_v = src_i->wheelchair.vel_v;
	des_i->wheelchair.vel_w = src_i->wheelchair.vel_w;

	//Goal copying
	// des_i->goal.pos_x = src_i->goal.pos_x;
	// des_i->goal.pos_y = src_i->goal.pos_y;
	// des_i->goal.angle_z = src_i->goal.angle_z;
	des_i->path_idx = src_i->path_idx;
	des_i->joystick_x = src_i->joystick_x;
	des_i->joystick_y = src_i->joystick_y;
	
	des_i->path_traversed.size = src_i->path_traversed.size;
	for(int i=0;i< des_i->path_traversed.size;i++)
	{
		des_i->path_traversed.poses[i] =  src_i->path_traversed.poses[i];
	}
	des_i->contracted_path.size = src_i->contracted_path.size;
	for(int i=0;i< des_i->contracted_path.size;i++)
	{
		des_i->contracted_path.poses[i] =  src_i->contracted_path.poses[i];
	}
	des_i->adaptability = src_i->adaptability;

	des_i->collision_idx = src_i->collision_idx;

	des_i->num_intermediate_goals = src_i->num_intermediate_goals;

	// des_i->peds=(Dvc_PedStruct*)((void*)(des_i)+3*sizeof(Dvc_PedStruct));
	for(int i=0;i< des_i->num;i++)
	{
		des_i->peds[i].vel=src_i->peds[i].vel;
		des_i->peds[i].pos.x=src_i->peds[i].pos.x;
		des_i->peds[i].pos.y=src_i->peds[i].pos.y;
		des_i->peds[i].goal=src_i->peds[i].goal;
		des_i->peds[i].id=src_i->peds[i].id;
	}
}
DEVICE Dvc_State* Dvc_WheelchairPomdp::Dvc_Get(Dvc_State* particles, int pos) {
	Dvc_PomdpState* particle_i= static_cast<Dvc_PomdpState*>(particles)+pos;

	return particle_i;
}

DEVICE float Dvc_WheelchairPomdp::Dvc_GetCarVel(Dvc_State* particles, int pos) {
	Dvc_PomdpState* particle_i= static_cast<Dvc_PomdpState*>(particles)+pos;

	return particle_i->wheelchair.vel_v;
}


//TODO modify this function based on wheelchair pomdo model
DEVICE Dvc_ValuedAction Dvc_WheelchairPomdp::Dvc_GetBestAction() {
	return Dvc_ValuedAction(LINEAR_MINUS, -1);
	//return Dvc_ValuedAction(0,
	//		Dvc_ModelParams::CRASH_PENALTY * (Dvc_ModelParams::VEL_MAX*Dvc_ModelParams::VEL_MAX + Dvc_ModelParams::REWARD_BASE_CRASH_VEL));
}

void  WheelchairDSPOMDP::ReadParticlesBackToCPU(std::vector<State*>& particles ,const Dvc_State* dvc_particles,
			bool deepcopy) const
{
	for (int i=0;i<particles.size();i++)
	{
		const Dvc_PomdpState* src=static_cast<const Dvc_PomdpState*>(dvc_particles)+i;
		WheelchairState* des=static_cast<WheelchairState*>(particles[i]);
		Dvc_PomdpState::ReadMainStateBackToCPU(src,des);
	}
	Dvc_PomdpState::ReadPedsBackToCPU(
			static_cast<const Dvc_PomdpState*>(dvc_particles),
			particles);
}

DEVICE int Dvc_WheelchairPomdp::Dvc_ReachingCheck(Dvc_PomdpState& wheelchair_state, bool& reaching_goal) 
{
	float dist2goal = 0;
	float reward = 0;
	//float goal_yaw = 0;
	int n = 0;
	float agent_yaw = wheelchair_state.wheelchair.angle_z;
	double goal_yaw = 0;
	//dist2goal = sqrtf(powf((wheelchair_state.wheelchair.pos_x - wheelchair_state.goal.pos_x), 2) 
	//	+ powf((wheelchair_state.wheelchair.pos_y - wheelchair_state.goal.pos_y), 2));
	int goal_path_size = intermediate_goal_list[wheelchair_state.path_idx].size_;
	for(int i = goal_path_size - 1; i>=0; --i )
	{
		dist2goal = hypotf(wheelchair_state.wheelchair.pos_x - intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].x, 
							wheelchair_state.wheelchair.pos_y - intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].y);
		goal_yaw = intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].z ;

		if (dist2goal <= 0.25 && fabs(goal_yaw - agent_yaw) <= M_PI / 6)
		{
			// check which intermediate goal can be reached
			n = i + 1;
			// wheelchair_state.state_goal_point = intermediate_goal_list.paths[wheelchair_state.path_idx].poses[i].pose.position;
			break;
		}

	}

	if (n == goal_path_size)
	{
		reaching_goal = true;
	}
	else
	{
		reaching_goal = false;
	}

	if (wheelchair_state.num_intermediate_goals < n)
	{
		for (int i = wheelchair_state.num_intermediate_goals; i < n; ++i)
		{
			reward += (i + 1) * Dvc_ModelParams::reaching_reward / goal_path_size;
		}
	}
	wheelchair_state.num_intermediate_goals = n;

	return reward;
	// wheelchair_state.state_goal_index = n;
	// n = 0, then no reaching, n = size of path, then reach the final goal
	// reward the agent based on the position along the path that it could reach
	// if it is the last point along the path, then the agent reaches the final goal
	//return n;
		
}

DEVICE float Dvc_WheelchairPomdp::Dvc_TransitWheelchair(Dvc_PomdpState& wheelchair_state, Eigen::Quaternionf & map_quat, int& transition_steps, float v_before, float v_after, float w_before, float w_after) 

{
	float reward = 0;
	transition_steps = 1;	// default number of transition steps, except for stop action

	int linear_steps = ceilf(fabs(v_after - v_before) / (Dvc_ModelParams::transition_time * Dvc_ModelParams::max_v_acceleration));
	int angular_steps = ceilf(fabs(w_after - w_before) / (Dvc_ModelParams::transition_time * Dvc_ModelParams::max_w_acceleration));
	
	if (linear_steps > 1 || angular_steps > 1)
		transition_steps = linear_steps > angular_steps ? linear_steps : angular_steps;

	float yaw = wheelchair_state.wheelchair.angle_z;
	float penalty_idx = 0, highest_penalty = 0;
	float first_phase = 0.5 * Dvc_ModelParams::repsonse_time;
	float second_phase = Dvc_ModelParams::transition_time - first_phase;

	float v_temp = 0, w_temp = 0, v_step = 0, w_step = 0;

	if (transition_steps == 1)	// only requires 1 step to finish the velocity change
	{
		// the first phase, namely response phase, 0.1s
		if (w_before == 0)
		{
			wheelchair_state.wheelchair.pos_x += v_before * cosf(yaw) * first_phase;
			wheelchair_state.wheelchair.pos_y += v_before * sin(yaw) * first_phase;
		}
		else
		{
			wheelchair_state.wheelchair.pos_x += v_before / w_before * (sinf(yaw + w_before * first_phase) - sinf(yaw));
			wheelchair_state.wheelchair.pos_y += v_before / w_before * (cosf(yaw) - cosf(yaw + w_before * first_phase));
			yaw += w_before * first_phase;
		}
		// do the collision check
		penalty_idx = Dvc_CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;

		// the second phase, 0.2s
		v_temp = 0.5 * (v_before + v_after);
		w_temp = 0.5 * (w_before + w_after);
		// cout << "v_before " << v_before << ", v_after " << v_after << endl;

		if (w_temp == 0)
		{
			wheelchair_state.wheelchair.pos_x += v_temp * cosf(yaw) * second_phase;
			wheelchair_state.wheelchair.pos_y += v_temp * sinf(yaw) * second_phase;
		}
		else
		{
			wheelchair_state.wheelchair.pos_x += v_temp / w_temp * (sinf(yaw + w_temp * second_phase) - sinf(yaw));
			wheelchair_state.wheelchair.pos_y += v_temp / w_temp * (cosf(yaw) - cosf(yaw + w_temp * second_phase));
			yaw += w_temp * second_phase;
		}
		// do the collision check
		penalty_idx = Dvc_CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
	}
	else	// requires multiple steps to finish the velocity change
	{
		float v_sign = v_after > v_before ? 1 : -1;
		float w_sign = w_after > w_before ? 1 : -1;
		// the first phase, namely response phase, 0.1s
		if (w_before == 0)
		{
			wheelchair_state.wheelchair.pos_x += v_before * cosf(yaw) * first_phase;
			wheelchair_state.wheelchair.pos_y += v_before * sinf(yaw) * first_phase;
		}
		else
		{
			wheelchair_state.wheelchair.pos_x += v_before / w_before * (sinf(yaw + w_before * first_phase) - sinf(yaw));
			wheelchair_state.wheelchair.pos_y += v_before / w_before * (cosf(yaw) - cosf(yaw + w_before * first_phase));
			yaw += w_before * first_phase;
		}
		// do the collision check
		penalty_idx = Dvc_CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		
		// the second phase, 0.2s
		// first check if v and w has been changed to v_after and w_after
		if (linear_steps > 1)
		{
			v_temp = 0.5 * (2 * v_before + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_v_acceleration * v_sign);
			v_step = v_before + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_v_acceleration * v_sign;
			linear_steps--;
		}
		else
		{
			v_temp = 0.5 * (v_before + v_after);
			v_step = v_after;
		}
		if (angular_steps > 1)
		{
			w_temp = 0.5 * (2 * w_before + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_w_acceleration * w_sign);
			w_step = w_before + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_w_acceleration * w_sign;
			angular_steps--;
		}
		else
		{
			w_temp = 0.5 * (w_before + w_after);
			w_step = w_after;
		}
		// transition for the second phase 0.2s
		if (w_temp == 0)
		{
			wheelchair_state.wheelchair.pos_x += v_temp * cosf(yaw) * second_phase;
			wheelchair_state.wheelchair.pos_y += v_temp * sinf(yaw) * second_phase;
		}
		else
		{
			wheelchair_state.wheelchair.pos_x += v_temp / w_temp * (sinf(yaw + w_temp * second_phase) - sinf(yaw));
			wheelchair_state.wheelchair.pos_y += v_temp / w_temp * (cosf(yaw) - cosf(yaw + w_temp * second_phase));
			yaw += w_temp * second_phase;
		}
		// do the collision check
		penalty_idx = Dvc_CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		
		// after the first transition step, finish the rest steps without response time
		for (int i = 0; i < transition_steps - 1; ++i)
		{
			if (linear_steps > 1)
			{
				v_temp = 0.5 * (2 * v_step + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_v_acceleration * v_sign);
				v_step = v_step + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_v_acceleration * v_sign;
				linear_steps--;
			}
			else
			{
				v_temp = 0.5 * (v_step + v_after);
				v_step = v_after;
			}
			if (angular_steps > 1)
			{
				w_temp = 0.5 * (2 * w_step + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_w_acceleration * w_sign);
				w_step = w_step + Dvc_ModelParams::transition_time * Dvc_ModelParams::max_w_acceleration * w_sign;
				angular_steps--;
			}
			else
			{
				w_temp = 0.5 * (w_temp + w_after);
				w_step = w_after;
			}
			// transition
			if (w_temp == 0)
			{
				wheelchair_state.wheelchair.pos_x += v_temp * cosf(yaw) * second_phase;
				wheelchair_state.wheelchair.pos_y += v_temp * sinf(yaw) * second_phase;
			}
			else
			{
				wheelchair_state.wheelchair.pos_x += v_temp / w_temp * (sinf(yaw + w_temp * second_phase) - sinf(yaw));
				wheelchair_state.wheelchair.pos_y += v_temp / w_temp * (cosf(yaw) - cosf(yaw + w_temp * second_phase));
				yaw += w_temp * second_phase;
			}
			// do the collision check
			penalty_idx = Dvc_CollisionCheck(wheelchair_state.wheelchair, map_quat);

			if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		}
	}

	// finish 2 phases
	wheelchair_state.collision_idx = highest_penalty;

	wheelchair_state.wheelchair.vel_v = v_after;
	wheelchair_state.wheelchair.vel_w = w_after;

	


	/*	Update joystick input, since joystick input direction keeps unchanged during the search,
		when the wheelchair rotates for an angle A, the joystick direction rotates for -A w.r.t. the wheelchair frame */
	//wheelchair_quat.setRPY(0, 0,  - (yaw - wheelchair_state.wheelchair.angle_z));
	Eigen::Quaternionf wheelchair_quat = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	* Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ());

	Eigen::Vector3f joystick_heading(wheelchair_state.joystick_x - wheelchair_state.wheelchair.pos_x,
	 wheelchair_state.joystick_y - wheelchair_state.wheelchair.pos_y, 
	 0);
	joystick_heading = wheelchair_quat.matrix()*joystick_heading;
	wheelchair_state.joystick_x = roundf(joystick_heading[0] * 100) / 100;
	wheelchair_state.joystick_y = roundf(joystick_heading[1] * 100) / 100;

	wheelchair_state.wheelchair.angle_z = yaw;

	// compute the reward

	if (highest_penalty == 1) // A severe collision happens
	{
		// collision penalty plus an extra penalty based on the current linear velocity
		reward = Dvc_ModelParams::collision_penalty * (1 + fabs(wheelchair_state.wheelchair.vel_v));
		// return true;	// Only terminate the task when a severe collision occurs
	}
	else if (highest_penalty != 0)	// Transition succeeds with mild collision
	{
		// basic penalty for mild collision plus an extra penalty based on the current linear velocity
		if (fabs(wheelchair_state.wheelchair.vel_v) > 0.25)
		{
			reward = 0.5 * Dvc_ModelParams::inflation_max_penalty * (1 + highest_penalty) * (fabs(wheelchair_state.wheelchair.vel_v) - 0.25) / 0.25;
		}
		else
		{
			reward = 0.5 * Dvc_ModelParams::inflation_basic_penalty * (1 + highest_penalty);
			// reward += (ModelParams::inflation_high_penalty - ModelParams::inflation_basic_penalty) * penalty_idx + ModelParams::inflation_basic_penalty;
			// reward += (ModelParams::inflation_max_penalty - ModelParams::inflation_basic_penalty) * fabs(wheelchair_status.agent_velocity.linear.x)
			// 	+ ModelParams::inflation_basic_penalty;	
		}	
	}
	
	return reward;			// return the reward for transition

	// float yaw = wheelchair_status.angle_z;
	// float step_time = 1/freq; //Updated from Model_params::control_freq in Init_GPU.cu
	// int loop_times = roundf(transition_time / step_time);
	// float penalty_idx = 0;
	// float accumulative_penalty = 0;
	// for (int i = 0; i < loop_times; i ++)
	// {
	// 	penalty_idx = Dvc_CollisionCheck(wheelchair_status);
	// 	if (penalty_idx == 1)
	// 	{
	// 		return penalty_idx;	// A severe collision happens, transition fails, the task is terminated.
	// 	}
	// 	if (wheelchair_status.vel_w == 0)
	// 	{
	// 		wheelchair_status.pos_x += wheelchair_status.vel_v * cosf(yaw) * step_time;
	// 		wheelchair_status.pos_y += wheelchair_status.vel_v * sinf(yaw) * step_time;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_status.pos_x += wheelchair_status.vel_v / wheelchair_status.vel_w * 
	// 			(sinf(yaw + wheelchair_status.vel_w * step_time) - sinf(yaw));
	// 		wheelchair_status.pos_y += wheelchair_status.vel_v / wheelchair_status.vel_w * 
	// 			(cosf(yaw) - cosf(yaw + wheelchair_status.vel_w * step_time));
	// 		yaw += wheelchair_status.vel_w * step_time;
	// 	}
	// 	accumulative_penalty += penalty_idx;
	// }
	// penalty_idx = accumulative_penalty / (1.0*loop_times);
	
	// //Update yaw
	// wheelchair_status.angle_z = yaw;

	// return penalty_idx;
}

DEVICE float Dvc_WheelchairPomdp::Dvc_FollowingVel(Eigen::Vector3f joystick_heading, float v_before, float w_before, float& v_follow, float& w_follow, float v_max)
{
	// follow the user action
	if (v_max < fabs(v_before))
	{
		v_max = v_before;
	}
	Eigen::Vector3f velocity_heading(0, 0, 0);
	float v_sign = 1, w_sign = 1;
	if(v_before < 0)
	{
		v_sign = -1;
	}
	if(w_before < 0)
	{
		w_sign = -1;
	}
	float joystick_heading_length = joystick_heading.norm();
	// generate the small 5*5 dynamic window
	// max acc = 1 m/s^2, so in the transition time 0.3s, the max velocity change is 0.3*1 = 0.3
	// float small_dw_v[5][5], small_dw_w[5][5];
	float step_size_v = 0.5 * Dvc_ModelParams::max_v_acceleration * Dvc_ModelParams::transition_time;
	float step_size_w = 0.5 * Dvc_ModelParams::max_w_acceleration * Dvc_ModelParams::transition_time;
	// step_size_v = step_size_v/2.0;
	// step_size_w = step_size_w/2.0;
	// find the nearest v, w pair
	float nearest_v = 0, nearest_w = 0, angle_diff = 0, vel_diff = 0, temp_v = 0, temp_w = 0, temp_cost = 0, total_cost = 10000;
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 5; ++j)
		{
			temp_v = v_before + (i - 2) * step_size_v;
			temp_w = w_before + (j - 2) * step_size_w;
			temp_v = fabs(temp_v) > v_max ? v_sign * v_max : temp_v;
			temp_w = fabs(temp_w) > Dvc_ModelParams::max_angular_speed ? w_sign * Dvc_ModelParams::max_angular_speed : temp_w;
			// cout << "i = " << i << ", j = " << j << endl;
			// cout << "temp_v = " << temp_v << ", temp_w = " << temp_w << endl;
			velocity_heading[0] = temp_v; //velocity_heading.setX(temp_v);
			velocity_heading[1] = temp_w; //velocity_heading.setY(temp_w);
			// small_dw_v[i][j] = wheelchair_status.agent_velocity.linear.x + (i - 2) * step_size_v;
			// small_dw_w[i][j] = wheelchair_status.agent_velocity.angular.z + (j - 2) * step_size_w;
			float velocity_heading_length = velocity_heading.norm();
			if (joystick_heading_length == 0)	// no joystick input
			{
				// stop will have no cost, other actions have highest angle cost and vel cost based on the length
				angle_diff = velocity_heading_length == 0 ? 0 : 1;
				vel_diff = velocity_heading_length;
			}
			else
			{
				if (velocity_heading_length == 0)
				{
					angle_diff = 1;
				}
				else
				{
					angle_diff = acosf(joystick_heading.dot(velocity_heading)/(joystick_heading_length * velocity_heading_length))/3.14159;
				}				
				vel_diff = fabs(joystick_heading_length - velocity_heading_length) / joystick_heading_length;
			}
			temp_cost = Dvc_ModelParams::weight_heading * angle_diff + Dvc_ModelParams::weight_velocity * vel_diff;
			// cout << "temp_cost = " << temp_cost << endl;
			if (temp_cost < total_cost)
			{
				nearest_v = temp_v;
				nearest_w = temp_w;
				total_cost = temp_cost;
			}
		}
	}
	// cout << "nearest_v = " << nearest_v << endl;
	// cout << "nearest_w = " << nearest_w << endl;
	v_follow = roundf(nearest_v * 100) / 100;
	w_follow = roundf(nearest_w * 100) / 100;
	total_cost = powf(total_cost, 0.25);
	return total_cost;
}
DEVICE int Dvc_WheelchairPomdp::Dvc_TurningSteps(float& angle2turn, float& current_w, int& case_num) 
{
	// case 0: angular+
	// case 1: angular-
	// case 2: keep

	float time2turn = 0;
	// max sector: accelerate to max speed and decelerate to 0
	float maxsector = Dvc_ModelParams::max_angular_speed * Dvc_ModelParams::max_angular_speed / Dvc_ModelParams::max_w_acceleration;
	// the wheelchair is not turning currently
	if (current_w == 0)
	{
		case_num = angle2turn >= 0 ? 0 : 1;

		// check if maxsector can cover the angle2turn

		// angle2turn can be finished inside max sector
		if (maxsector >= fabs(angle2turn))
		{
			time2turn = 2 * sqrtf(fabs(angle2turn) / Dvc_ModelParams::max_w_acceleration);
		}
		// accelerate to max, keep and decelerate to 0
		else
		{
			time2turn = 2 * (Dvc_ModelParams::max_angular_speed / Dvc_ModelParams::max_w_acceleration) + (fabs(angle2turn) - maxsector) / Dvc_ModelParams::max_angular_speed;
		}
	}

	// the wheelchair is turning now
	else
	{
		// the wheelchair is rotating towards the path
		if (current_w * angle2turn > 0)
		{
			// first check when exerting max deceleration, how many radians can be finished before stop
			float sector2stop = 0.5 * current_w * current_w / Dvc_ModelParams::max_w_acceleration;

			// the wheelchair cannot stop when facing the path
			if (fabs(sector2stop) > fabs(angle2turn))
			{
				case_num = current_w > 0 ? 1 : 0;
				// add the time to stop to the total time first
				time2turn += fabs(current_w)/ Dvc_ModelParams::max_w_acceleration;
				// then it becomes the first situation, wheelchair not moving, the rest angle to turn is sector2stop - angle2turn

				if (maxsector >= fabs(sector2stop) - fabs(angle2turn))
				{
					time2turn += 2 * sqrt((fabs(sector2stop) - fabs(angle2turn)) / Dvc_ModelParams::max_w_acceleration);
				}
				// accelerate to max, keep and decelerate to 0
				else
				{
					time2turn += 2 * (Dvc_ModelParams::max_angular_speed / Dvc_ModelParams::max_w_acceleration) + (fabs(sector2stop) - fabs(angle2turn) - maxsector) / Dvc_ModelParams::max_angular_speed;
				}
			}

			// the wheelchair can stop when facing the path
			else
			{
				// the fastest action sequence is to accelerate to max first, keep, and decelerate to 0 then

				// check the max sector
				float time2max = (Dvc_ModelParams::max_angular_speed - fabs(current_w)) / Dvc_ModelParams::max_w_acceleration;
				float current_maxsector = 0.5 * maxsector + 0.5 * (fabs(current_w) + Dvc_ModelParams::max_angular_speed) * time2max;

				// accelerate and decelerate
				if (current_maxsector > fabs(angle2turn))
				{
					// solve the Quadratic Equation, where acceleration time t is the unknown value
					// 2*(a*t)^2 + 4*w*a*t + w^2 - 2*a*S = 0, a: acceleration, w: current_w, S: the angle to turn
					// A = 2*a^2, B = 4*w*a, C = w^2 - 2*a*S, the solutions: (-B ± sqrt(B^2- 4*A*C)) / (2*A)
					float A = 2 * Dvc_ModelParams::max_w_acceleration * Dvc_ModelParams::max_w_acceleration;
					float B = 4 * fabs(current_w) * Dvc_ModelParams::max_w_acceleration;
					float C = current_w * current_w - 2 * Dvc_ModelParams::max_w_acceleration * fabs(angle2turn);
					float time1 = (-B + sqrt(B*B- 4*A*C)) / (2*A), time2 = (-B - sqrt(B*B- 4*A*C)) / (2*A);
					if (time2 <= 0)
					{
						time2turn += time1;
						time2turn += (fabs(current_w) + Dvc_ModelParams::max_w_acceleration * time1) / Dvc_ModelParams::max_w_acceleration;
					}
					else
					{
						time2turn += time2;
						time2turn += (fabs(current_w) + Dvc_ModelParams::max_w_acceleration * time2) / Dvc_ModelParams::max_w_acceleration;
					}
				}
				// accelerate to max, keep and decelerate
				else
				{
					// first compute the time to keep max w
					time2turn += (fabs(angle2turn) - current_maxsector) / Dvc_ModelParams::max_angular_speed;
					// then add the acceleration time and deceleration time
					time2turn += time2max + Dvc_ModelParams::max_angular_speed / Dvc_ModelParams::max_w_acceleration;
				}
				case_num = current_w > 0 ? 0 : 1;
			}

		}

		// the wheelchair is rotating away from the path
		else
		{
			// stop the wheelchair first by exerting max deceleration
			time2turn += fabs(current_w) / Dvc_ModelParams::max_w_acceleration;
			// radians that can be finished before stop
			float sector2stop = 0.5 * current_w * current_w / Dvc_ModelParams::max_w_acceleration;
			// then it becomes the first situation, wheelchair not moving, the rest angle to turn is sector2stop + angle2turn
			// but if the sum is greater than PI, namely 180 degrees, the angle to turn is just 2*PI - sum
			float sum_angle2turn = fabs(sector2stop) + fabs(angle2turn);
			if (fabs(sector2stop) + fabs(angle2turn) > M_PI)
				sum_angle2turn = 2 * M_PI - sum_angle2turn;

			if (maxsector >= fabs(sum_angle2turn))
			{
				time2turn += 2 * sqrtf((fabs(sum_angle2turn)) / Dvc_ModelParams::max_w_acceleration);
			}
			// accelerate to max, keep and decelerate to 0
			else
			{
				time2turn += 2 * (Dvc_ModelParams::max_angular_speed / Dvc_ModelParams::max_w_acceleration) + (fabs(sum_angle2turn) - maxsector) / Dvc_ModelParams::max_angular_speed;
			}
			case_num = current_w > 0 ? 1 : 0;
		}

	}

	return ceilf(time2turn / Dvc_ModelParams::transition_time);
}


DEVICE float Dvc_WheelchairPomdp::Dvc_CalAngleDiff(const Dvc_WheelchairStruct& wheelchair_status, int &goal_idx)
{
	float angle_difference;
	Eigen::Vector3f agent_heading(1,0,0);
	Eigen::Vector3f agent2goal(0,0,0);
	//Eigen::Matrix3f m_agent_heading;
	//m_agent_heading = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
	Eigen::Quaternionf q_heading = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	* Eigen::AngleAxisf(wheelchair_status.angle_z, Eigen::Vector3f::UnitZ());
	//Eigen::Quaternionf q_heading(m_agent_heading);
	Eigen::Matrix3f rotation_mat = q_heading.matrix();
	agent_heading = rotation_mat*agent_heading;
	agent2goal[0] = goal_positions[goal_idx].x - wheelchair_status.pos_x;
	agent2goal[1] = goal_positions[goal_idx].y - wheelchair_status.pos_y;
	angle_difference = acosf(agent_heading.dot(agent2goal)/(agent_heading.norm() * agent2goal.norm()));

	// use cross product to judge if the angle is positive or negative
	float cross_product = agent_heading[0] * agent2goal[1] - agent_heading[1] * agent2goal[0];
	if(cross_product < 0)
	{
		angle_difference = -angle_difference;

	}
	
	return angle_difference;
}


DEVICE float Dvc_WheelchairPomdp::Dvc_CollisionCheck(const Dvc_WheelchairStruct& wheelchair_status, const Eigen::Quaternionf& map_quat) 
{
	float x_check = wheelchair_status.pos_x;
	float y_check = wheelchair_status.pos_y;

	Eigen::Vector3f check_vector(x_check, y_check, 0);
	check_vector = map_quat.matrix()* check_vector;

	x_check = check_vector.x();
	y_check = check_vector.y();

	int col_check = x_check >= 0 ? ceilf(x_check / map_resolution) : floorf(x_check / map_resolution);
	int row_check = y_check >= 0 ? ceilf(y_check / map_resolution) : floorf(y_check / map_resolution);

	int local_costmap_index_value = Dvc_GetLocalCostmapIndexValue((y_center + row_check) , (x_center + col_check));
	int local_costmap_value = local_costmap_data[local_costmap_index_value];
	if (local_costmap_value >= Dvc_ModelParams::outer_pixel_thres)
	{
		return 1;
	}
	else if (local_costmap_value >= Dvc_ModelParams::inner_pixel_thres)
	{
		return (static_cast<float>(local_costmap_value) - Dvc_ModelParams::inner_pixel_thres) / (Dvc_ModelParams::outer_pixel_thres - Dvc_ModelParams::inner_pixel_thres);
	}
    else
    {
        return 0;
    }
	// float x_check = wheelchair_status.pos_x;
	// float y_check = wheelchair_status.pos_y;
	// float r_collision = 0.42;
	// float r_outer = 0.45;
	// float temp_dist = 0;
	// float penalty_idx = 0;

	// //for (const auto &point : lidar_points)
	// for(int i = 0; i < Dvc_ModelParams::LIDAR_POINTS_SIZE; i++)
	// {

	// 	const Dvc_3DCOORD& point = lidar_points[i];
	// 	//https://stackoverflow.com/a/7227057
	// 	//Check through easy conditions first, getting distance with sqrt is computationally expensive
	// 	double dx = fabs(point.x - x_check);
	// 	double dy = fabs(point.y - y_check);

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
	// 		else if (temp_dist <= r_outer)
	// 		{
	// 			penalty_idx = 1 - (temp_dist - r_collision) / (r_outer - r_collision);
	// 		}
	// 	}
	// }
	// return penalty_idx;
}

DEVICE void Dvc_WheelchairPomdp::Dvc_GenerateNewPath(Dvc_WheelchairStruct &wheelchair_status, Dvc_PathStruct &original_path) 
{
	original_path.size = original_path.size + 1;
	original_path.poses[original_path.size -1].x = wheelchair_status.pos_x;
	original_path.poses[original_path.size -1].y = wheelchair_status.pos_y;
	original_path.poses[original_path.size -1].z = wheelchair_status.angle_z; 
}

//Assuming it wil happen only once. The original path is takem from intermediate_goal_list
DEVICE void Dvc_WheelchairPomdp::Dvc_ContractAndInterpolatePath(Dvc_WheelchairStruct &wheelchair_status, Dvc_PathStruct &path_traversed, Dvc_PathStruct &new_path, int path_index, Eigen::Quaternionf& map_quat)
{

	Dvc_3DCOORD current_position;
	current_position.x = wheelchair_status.pos_x;
	current_position.y = wheelchair_status.pos_y;
	current_position.z = wheelchair_status.angle_z;

	int voronoi_path_size = intermediate_goal_list[path_index].size_;
	Dvc_3DCOORD check_point;
	float dist2checkpoint = 0;
	// the index along the path whose point is the last collision free point
	int collision_free_index = 1000;

	Dvc_PathStruct temp_path;
	temp_path.size = 0;
	for (int i = path_traversed.size; i > - voronoi_path_size; --i)
	{
		if (i > 0)
		{
			check_point = path_traversed.poses[i-1];
		}
		else
		{
			check_point = intermediate_goal_list[path_index].way_points_[- i];
		}
		dist2checkpoint = sqrtf(powf(current_position.x - check_point.x, 2) + powf(current_position.y - check_point.y, 2));
		if (dist2checkpoint < map_resolution)	// distance is smaller than map_resolution
		{
			continue;	// skip this point and move to the next point
		}
		else	// distance >= map_resolution, form a straight line to link current_position and check_point to do collision check
		{
			float num_steps = dist2checkpoint / map_resolution;
			// cout << "num_steps " << num_steps << endl;
			// calculate linear increment in x and y to reduce computation costs
			float increment_x = (check_point.x - current_position.x) / num_steps;
			float increment_y = (check_point.y - current_position.y) / num_steps;

			Dvc_3DCOORD moving_point = current_position;

			for (int j = 1; j < ceilf(num_steps); ++j)
			{
				moving_point.x += increment_x;
				moving_point.y += increment_y;

				// cout << "x_check in local frame: " << moving_point.x << endl;
				// cout << "y_check in local frame: " << moving_point.y << endl;

				// add a point to the path every 0.2m, namely every 4 steps since the step size is 0.05m

				if (j % 4 == 0)
				{
					//pose2add.pose.position = moving_point;
					//temp_path.push_back(pose2add);
					temp_path.poses[temp_path.size] = moving_point;
					temp_path.size = temp_path.size + 1;
				}

				Eigen::Vector3f check_vector(moving_point.x, moving_point.y, 0);
				check_vector = map_quat.matrix()* check_vector;

				float x_check = check_vector[0];
				float y_check = check_vector[1];

				int col_check = x_check >= 0 ? ceilf(x_check / map_resolution) : floorf(x_check / map_resolution);
				int row_check = y_check >= 0 ? ceilf(y_check / map_resolution) : floorf(y_check / map_resolution);

				// cout << "x_check in costmap frame: " << x_check << endl;
				// cout << "y_check in costmap frame: " << y_check << endl;
				int local_costmap_index_value = Dvc_GetLocalCostmapIndexValue(y_center + row_check,  x_center + col_check);
				int local_costmap_value = local_costmap_data[local_costmap_index_value];
	
				if (local_costmap_value <= Dvc_ModelParams::pixel_path)	// no collision
				{
					continue;
				}
				else	// collision
				{
					collision_free_index = i + 1; // collision detected, so the last collision free point would be the previous one
					break;
				}
			}
		}
		if (collision_free_index != 1000)	// collision is already detected, quit the for loop
		{
			temp_path.size = 0;
			break;
		}
		else	// no collision along this check_point, move to the next check_point and store the current path
		{
			new_path.size = temp_path.size;
			for(int j = 0; j < temp_path.size; j++)
			{
				new_path.poses[j] = temp_path.poses[j];
			}
			
			temp_path.size = 0;
		}
		
	}

	// remove the points before collision_free_index on the original path, and add the interpolated path at the beginning to form the final new path

	// collision detected along the whole path
	// back_inserter doesn't need to declare space first
	if (collision_free_index != 1000)
	{
		// cout << "collision detected along the whole path" << endl;
		// first copy the interpolated path segment to the new path
		// declare the space for the container first
		// if the very first point is inside collision zone, reserve all the original path
		collision_free_index = collision_free_index == path_traversed.size + 1 ? path_traversed.size : collision_free_index;
		//new_path.resize(interpolated_path.size());
		//std::copy(interpolated_path.begin(), interpolated_path.end(), new_path.begin());
		int intermediate_goal_list_index = -collision_free_index;
		if (collision_free_index > 0)	// the path_traversed is not finished after the contraction
		{
			// copy the rest path_traversed into new path in a reverse order
			for (int j = collision_free_index; j > 0; --j)
			{
				new_path.poses[new_path.size] = path_traversed.poses[j - 1];
				new_path.size = new_path.size + 1;
			}
			intermediate_goal_list_index = 0;
			// copy the whole original path
			//std::copy(original_path.poses.begin(), original_path.poses.end(), std::back_inserter(new_path));
		}
		//else	// the path_traversed is finished after the contraction
		//{
			// copy the rest segment from the original path with points before collision_free_index removed
		for (int j = intermediate_goal_list_index; j < voronoi_path_size; j++)
		{
			//std::copy(original_path.poses.begin() - collision_free_index, original_path.poses.end(), std::back_inserter(new_path));
			new_path.poses[new_path.size] = intermediate_goal_list[path_index].way_points_[j];;
			new_path.size = new_path.size + 1;
		}
		//}		
		// cout << "original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << new_path.size() << endl;
		//original_path.poses.clear();
		//original_path.poses = new_path;
		// cout << "after clearing, original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << new_path.size() << endl;
	}
	else	// no collision detected along the whole path, the whole original path is discarded
	{
		// cout << "no collision detected along the whole path" << endl;
		// cout << "original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << interpolated_path.size() << endl;
		//original_path.poses.clear();
		//original_path.poses = interpolated_path;
		//pose2add.pose.position = check_point;
		//original_path.poses.push_back(pose2add);
		new_path.poses[new_path.size] = check_point;
		new_path.size = new_path.size + 1;
		// cout << "after clearing, original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << interpolated_path.size() << endl;
	}
	path_traversed.size = 0;

}

DEVICE int Dvc_WheelchairPomdp::Dvc_GetLocalCostmapIndexValue(int i, int j)
{
	return (i*local_costmap_cols) +  j;
}
DEVICE float Dvc_WheelchairPomdp::Dvc_FollowUserReward(float v_follow, float w_follow, float v_after, float w_after) 
{
	if (v_after == v_follow && w_after == w_follow)
	{
		return Dvc_ModelParams::user_following_reward;
	}
	else //Small Dynamic Window
	{
		Eigen::Vector3f velocity_heading(v_after, w_after, 0);		
		Eigen::Vector3f follow_heading(v_follow, w_follow, 0);

		float follow_length = follow_heading.norm();
		float velocity_length = velocity_heading.norm();
		float heading_diff = 0, vel_diff = 0;
		if (follow_length == 0)	// (v_follow, w_follow) is 0, following user is to stop
		{
			heading_diff = velocity_length == 0 ? 0 : 1;	// if (v_after, w_after) is also 0, then no cost, otherwise highest cost
			vel_diff = velocity_length;	// cost equals the length of (v_after, w_after)
		}
		else	// (v_follow, w_follow) is non-zero, following user is to move
		{

			float angle_difference = acosf(follow_heading.dot(velocity_heading)/(follow_length * velocity_length));

			// if (v_after, w_after) is 0, then highest cost, otherwise based on angle discrepancy
			heading_diff = velocity_length == 0 ? 1 : angle_difference / M_PI;
			// cost equals the length difference between (v_after, w_after) and (v_follow, w_follow) 
			vel_diff = fabs(follow_length - velocity_length) / follow_length;
		}
		// (v_combined == 0 ? 0 : fabs(dynamic_window.at<cv::Vec2f>(i, j)[0] - v_combined) / fabs(v_combined));

		heading_diff = powf(heading_diff, 0.25);
		
		float total_cost = Dvc_ModelParams::weight_heading * heading_diff + Dvc_ModelParams::weight_velocity * vel_diff;

		// float reward_idx = 1 - total_cost;

		return (0.5 - total_cost) * 2 * Dvc_ModelParams::user_following_reward;
	}
}

	

DEVICE void Dvc_WheelchairPomdp::Dvc_PrintObs(Dvc_WheelchairObs& wheelchair_obs, long& obs_int) 
{
	// joystick + pos_x + pos_y + angle_z + vel_v + vel_w
	//std::string obs_string;
	//obs_string = std::to_string(wheelchair_obs.joystick_obs) + std::to_string(wheelchair_obs.obs_pos_x) + std::to_string(wheelchair_obs.obs_pos_y) +
	//	std::to_string(wheelchair_obs.obs_angle_z) + std::to_string(wheelchair_obs.obs_vel_v) + std::to_string(wheelchair_obs.obs_vel_w);
	
	//obs_int = std::stoi(obs_string);
}

DEVICE void Dvc_WheelchairPomdp::Dvc_PrintDvcState(Dvc_State* particle)
{
	Dvc_PomdpState* wheelchairpomdp_state = static_cast<Dvc_PomdpState*>(particle);
	//Dvc_WheelchairStruct& wheelchairpomdp_status = wheelchairpomdp_state->wheelchair;
	// int goal_point_idx = wheelchairpomdp_state->path_idx;
	//Dvc_GoalStruct& wheelchair_goal = wheelchairpomdp_state.goal;
	printf("Printing state: \n\
	GPU idx = %d.\n\
	Block idx.x = %d, y = %d, thread idx.x = %d, y = %d.\n\
	Scenario id = %d.\n\
	Particle weight = %f, state_id = %d.\n\
	Position: x = %f, y = %f, theta = %f.\n\
	Velocity: v = %f, w = %f.\n\
	Path index = %d.\n\
	Goal Position: x = %f, y = %f\n",
	blockDim.x * blockIdx.x + threadIdx.x,
	blockIdx.x, blockIdx.y, threadIdx.x, threadIdx.y,
	wheelchairpomdp_state->scenario_id,
	wheelchairpomdp_state->weight, wheelchairpomdp_state->state_id,
	wheelchairpomdp_state->wheelchair.pos_x, wheelchairpomdp_state->wheelchair.pos_y, wheelchairpomdp_state->wheelchair.angle_z,
	wheelchairpomdp_state->wheelchair.vel_v, wheelchairpomdp_state->wheelchair.vel_w,
	wheelchairpomdp_state->path_idx,
	goal_positions[wheelchairpomdp_state->path_idx].x, goal_positions[wheelchairpomdp_state->path_idx].y);

}

