#include "GPU_wheelchair_model.h"
#include <despot/GPUcore/thread_globals.h>

#include <wheelchair_pomdp/wheelchair_model.h>
#include <despot/util/coord.h>
#include <driver_types.h>
#include <stddef.h>
#include "despot/GPUutil/GPUmemorypool.h"
#include "despot/GPUutil/GPUrandom.h"

#include "GPU_WheelchairUpperBound.h"
#include <Eigen/Geometry>

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
DEVICE float transition_time=0;
DEVICE double in_front_angle_cos=0;
DEVICE Dvc_3DCOORD* lidar_points=NULL;
DEVICE Dvc_COORD* goal_positions=NULL;
DEVICE Dvc_Path* intermediate_goal_list=NULL;
DEVICE float external_joy_x = 0;
DEVICE float external_joy_y = 0;




using namespace despot;
/* ==============================================================================
 * Dvc_PomdpState class
 * ==============================================================================*/

DEVICE Dvc_PomdpState::Dvc_PomdpState():num(0), peds(NULL)
{
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
	
	dvc_particles[scenarioID].num= 0; //hst_particle->num; Making it zero. When we add dynamic scene, this should be set back to hst_particle->num



	dvc_particles[scenarioID].weight=hst_particle->weight;
	dvc_particles[scenarioID].state_id=hst_particle->state_id;
	dvc_particles[scenarioID].scenario_id=hst_particle->scenario_id;
	dvc_particles[scenarioID].path_idx=hst_particle->path_idx;

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

	//hst_particle->num=Hst_temp_mainstates[ThreadID]->num;
	hst_particle->weight=Hst_temp_mainstates[ThreadID]->weight;
	hst_particle->state_id=Hst_temp_mainstates[ThreadID]->state_id;
	hst_particle->scenario_id=Hst_temp_mainstates[ThreadID]->scenario_id;
	hst_particle->path_idx=Hst_temp_mainstates[ThreadID]->path_idx;
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

	float wheelchairpomdp_status_vel_v = roundf(wheelchairpomdp_state.wheelchair.vel_v*100)/100; //round(wheelchairpomdp_status.vel_v * 100) / 100;
	float wheelchairpomdp_status_vel_w = roundf(wheelchairpomdp_state.wheelchair.vel_w*100)/100; //round(wheelchairpomdp_status.vel_w * 100) / 100;
	
	if (action == LINEAR_PLUS)
	{
		if (wheelchairpomdp_status_vel_v > max_linear_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchairpomdp_status_vel_v < -0.1 && wheelchairpomdp_status_vel_v > -0.2)
				wheelchairpomdp_state.wheelchair.vel_v += (-wheelchairpomdp_status_vel_v);
			else
				wheelchairpomdp_state.wheelchair.vel_v += 0.2; // Increase the linear speed by 0.2
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == LINEAR_MINUS)
	{
		if (wheelchairpomdp_status_vel_v < -max_linear_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchairpomdp_status_vel_v > 0.1 && wheelchairpomdp_status_vel_v < 0.2)
				wheelchairpomdp_state.wheelchair.vel_v += (-wheelchairpomdp_status_vel_v);
			else
				wheelchairpomdp_state.wheelchair.vel_v -= 0.2; // Decrease the linear speed by 0.2
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}	
	}
	else if (action == ANGULAR_PLUS)
	{
		if ( wheelchairpomdp_status_vel_w > Dvc_ModelParams::max_angular_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchairpomdp_status_vel_w < -0.1 && wheelchairpomdp_status_vel_w > -0.2)
				wheelchairpomdp_state.wheelchair.vel_w += (-wheelchairpomdp_status_vel_w);
			else
				wheelchairpomdp_state.wheelchair.vel_w += 0.2; // Increase the angular speed by 0.2
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == ANGULAR_MINUS)
	{
		if (wheelchairpomdp_status_vel_w < -Dvc_ModelParams::max_angular_speed)
		{
			reward = Dvc_ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchairpomdp_status_vel_w > 0.1 && wheelchairpomdp_status_vel_w < 0.2)
				wheelchairpomdp_state.wheelchair.vel_w += (-wheelchairpomdp_status_vel_w);
			else
				wheelchairpomdp_state.wheelchair.vel_w -= 0.2; // Decrease the angular speed by 0.2
			reward = Dvc_ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == KEEP)
	{
		reward = Dvc_ModelParams::step_penalty; // Penalize the step
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

	// add white noise to the current velocity
	// according to the log file, the velocity diff can be as high as 0.04, so the tolerance will be ±0.04
	if (Dvc_ModelParams::using_probabilistic_model)
	{
		wheelchairpomdp_state.wheelchair.vel_v += (rand_num - 0.5) * 2 * Dvc_ModelParams::noise_amplitude;
		wheelchairpomdp_state.wheelchair.vel_w += (rand_num - 0.5) * 2 * Dvc_ModelParams::noise_amplitude;
	}
	
	// Transit to the next state with a penalty_idx [0, 1] as a return value
	float penalty_idx = Dvc_TransitWheelchair(wheelchairpomdp_status);
	
	if (penalty_idx == 1) // A severe collision happens
	{
		reward += Dvc_ModelParams::collision_penalty;
		return true;	// Only terminate the task when a severe collision occurs
	}
	else if (penalty_idx != 0)	// Transition succeeds with mild collision
	{
		// -100 basic penalty for mild collision plus an extra penalty based on the distance to obstacles
		reward += (Dvc_ModelParams::collision_penalty - Dvc_ModelParams::inflation_basic_penalty) * penalty_idx + Dvc_ModelParams::inflation_basic_penalty;
	}
	// printf("Reward after TransitWheelchair: %d.\n", reward);
	// else penalty_idx == 0, transition succeeds with no collision, no basic penalty added
	angle_diff = Dvc_CalAngleDiff(wheelchairpomdp_status, goal_point_idx);
	// Do the reaching check after transition
	int num_reached_goals = Dvc_ReachingCheck(wheelchairpomdp_state);
	// Reach the final goal
	if (num_reached_goals == intermediate_goal_list[wheelchairpomdp_state.path_idx].size_)
	{
		reward += Dvc_ModelParams::reaching_reward;
		return true;	// Only terminate the task when a final goal is reached
	}
	// Reach the intermediate goals or not reach any goal
	else
	{
		reward += Dvc_ModelParams::inter_goal_reward * num_reached_goals;	// Reward a +10 for each intermediate goal reached
	}
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
	int joystick_obs;

	angle_diff = angle_diff >= 0 ? angle_diff : angle_diff + 2 * M_PI;
	int direction_index = round(angle_diff * 8 / M_PI);
	direction_index = direction_index == 16 ? 0 : direction_index;
	// intermediate goal is in the right direction
	if (angle_diff <= -0.375 * M_PI)
	{
		if (rand_num < 0.95)
		{
			joystick_obs = OBS_RIGHT;
		}
		else
		{
			joystick_obs = OBS_FR_R;
		}
	}
	// intermediate goal is in the front right direction
	else if (angle_diff > -0.375 * M_PI && angle_diff <= -0.125 * M_PI)
	{
		if (rand_num < 0.9)
		{
			joystick_obs = OBS_FR_R;
		}
		else if (rand_num < 0.95)
		{
			joystick_obs = OBS_FRONT;
		}
		else
		{
			joystick_obs = OBS_RIGHT;
		}
	}
	// intermediate goal is in the front direction
	else if (angle_diff > -0.125 * M_PI && angle_diff < 0.125 * M_PI)
	{
		if (rand_num < 0.9)
		{
			joystick_obs = OBS_FRONT;
		}
		else if (rand_num < 0.95)
		{
			joystick_obs = OBS_FR_L;
		}
		else
		{
			joystick_obs = OBS_FR_R;
		}		
	}	
	// intermediate goal is in the front left direction
	else if (angle_diff >= 0.125 * M_PI && angle_diff < 0.375 * M_PI)
	{
		if (rand_num < 0.9)
		{
			joystick_obs = OBS_FR_L;
		}
		else if (rand_num < 0.95)
		{
			joystick_obs = OBS_LEFT;
		}
		else
		{
			joystick_obs = OBS_FRONT;
		}		
	}		
	// intermediate goal is in the left direction
	else		// angle_diff >= 0.375 * M_PI
	{
		if (rand_num < 0.95)
		{
			joystick_obs = OBS_LEFT;
		}
		else
		{
			joystick_obs = OBS_FR_L;
		}
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
	if (angle_diff <= -0.375 * M_PI)
	{
		if (wheelchair_obs.joystick_obs == OBS_RIGHT)
		{
			return 15;
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
	// goal door is in the front right direction
	else if (angle_diff > -0.375 * M_PI && angle_diff <= -0.125 * M_PI)
	{
		if (wheelchair_obs.joystick_obs == OBS_FR_R)
		{
			return 10;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FRONT)
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
	// goal door is in the front direction
	else if (angle_diff > -0.125 * M_PI && angle_diff < 0.125 * M_PI)
	{
		if (wheelchair_obs.joystick_obs == OBS_FRONT)
		{
			return 10;
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
	// goal door is in the front left direction
	else if (angle_diff >= 0.125 * M_PI && angle_diff < 0.375 * M_PI)
	{
		if (wheelchair_obs.joystick_obs == OBS_FR_L)
		{
			return 10;
		}
		else if (wheelchair_obs.joystick_obs == OBS_LEFT)
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
	// goal door is in the left direction
	else		// angle_diff >= 0.375 * M_PI
	{
		if (wheelchair_obs.joystick_obs == OBS_LEFT)
		{
			return 15;
		}
		else if (wheelchair_obs.joystick_obs == OBS_FR_L)
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

DEVICE int Dvc_WheelchairPomdp::Dvc_ReachingCheck(const Dvc_PomdpState& wheelchair_state) 
{
	float dist2goal = 0;
	int n = 0;
	float agent_yaw = wheelchair_state.wheelchair.angle_z;
	double goal_yaw = 0;
	//dist2goal = sqrtf(powf((wheelchair_state.wheelchair.pos_x - wheelchair_state.goal.pos_x), 2) 
	//	+ powf((wheelchair_state.wheelchair.pos_y - wheelchair_state.goal.pos_y), 2));
	for(int i = intermediate_goal_list[wheelchair_state.path_idx].size_ - 1; i>=0; --i )
	{
		dist2goal = hypotf(wheelchair_state.wheelchair.pos_x - intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].x, 
							wheelchair_state.wheelchair.pos_y - intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].y);
		goal_yaw = intermediate_goal_list[wheelchair_state.path_idx].way_points_[i].z ;

		if (dist2goal <= 0.25 && fabs(goal_yaw - agent_yaw) <= M_PI / 12)
		{
			// check which intermediate goal can be reached
			n = i + 1;
			// wheelchair_state.state_goal_point = intermediate_goal_list.paths[wheelchair_state.path_idx].poses[i].pose.position;
			break;
		}

	}
	// wheelchair_state.state_goal_index = n;
	// n = 0, then no reaching, n = size of path, then reach the final goal
	// reward the agent based on the position along the path that it could reach
	// if it is the last point along the path, then the agent reaches the final goal
	return n;
		
}

DEVICE float Dvc_WheelchairPomdp::Dvc_TransitWheelchair(Dvc_WheelchairStruct& wheelchair_status) 
{
	float yaw = wheelchair_status.angle_z;
	float step_time = 1/freq; //Updated from Model_params::control_freq in Init_GPU.cu
	int loop_times = roundf(transition_time / step_time);
	float penalty_idx = 0;
	float accumulative_penalty = 0;
	for (int i = 0; i < loop_times; i ++)
	{
		penalty_idx = Dvc_CollisionCheck(wheelchair_status);
		if (penalty_idx == 1)
		{
			return penalty_idx;	// A severe collision happens, transition fails, the task is terminated.
		}
		if (wheelchair_status.vel_w == 0)
		{
			wheelchair_status.pos_x += wheelchair_status.vel_v * cosf(yaw) * step_time;
			wheelchair_status.pos_y += wheelchair_status.vel_v * sinf(yaw) * step_time;
		}
		else
		{
			wheelchair_status.pos_x += wheelchair_status.vel_v / wheelchair_status.vel_w * 
				(sinf(yaw + wheelchair_status.vel_w * step_time) - sinf(yaw));
			wheelchair_status.pos_y += wheelchair_status.vel_v / wheelchair_status.vel_w * 
				(cosf(yaw) - cosf(yaw + wheelchair_status.vel_w * step_time));
			yaw += wheelchair_status.vel_w * step_time;
		}
		accumulative_penalty += penalty_idx;
	}
	penalty_idx = accumulative_penalty / (1.0*loop_times);
	
	//Update yaw
	wheelchair_status.angle_z = yaw;

	return penalty_idx;
}

DEVICE float Dvc_WheelchairPomdp::Dvc_CalAngleDiff(const Dvc_WheelchairStruct& wheelchair_status, int &goal_idx)
{
	float angle_difference;
	Eigen::Vector3f agent_heading(1,0,0);
	Eigen::Vector3f agent2goal(0,0,0);
	Eigen::Matrix3f m_agent_heading;
	m_agent_heading = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
  	* Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
  	* Eigen::AngleAxisf(wheelchair_status.angle_z, Eigen::Vector3f::UnitZ());
	Eigen::Quaternionf q_heading(m_agent_heading);
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


DEVICE float Dvc_WheelchairPomdp::Dvc_CollisionCheck(const Dvc_WheelchairStruct& wheelchair_status) 
{
	float x_check = wheelchair_status.pos_x;
	float y_check = wheelchair_status.pos_y;
	float r_collision = 0.42;
	float r_outer = 0.45;
	float temp_dist = 0;
	float penalty_idx = 0;

	//for (const auto &point : lidar_points)
	for(int i = 0; i < Dvc_ModelParams::LIDAR_POINTS_SIZE; i++)
	{

		const Dvc_3DCOORD& point = lidar_points[i];
		//https://stackoverflow.com/a/7227057
		//Check through easy conditions first, getting distance with sqrt is computationally expensive
		double dx = fabs(point.x - x_check);
		double dy = fabs(point.y - y_check);

		if (dx + dy <= r_collision)
		{
			penalty_idx = 1;
			return penalty_idx;
		}
		else
		{
			temp_dist = sqrtf(powf(dx, 2) + powf(dy, 2));
			if (temp_dist <= r_collision)
			{
				penalty_idx = 1;
				return penalty_idx;
			}
			else if (temp_dist <= r_outer)
			{
				penalty_idx = 1 - (temp_dist - r_collision) / (r_outer - r_collision);
			}
		}
	}
	return penalty_idx;
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
	Dvc_WheelchairStruct& wheelchairpomdp_status = wheelchairpomdp_state->wheelchair;
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

