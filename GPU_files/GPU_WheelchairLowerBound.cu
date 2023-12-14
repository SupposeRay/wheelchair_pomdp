#include "GPU_WheelchairLowerBound.h"

#include <despot/GPUcore/GPUhistory.h>
#include <despot/GPUrandom_streams.h>
#include <despot/GPUutil/GPUcoord.h>
#include "GPU_wheelchair_model.h"
using archaeopteryx::util::numeric_limits;
using despot::Dvc_History;
using despot::Dvc_RandomStreams;
using despot::Dvc_State;

#include <despot/GPUinterface/GPUpomdp.h>

DEVICE int Dvc_PedPomdpDoNothingPolicy::Action(
		int scenarioID, const Dvc_State* particles,
				Dvc_RandomStreams& streams,
				Dvc_History& history)
{

	/*int size = history.Size();
	int num_acc = 0;
	int num_dec = 0;
	for(int i = 0; i < size; i++)
	{
		ACT_TYPE action = history.Action(i);
		if(action == 1) num_acc++;
		if(action == 2) num_dec++;

	}

	if(num_acc > num_dec)
	{
		return 2;
	}
	else
	{
		if(num_acc == num_dec)
		{
			if(num_acc == 0)
			{
				return 0;
			}
			else
			{
				return 2; //Try to decelerate once more to account for decelaration failure
			}
		}
		else
		{
			return 0;
		}
	}*/
	//return 2;
	//For despot with alpha vector update action cannot depend on observation
	//const Dvc_PomdpState &state=static_cast<const Dvc_PomdpState&>(particles[0]);
	float carvel = history.carvel;
	/*if (round(linear_v * 100) / 100 > 0)
		return LINEAR_MINUS;
	else if (round(linear_v * 100) / 100 < 0)
		return LINEAR_PLUS;
	else
		return -1;
	*/
	
	if (round(carvel * 100) / 100 > 0)
		{
		history.carvel = carvel  - (Dvc_ModelParams::AccSpeed/freq); //Every particle gets its own local history
		if (blockIdx.x == 0 && threadIdx.x == 0 && threadIdx.y == 0)
			// printf("Carvel is %f action retuened is 1\n", carvel);
		return 1; //Action hard coded for WheelchairDSPOMDP::LINEAR_MINUS
		}
	
	if (round(carvel * 100) / 100 < 0){
		history.carvel = carvel  + (Dvc_ModelParams::AccSpeed/freq); //Every particle gets its own local history
		if (blockIdx.x == 0 && threadIdx.x == 0 && threadIdx.y == 0)
			// printf("Carvel is %f action retuened is 0\n", carvel);
		return 0; //Action hard coded for WheelchairDSPOMDP::LINEAR_PLUS
	}
	if (blockIdx.x == 0 && threadIdx.x == 0 && threadIdx.y == 0)
		// printf("Carvel is %f action retuened is -1\n", carvel);
	return -1;
}



// DEVICE int Dvc_PedPomdpSmartPolicy::Action(
// 		int scenarioID, const Dvc_State* particles,
// 				Dvc_RandomStreams& streams,
// 				Dvc_History& history)
// {

// 	const Dvc_PomdpState &state=static_cast<const Dvc_PomdpState&>(particles[0]);
// 	__shared__ int mindist[32];
// 	auto& carpos = path->way_points_[state.car.pos];


// 	float carvel = state.car.vel;

// 	mindist[threadIdx.x]=__float_as_int(numeric_limits<float>::infinity());
// 	__syncthreads();
// 	if (threadIdx.y<state.num) {
// 		auto& p = state.peds[threadIdx.y];
// 		bool infront=false;

// 		if(Dvc_ModelParams::IN_FRONT_ANGLE_DEG >= 180.0) {
// 			// inFront check is disabled in this case
// 			infront=true;
// 		}
// 		else
// 		{
// 			const Dvc_COORD& car_pos = path->way_points_[state.car.pos];
// 			const Dvc_COORD& forward_pos = path->way_points_[path->forward(state.car.pos, 1.0)];
			


// 			float d0 = Dvc_COORD::EuclideanDistance(car_pos, p.pos);


// 			if(d0 <= /*0.7*/3.5)
// 				infront=true;
// 			else
// 			{
// 				float d1 = Dvc_COORD::EuclideanDistance(car_pos, forward_pos);
// 				if(d1<=0)
// 					infront=true;
// 				else
// 				{
// 					float dot = Dvc_Vector::DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
// 							p.pos.x - car_pos.x, p.pos.y - car_pos.y);
// 					float cosa = dot / (d0 * d1);
// 					if(cosa > 1.0 + 1E-6 || cosa < -1.0 - 1E-6)
// 					{
// 						;
					
// 					}
// 					infront=cosa > in_front_angle_cos;
// 				}
// 			}
// 		}



// 		if(infront) {
// 			float d = Dvc_COORD::EuclideanDistance(carpos, p.pos);
// 			atomicMin(mindist+threadIdx.x, __float_as_int(d));
// 		}
// 	}
// 	__syncthreads();

// 	// TODO set as a param
// 	if (__int_as_float(mindist[threadIdx.x]) < /*2*/3.5) {
// 		return (carvel <= 0.01) ? 0 : 2;
// 	}

// 	if (__int_as_float(mindist[threadIdx.x]) < /*4*/5) {
// 		if (carvel > 1.0+1e-4) return 2;
// 		else if (carvel < 0.5-1e-4) return 1;
// 		else return 0;
// 	}


// 	return carvel >= Dvc_ModelParams::VEL_MAX-1e-4 ? 0 : 1;
// 	return 0;
// }
