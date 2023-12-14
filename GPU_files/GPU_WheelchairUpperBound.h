/*
 * DvcPedPomdpSmartScenarioLowerBoundPolicy.h
 *
 *  Created on: 14 Sep, 2017
 *      Author: panpan
 */

#ifndef DVCWHEELCHAIRPOMDPUPPERBOUND_H_
#define DVCWHEELCHAIRPOMDPUPPERBOUND_H_
#include <despot/GPUcore/GPUhistory.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUinterface/GPUupper_bound.h>

using despot::Dvc_History;
using despot::Dvc_State;
using despot::Dvc_ScenarioUpperBound;
using namespace despot;

//Not changing name of the upper bound to avoid changing name everywhere
class Dvc_PedPomdpParticleUpperBound1: public Dvc_ScenarioUpperBound{ //Implemented in Gpu_wheelchair_model.cu
public:

	DEVICE static float Value(const Dvc_State* particles, int scenarioID, Dvc_History& history);

};
#endif /* DVCPEDPOMDPSMARTSCENARIOLOWERBOUNDPOLICY_H_ */

