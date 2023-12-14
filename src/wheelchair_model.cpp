#include "wheelchair_pomdp/wheelchair_model.h"


#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot
{

// action space
const ACT_TYPE WheelchairDSPOMDP::LINEAR_PLUS = 0;
const ACT_TYPE WheelchairDSPOMDP::LINEAR_MINUS = 1;
const ACT_TYPE WheelchairDSPOMDP::ANGULAR_PLUS = 2;
const ACT_TYPE WheelchairDSPOMDP::ANGULAR_MINUS = 3;
const ACT_TYPE WheelchairDSPOMDP::KEEP = 4;

/* =============================================================================
 * WheelchairBelief class
 * =============================================================================*/

WheelchairParticleBelief::WheelchairParticleBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior, bool split)
	:ParticleBelief(particles, model, prior, split),
	wheelchair_model_(static_cast<const WheelchairDSPOMDP*>(model))
{}

void WheelchairParticleBelief::Update(ACT_TYPE action, OBS_TYPE obs)
{
	history_.Add(action, obs);

	// Use HashMap to get observation
    std::map<uint64_t, WheelchairObs>::iterator it = wheelchair_model_->obsHashMap_execution.find(obs);

    if (it == wheelchair_model_->obsHashMap_execution.end())
    {
      it = wheelchair_model_->obsHashMap.find(obs);
      if (it == wheelchair_model_->obsHashMap.end())
      {
        std::cout << "Observation not in hash map and execution hash map. This should not happen." << std::endl;
        // std::cout << "obs hash value " << obs << std::endl;
        // std::map<uint64_t, WheelchairObs>::iterator iter = obsHashMap.begin();
        // while(iter != obsHashMap.end())
        // {
        // 	std::cout << "obsHashMap value " << iter->first << std::endl;
        // 	iter++;
        // }
      }
    //   std::cout << "Observation not in execution hash map. This should not happen." << std::endl;
	}
    WheelchairObs wheelchair_obs = it->second;

	wheelchair_obs.joystick_signal.x = round(wheelchair_obs.joystick_signal.x * 100) / 100;
	wheelchair_obs.joystick_signal.y = round(wheelchair_obs.joystick_signal.y * 100) / 100;

	vector<State*> updated;
	/*	Use goal positions	*/

	// Initialize previous goal positions if pre_positions is empty
	if (wheelchair_model_->pre_positions.size() == 0)
	{
		cout << "Belief Space Initializing." << endl;
		wheelchair_model_->pre_positions = wheelchair_model_->goal_positions;
		// check if the max number of paths have been found
		if (particles_.size() > wheelchair_model_->goal_positions.size())
		{
			// remove extra particles
			for (int i = 0; i < particles_.size(); ++i)
			{
				State* particle = particles_[i];
				WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
				if (i < wheelchair_model_->goal_positions.size())
				{
					wheelchair_particle->weight = static_cast<float> (1 / wheelchair_model_->goal_positions.size());
					wheelchair_particle->path_idx = i;
					wheelchair_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
					wheelchair_particle->wheelchair.agent_pose.position.x = 0;
					wheelchair_particle->wheelchair.agent_pose.position.y = 0;
					wheelchair_particle->wheelchair.agent_pose.orientation.x = 0;
					wheelchair_particle->wheelchair.agent_pose.orientation.y = 0;
					wheelchair_particle->wheelchair.agent_pose.orientation.z = 0;
					wheelchair_particle->wheelchair.agent_pose.orientation.w = 1;
					updated.push_back(wheelchair_particle);
					// wheelchair_model_->belief_goal.push_back(wheelchair_particle->weight);
				}
				else
				{
					wheelchair_model_->Free(wheelchair_particle);
				}
			}
			particles_ = updated;
		}
		else
		{
			for (int i = 0; i < particles_.size(); ++i)
			{
				State* particle = particles_[i];
				WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
				wheelchair_particle->path_idx = i;
				wheelchair_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
				wheelchair_particle->wheelchair.agent_pose.position.x = 0;
				wheelchair_particle->wheelchair.agent_pose.position.y = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.x = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.y = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.z = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.w = 1;
			}
		}
	}

	// New paths are appended
	if (wheelchair_model_->goal_positions.size() != wheelchair_model_->pre_positions.size())
	{
		cout << "New paths are appended." << endl;
		int num_changed_goal = 0;
		int num_added_goal = 0;
		float sum_changed_goal = 0;
		// add new particles to current particles_
		for (int i = wheelchair_model_->pre_positions.size(); i < wheelchair_model_->goal_positions.size(); ++i)
		{
			WheelchairState* new_particle = static_cast<WheelchairState*>(wheelchair_model_->Allocate(i, 1.0 / wheelchair_model_->goal_positions.size()));
			new_particle->path_idx = i;
			new_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
			new_particle->wheelchair.agent_pose.position.x = 0;
			new_particle->wheelchair.agent_pose.position.y = 0;
			new_particle->wheelchair.agent_pose.orientation.x = 0;
			new_particle->wheelchair.agent_pose.orientation.y = 0;
			new_particle->wheelchair.agent_pose.orientation.z = 0;
			new_particle->wheelchair.agent_pose.orientation.w = 1;
			new_particle->weight = 0;	// assign zero probabilities to the appended new paths
			particles_.push_back(new_particle);
			num_added_goal++;  // count the number of added paths
		}
		// wheelchair_model_->belief_goal.resize(wheelchair_model_->goal_positions.size(), 0);
		// check if previous registered paths have changed
		for (int i = 0; i < wheelchair_model_->pre_positions.size(); i++)
		{
			if (wheelchair_model_->pre_positions[i].header.seq == wheelchair_model_->goal_positions[i].header.seq) // not changed
			{
				continue;
			}
			else // paths have been replaced
			{
				num_changed_goal++; // count the number of replaced paths
				sum_changed_goal += particles_[i]->weight; // sum of the probabilities over replaced paths
				// wheelchair_model_->belief_goal[i] = 0; // assign zero probabilities to the replaced paths
				particles_[i]->weight = 0;
			}
		}
		cout << num_added_goal << " path(s) have been added, " << num_changed_goal << " path(s) have been changed." << endl;
		// distribute the probabilites from replaced paths to the remaining paths evenly
		for (int i = 0; i < wheelchair_model_->pre_positions.size(); i++)
		{
			if (wheelchair_model_->pre_positions[i].header.seq == wheelchair_model_->goal_positions[i].header.seq)
			{
				particles_[i]->weight += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
				// wheelchair_model_->belief_goal[i] += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
			}
		}
		wheelchair_model_->clipBelief(particles_);
		wheelchair_model_->normalize(particles_);

		// store the current path info for the next checking process
		wheelchair_model_->pre_positions = wheelchair_model_->goal_positions;
	}
	else // no new paths added, only replacement
	{
		cout << "No paths added." << endl;
		int num_changed_goal = 0;
		float sum_changed_goal = 0;
		bool path_replaced = false;
		for (int i = 0; i < wheelchair_model_->goal_positions.size(); i++)
		{
			// check if previous registered paths have changed
			if (wheelchair_model_->pre_positions[i].header.seq == wheelchair_model_->goal_positions[i].header.seq)
			{
				continue;
			}
			num_changed_goal++;
			sum_changed_goal += particles_[i]->weight;;
			// wheelchair_model_->belief_goal[i] = 0;
			particles_[i]->weight = 0;
			path_replaced = true;
		}
		if (path_replaced)
		{
			cout << num_changed_goal << " path(s) have been changed." << endl;
			for (int i = 0; i < wheelchair_model_->goal_positions.size(); i++)
			{
				if (wheelchair_model_->pre_positions[i].header.seq == wheelchair_model_->goal_positions[i].header.seq)
				{
					// wheelchair_model_->belief_goal[i] += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
					particles_[i]->weight += sum_changed_goal / (wheelchair_model_->pre_positions.size() - num_changed_goal);
				}
			}
			wheelchair_model_->clipBelief(particles_);
			wheelchair_model_->normalize(particles_);
		}
		wheelchair_model_->pre_positions = wheelchair_model_->goal_positions;
	}

	updated.clear();
	double total_weight = 0;
	double reward;
	OBS_TYPE o;
	float Q_rotation = 0, Pi_rotation = 0;
	// cout << "Obs in particlebelief update " << obs << endl;
	cout << "Joystick input: x = " <<  wheelchair_obs.joystick_signal.x << ", y = " << wheelchair_obs.joystick_signal.y << endl;
	// Update particles
	for (int i = 0; i < particles_.size(); i++)
	{
		State* particle = particles_[i];
		////////Resyncing particles in local frame
		WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
		wheelchair_particle->wheelchair.agent_pose.position.x = 0;
		wheelchair_particle->wheelchair.agent_pose.position.y = 0;
		wheelchair_particle->wheelchair.agent_pose.orientation.x = 0;
		wheelchair_particle->wheelchair.agent_pose.orientation.y = 0;
		wheelchair_particle->wheelchair.agent_pose.orientation.z = 0;
		wheelchair_particle->wheelchair.agent_pose.orientation.w = 1;
		//Store yaw for use later
	
	
		tf2::Quaternion agent_quat;
		tf2::convert(wheelchair_particle->wheelchair.agent_pose.orientation, agent_quat);
		wheelchair_particle->wheelchair.agent_pose_angle = tf2::getYaw(agent_quat);

		wheelchair_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
		wheelchair_particle->joystick_x = wheelchair_obs.joystick_signal.x;
		wheelchair_particle->joystick_y = wheelchair_obs.joystick_signal.y;
		// wheelchair_particle->weight = wheelchair_model_->belief_goal[wheelchair_particle->path_idx];
		// Resyncing syncs the particles with the current state, which is required to calculate the step and belief.
		bool terminal = wheelchair_model_->Step(*wheelchair_particle, Random::RANDOM.NextDouble(), action, reward, o);
		// only update belief when joystick input is not zero
		if (wheelchair_obs.joystick_signal.x != 0 || wheelchair_obs.joystick_signal.y != 0)
		{
			// cout << "Updating belief of path " << i + 1 << "..." << endl;
			Q_rotation = wheelchair_model_->calRotationValue(wheelchair_obs.joystick_signal.x, wheelchair_obs.joystick_signal.y,
				wheelchair_model_->goal_positions[wheelchair_particle->path_idx].point);
			// cout << "Q_rotation " << Q_rotation << endl;
			Pi_rotation =  exp(- ModelParams::exp_temperature * Q_rotation);	// MaxEnt IOC
			// cout << "Pi_rotation " << Pi_rotation << endl;
			
			// DO NOT call ObsProb in Update function
			// double prob = wheelchair_model_->ObsProb(obs, *wheelchair_particle, action);
			// Step all the particles with the latest action to take into account the lag, change in the wheelchair position and velocity.
			//Step function only updates the agent_pose_angle (which is yaw) also
			if (!terminal)
			{ // Terminal state is not required to be explicitly represented and may not have any observation
				particle->weight *= Pi_rotation;
				// cout << "Prob " << i << ": " << prob << endl;
				total_weight += particle->weight;
				updated.push_back(particle);
			}
			else
			{
				// cout << "Particle " << i << ", Terminal: " << terminal << ", Prob: " << prob << endl;
				wheelchair_model_->Free(particle);
			}
		}
	}
	if (wheelchair_obs.joystick_signal.x != 0 || wheelchair_obs.joystick_signal.y != 0)
	{
		logd << "[ParticleBelief::Update] " << updated.size()
			<< " particles survived among " << particles_.size() << endl;
		particles_ = updated;

		// Resample if the particle set is empty
		if (particles_.size() == 0)
		{
			logw << "Particle set is empty!" << endl;
			if (prior_ != NULL)
			{
				logw
					<< "Resampling by drawing random particles from prior which are consistent with history"
					<< endl;
				particles_ = Resample(num_particles_, *prior_, history_);
			}
			else
			{
				logw
					<< "Resampling by searching initial particles which are consistent with history"
					<< endl;
				particles_ = Resample(num_particles_, initial_particles_, model_,
					history_);
			}

			if (particles_.size() == 0 && state_indexer_ != NULL)
			{
				logw
					<< "Resampling by searching states consistent with last (action, observation) pair"
					<< endl;
				particles_ = Resample(num_particles_, model_, state_indexer_,
					action, obs);
			}

			if (particles_.size() == 0)
			{
				logw << "Resampling failed - Using initial particles" << endl;
				for (int i = 0; i < initial_particles_.size(); i ++)
					particles_.push_back(model_->Copy(initial_particles_[i]));
			}

			//Update total weight so that effective number of particles are computed correctly
			total_weight = 0;
			for (int i = 0; i < particles_.size(); i++)
			{
				State* particle = particles_[i];
				total_weight = total_weight + particle->weight;
			}
		}
		cout << "Updating belief..." << endl;
		double weight_square_sum = 0;
		for (int i = 0; i < particles_.size(); i++)
		{
			State* particle = particles_[i];
			particle->weight /= total_weight;
			// weight_square_sum += particle->weight * particle->weight;
		}

		wheelchair_model_->clipBelief(particles_);
		wheelchair_model_->normalize(particles_);

		// Resample if the effective number of particles is "small"
		// double num_effective_particles = 1.0 / weight_square_sum;
		// if (num_effective_particles < num_particles_ / 2.0)
		// {
		// 	vector<State*> new_belief = Sample(num_particles_, particles_, model_);
		// 	for (int i = 0; i < particles_.size(); i++)
		// 		model_->Free(particles_[i]);

		// 	particles_ = new_belief;
		// }
	}
	cout << "Belief:" << endl;
	for (int i = 0; i < particles_.size(); ++i)
	{
		cout << "Goal " << i + 1 << ", position: x = " << wheelchair_model_->goal_positions[i].point.x << ", y = " << wheelchair_model_->goal_positions[i].point.y << ", belief: " << particles_[i]->weight << endl;
	}
}

/* =============================================================================
 * WheelchairState class
 * =============================================================================*/

WheelchairState::WheelchairState() 
{}

WheelchairState::WheelchairState(WheelchairStruct _wheelchair, int _path_idx, float _joystick_x, float _joystick_y):
wheelchair(_wheelchair),
path_idx(_path_idx),
joystick_x(_joystick_x),
joystick_y(_joystick_y)
{}

WheelchairState::~WheelchairState()
{}

string WheelchairState::text() const
{
	return "Wheelchair position: x = " + to_string(wheelchair.agent_pose.position.x) + " y = " + to_string(wheelchair.agent_pose.position.y)
	+ "\n Wheelchair orientation: x = " + to_string(wheelchair.agent_pose.orientation.x) + " y = " + to_string(wheelchair.agent_pose.orientation.x)
	+ " z = " + to_string(wheelchair.agent_pose.orientation.z) + " w = " + to_string(wheelchair.agent_pose.orientation.w)
	+ "\n Wheelchair velocity: v = " + to_string(wheelchair.agent_velocity.linear.x) + " w = " + to_string(wheelchair.agent_velocity.angular.z)
	+ "\n Joystick input: x = " + to_string(joystick_x) + " y = " + to_string(joystick_y)
	// + "\n Wheelchair goal index = " + to_string(state_goal_index)
	// + "\n Wheelchair goal x = " + to_string(state_goal_point.x) + " y = " + to_string(state_goal_point.y)
	+ "\n Preferred path index = " + to_string(path_idx);
}

/* =============================================================================
 * WheelchairDSPOMDP class
 * =============================================================================*/

WheelchairDSPOMDP::WheelchairDSPOMDP()
{
	// initialization
	current_wheelchair_status.agent_pose.position.x = 0;
	current_wheelchair_status.agent_pose.position.y = 0;
	current_wheelchair_status.agent_pose.position.z = 0;
	current_wheelchair_status.agent_pose.orientation.x = 0;
	current_wheelchair_status.agent_pose.orientation.y = 0;
	current_wheelchair_status.agent_pose.orientation.z = 0;
	current_wheelchair_status.agent_pose.orientation.w = 1;

	//Store yaw for use later
	
	
	tf2::Quaternion agent_quat;
	tf2::convert(current_wheelchair_status.agent_pose.orientation, agent_quat);
	current_wheelchair_status.agent_pose_angle = tf2::getYaw(agent_quat);
	
	current_wheelchair_status.agent_velocity.linear.x = 0;
	current_wheelchair_status.agent_velocity.linear.y = 0;
	current_wheelchair_status.agent_velocity.linear.z = 0;
	current_wheelchair_status.agent_velocity.angular.x = 0;
	current_wheelchair_status.agent_velocity.angular.y = 0;
	current_wheelchair_status.agent_velocity.angular.z = 0;
	goal_positions.reserve(ModelParams::num_paths);
	intermediate_goal_list.paths.reserve(ModelParams::num_paths);
	pre_positions.clear();
	dmp_init_variables = DMP_init_variables(ModelParams::num_paths);
}

WheelchairDSPOMDP::WheelchairDSPOMDP(std::string config_file)
{
	//TODO
	dmp_init_variables = DMP_init_variables(ModelParams::num_paths);
}

/* ======
 * Action
 * ======*/

int WheelchairDSPOMDP::NumActions() const
{
	return 5;
}

/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool WheelchairDSPOMDP::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const
{
    WheelchairState& wheelchair_state = static_cast < WheelchairState& >(state);
	WheelchairStruct& wheelchair_status = wheelchair_state.wheelchair;
	int& goal_point_idx = wheelchair_state.path_idx;

	//tf2::Quaternion agent_quat;
	//tf2::convert(wheelchair_status.agent_pose.orientation, agent_quat);

	double angle_diff;

	std::ostringstream obs_string;
	WheelchairObs wheelchair_obs;
	wheelchair_obs.continuous_obs = false;

	// if (action == ASK)
	// {
	// 	// wheelchair stops and asks the question
	// 	wheelchair_status.agent_velocity.linear.x = 0;
	// 	wheelchair_status.agent_velocity.angular.z = 0;
	// 	reward = -50;
	// 	// observation part
	// 	wheelchair_obs.getObsFromWheelchairStatus(wheelchair_status);
	// 	wheelchair_obs.joystick_obs = OBS_NONE;
	// 	PrintObs(wheelchair_obs, obs_string);
	// 	// takes printed string to obs_string and converts to a hash value
	// 	uint64_t hashValue = obsHash(obs_string.str());
	// 	// takes hashValue and updates value of obs in ObservationClass
	// 	wheelchair_obs.SetIntObs(hashValue);
	// 	obs = (OBS_TYPE) hashValue;

	// 	// Updates hashmap:
	// 	// takes hash value as key and stores wheelchair_obs as value in map
	// 	// First check if hash value is already stored
	// 	if (obsHashMap.find(hashValue) == obsHashMap.end())
	// 	{
	// 		obsHashMap[hashValue] = wheelchair_obs;
	// 	}
	// }
	// else
	// {
	float max_linear_speed = external_joy_x > 0 ? fabs(external_joy_x) : ModelParams::backing_ratio * fabs(external_joy_x);
	// float max_linear_speed = 1.0;
	wheelchair_status.agent_velocity.linear.x = round(wheelchair_status.agent_velocity.linear.x * 100) / 100;
	wheelchair_status.agent_velocity.angular.z = round(wheelchair_status.agent_velocity.angular.z * 100) / 100;

	if (action == LINEAR_PLUS)
	{
		if (wheelchair_status.agent_velocity.linear.x > max_linear_speed)
		{
			reward = ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchair_status.agent_velocity.linear.x < -0.1 && wheelchair_status.agent_velocity.linear.x > -0.2)
				wheelchair_status.agent_velocity.linear.x += (-wheelchair_status.agent_velocity.linear.x);
			else
				wheelchair_status.agent_velocity.linear.x += 0.2; // Increase the linear speed by 0.2
			reward = ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == LINEAR_MINUS)
	{
		if (wheelchair_status.agent_velocity.linear.x < -max_linear_speed)
		{
			reward = ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchair_status.agent_velocity.linear.x > 0.1 && wheelchair_status.agent_velocity.linear.x < 0.2)
				wheelchair_status.agent_velocity.linear.x += (-wheelchair_status.agent_velocity.linear.x);
			else
				wheelchair_status.agent_velocity.linear.x -= 0.2; // Decrease the linear speed by 0.2
			reward = ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == ANGULAR_PLUS)
	{
		if (wheelchair_status.agent_velocity.angular.z > ModelParams::max_angular_speed)
		{
			reward = ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchair_status.agent_velocity.angular.z < -0.1 && wheelchair_status.agent_velocity.angular.z > -0.5)
				wheelchair_status.agent_velocity.angular.z += (-wheelchair_status.agent_velocity.angular.z);
			else
				wheelchair_status.agent_velocity.angular.z += 0.5; // Increase the angular speed by 0.2
			reward = ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == ANGULAR_MINUS)
	{
		if (wheelchair_status.agent_velocity.angular.z < -ModelParams::max_angular_speed)
		{
			reward = ModelParams::excess_speed_penalty;
		}
		else
		{
			if (wheelchair_status.agent_velocity.angular.z > 0.1 && wheelchair_status.agent_velocity.angular.z < 0.5)
				wheelchair_status.agent_velocity.angular.z += (-wheelchair_status.agent_velocity.angular.z);
			else
				wheelchair_status.agent_velocity.angular.z -= 0.5; // Decrease the angular speed by 0.2
			reward = ModelParams::step_penalty; // Penalize the step
		}
	}
	else if (action == KEEP)
	{
		reward = ModelParams::step_penalty; // Penalize the step
	}

	/* THE SHARED CONTROL PART */
	reward += SharedControlReward(wheelchair_state, rand_num);
	// add white noise to the current velocity
	// according to the log file, the velocity diff can be as high as 0.04, so the tolerance will be ±0.04
	if (ModelParams::using_probabilistic_model)
	{
		wheelchair_status.agent_velocity.linear.x += (rand_num - 0.5) * 2 * ModelParams::noise_amplitude;
		wheelchair_status.agent_velocity.angular.z += (rand_num - 0.5) * 2 * ModelParams::noise_amplitude;
	}
	
	// cout << "Reward after applying action: " << reward << endl;
	// Transit to the next state with a penalty_idx [0, 1] as a return value
	float penalty_idx = TransitWheelchair(wheelchair_state);

	if (penalty_idx == 1) // A severe collision happens
	{
		reward += ModelParams::collision_penalty;
		return true;	// Only terminate the task when a severe collision occurs
	}
	else if (penalty_idx != 0)	// Transition succeeds with mild collision
	{
		// -100 basic penalty for mild collision plus an extra penalty based on the distance to obstacles
		reward += (ModelParams::collision_penalty - ModelParams::inflation_basic_penalty) * penalty_idx + ModelParams::inflation_basic_penalty;
	}
	// else penalty_idx == 0, transition succeeds with no collision, no basic penalty added

	// cout << "Reward after collision check: " << reward << endl;

	angle_diff = CalAngleDiff(wheelchair_status, goal_point_idx);
	// Do the reaching check after transition
	int n = ReachingCheck(wheelchair_state);
	// Reach the final goal
	if (n == intermediate_goal_list.paths[wheelchair_state.path_idx].poses.size())
	{
		reward += ModelParams::reaching_reward;
		return true;	// Only terminate the task when a final goal is reached
	}
	// Reach the intermediate goals or not reach any goal
	else
	{
		reward += ModelParams::inter_goal_reward * n;	// Reward a +10 for each intermediate goal reached
	}
	// cout << "Reward after reaching check: " << reward << endl;
	// Receding penalty

	// reward += RecedingPenalty(wheelchair_state);

	// cout << "Reward after receding penalty: " << reward << endl;

	// cout << "Reward after shared control: " << reward << endl;
	
	// Observation part
	wheelchair_obs.getObsFromWheelchairStatus(wheelchair_status);
	
	// // intermediate goal is in the right direction
	// if (angle_diff <= -0.375 * M_PI)
	// {
	// 	if (rand_num < 0.95)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_RIGHT;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_R;
	// 	}
	// }
	// // intermediate goal is in the front right direction
	// else if (angle_diff > -0.375 * M_PI && angle_diff <= -0.125 * M_PI)
	// {
	// 	if (rand_num < 0.9)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_R;
	// 	}
	// 	else if (rand_num < 0.95)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FRONT;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_RIGHT;
	// 	}
	// }
	// // intermediate goal is in the front direction
	// else if (angle_diff > -0.125 * M_PI && angle_diff < 0.125 * M_PI)
	// {
	// 	if (rand_num < 0.9)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FRONT;
	// 	}
	// 	else if (rand_num < 0.95)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_L;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_R;
	// 	}		
	// }
	// // intermediate goal is in the front left direction
	// else if (angle_diff >= 0.125 * M_PI && angle_diff < 0.375 * M_PI)
	// {
	// 	if (rand_num < 0.9)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_L;
	// 	}
	// 	else if (rand_num < 0.95)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_LEFT;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FRONT;
	// 	}		
	// }
	// // intermediate goal is in the left direction
	// else		// angle_diff >= 0.375 * M_PI
	// {
	// 	if (rand_num < 0.95)
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_LEFT;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_obs.joystick_obs = OBS_FR_L;
	// 	}
	// }
	angle_diff = angle_diff >= 0 ? angle_diff : angle_diff + 2 * M_PI;

	int direction_index = round(angle_diff * 8 / M_PI);

	direction_index = direction_index == 16 ? 0 : direction_index;

	switch (direction_index)
	{
		case 0:	// intermediate goal is in the front direction (-pi/16 ~ pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_FRONT;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_FR_L;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_FR_R;
			}
			break;
		}
		case 1:	// intermediate goal is in the front left direction (pi/16 ~ 3pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_FR_L;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_FRONT;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_F_L;
			}
			break;
		}
		case 2: // intermediate goal is in the front left direction (3pi/16 ~ 5pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_F_L;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_FR_L;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_F_LE;
			}
			break;
		}
		case 3:	// intermediate goal is in the front left direction (5pi/16 ~ 7pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_F_LE;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_F_L;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_LEFT;
			}
			break;
		}
		case 4:	// intermediate goal is in the left direction (7pi/16 ~ 9pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_LEFT;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_F_LE;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_B_LE;
			}
			break;
		}
		case 5:	// intermediate goal is in the back left direction (9pi/16 ~ 11pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_B_LE;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_LEFT;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_B_L;
			}
			break;
		}
		case 6:	// intermediate goal is in the back left direction (11pi/16 ~ 13pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_B_L;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_B_LE;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_BA_L;
			}
			break;
		}
		case 7:	// intermediate goal is in the back left direction (13pi/16 ~ 15pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_BA_L;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_B_L;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_BACK;
			}
			break;
		}
		case 8:	// intermediate goal is in the back direction (15pi/16 ~ pi, -pi ~ -15pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_BACK;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_BA_L;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_BA_R;
			}
			break;
		}
		case 9:	// intermediate goal is in the back right direction (-15pi/16 ~ -13pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_BA_R;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_BACK;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_B_R;
			}
			break;
		}
		case 10:	// intermediate goal is in the back right direction (-13pi/16 ~ -11pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_B_R;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_BA_R;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_B_RI;
			}
			break;
		}
		case 11:	// intermediate goal is in the back right direction (-11pi/16 ~ -9pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_B_RI;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_B_R;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_RIGHT;
			}
			break;
		}
		case 12:	// intermediate goal is in the right direction (-9pi/16 ~ -7pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_RIGHT;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_B_RI;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_F_RI;
			}
			break;
		}
		case 13:	// intermediate goal is in the front right direction (-7pi/16 ~ -5pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_F_RI;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_RIGHT;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_F_R;
			}
			break;
		}
		case 14:	// intermediate goal is in the front right direction (-5pi/16 ~ -3pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_F_R;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_F_RI;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_FR_R;
			}
			break;
		}
		case 15:	// intermediate goal is in the front right direction (-3pi/16 ~ -pi/16)
		{
			if (rand_num < 0.7)
			{
				wheelchair_obs.joystick_obs = OBS_FR_R;
			}
			else if (rand_num < 0.85)
			{
				wheelchair_obs.joystick_obs = OBS_F_R;
			}
			else
			{
				wheelchair_obs.joystick_obs = OBS_FRONT;
			}
			break;
		}
	}
	// Updates hashmap:
	// takes hash value as key and stores wheelchair_obs as value in map
	// First check if hash value is already stored
	PrintObs(wheelchair_obs, obs_string);
	// takes printed string to obs_string and converts to a hash value
	uint64_t hashValue = obsHash(obs_string.str());
	// takes hashValue and updates value of obs in ObservationClass
	wheelchair_obs.SetIntObs(hashValue);
	obs = (OBS_TYPE) hashValue;
	// if (obsHashMap.find(hashValue) == obsHashMap.end())
	// {
	// 	obsHashMap[hashValue] = wheelchair_obs;
	// }
	
	return false;
}

/* ================================================
 * Functions related to beliefs and starting states
 * ================================================*/

// Probability of observation given current state resulting 
// from executing action in previous state
// P(O | s, a)

double WheelchairDSPOMDP::ObsProb(OBS_TYPE obs, const State& state,	ACT_TYPE action) const
{	
	const WheelchairState& wheelchair_state = static_cast <const WheelchairState&>(state);
	WheelchairStruct wheelchair_status = wheelchair_state.wheelchair;
	int goal_point_idx = wheelchair_state.path_idx;
	//tf2::Quaternion agent_quat;
	//tf2::convert(wheelchair_status.agent_pose.orientation, agent_quat);

	// Use HashMap to get observation

	std::map<uint64_t, WheelchairObs>::iterator it = obsHashMap_execution.find(obs);

	if (it == obsHashMap_execution.end())
	{
		it = obsHashMap.find(obs);
		if (it == obsHashMap.end())
		{
			std::cout << "Calling ObsProb." << std::endl;
			std::cout << "Observation not in hash map and execution hash map. This should not happen." << std::endl;
			// std::cout << "obs hash value " << obs << std::endl;
			// std::map<uint64_t, WheelchairObs>::iterator iter = obsHashMap.begin();
			// while(iter != obsHashMap.end())
			// {
			// 	std::cout << "obsHashMap value " << iter->first << std::endl;
			// 	iter++;
			// }
		}
		// std::cout << "Observation not in execution hash map. This should not happen." << std::endl;
	}	
	
	WheelchairObs wheelchair_obs = it->second;

	// if (action == ASK)
	// {
	// 	// ROS_INFO_STREAM("Last action: Asking question...");
	// 	// ROS_INFO_STREAM("Last obs: " << wheelchair_obs.joystick_obs);
	// 	if (wheelchair_obs.joystick_obs == OBS_NONE)
	// 	{
	// 		return 1.0;
	// 	}
	// 	else
	// 	{
	// 		return 0.0;
	// 	}
	// }
	// else
	// {
	// ROS_INFO_STREAM("Last action: Other actions...");
	// ROS_INFO_STREAM("Last obs: " << wheelchair_obs.joystick_obs);
	// Obtain the angle between wheelchair heading and the vector pointing from wheelchair position to goal door

	double angle_diff = CalAngleDiff(wheelchair_status, goal_point_idx);

	// // goal door is in the right direction
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

	int direction_index = round(angle_diff * 8 / M_PI);

	direction_index = direction_index == 16 ? 0 : direction_index;

	switch (direction_index)
	{
		case 0:	// intermediate goal is in the front direction (-pi/16 ~ pi/16)
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
			break;
		}
		case 1:	// intermediate goal is in the front left direction (pi/16 ~ 3pi/16)
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
			break;
		}
		case 2: // intermediate goal is in the front left direction (3pi/16 ~ 5pi/16)
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
			break;
		}
		case 3:	// intermediate goal is in the front left direction (5pi/16 ~ 7pi/16)
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
			break;
		}
		case 4:	// intermediate goal is in the left direction (7pi/16 ~ 9pi/16)
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
			break;
		}
		case 5:	// intermediate goal is in the back left direction (9pi/16 ~ 11pi/16)
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
			break;
		}
		case 6:	// intermediate goal is in the back left direction (11pi/16 ~ 13pi/16)
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
			break;
		}
		case 7:	// intermediate goal is in the back left direction (13pi/16 ~ 15pi/16)
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
			break;
		}
		case 8:	// intermediate goal is in the back direction (15pi/16 ~ pi, -pi ~ -15pi/16)
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
			break;
		}
		case 9:	// intermediate goal is in the back right direction (-15pi/16 ~ -13pi/16)
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
			break;
		}
		case 10:	// intermediate goal is in the back right direction (-13pi/16 ~ -11pi/16)
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
			break;
		}
		case 11:	// intermediate goal is in the back right direction (-11pi/16 ~ -9pi/16)
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
			break;
		}
		case 12:	// intermediate goal is in the right direction (-9pi/16 ~ -7pi/16)
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
			break;
		}
		case 13:	// intermediate goal is in the front right direction (-7pi/16 ~ -5pi/16)
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
			break;
		}
		case 14:	// intermediate goal is in the front right direction (-5pi/16 ~ -3pi/16)
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
			break;
		}
		case 15:	// intermediate goal is in the front right direction (-3pi/16 ~ -pi/16)
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
			break;
		}
	}
	// }
}

// only called if POMDP model = world
State* WheelchairDSPOMDP::CreateStartState(string type) const
{
	int max = goal_positions.size();
	int min = 0;
	
	// pick a random goal
	std::srand (std::time (0));
	return new WheelchairState(current_wheelchair_status, std::rand() % (max - min));
}

Belief* WheelchairDSPOMDP::InitialBelief(const State* start, string type) const
{
	vector<State*> particles;
	// Allocate() function allocates some space for creating new state;
	// Equal weights for each possible goal
	int num_goals = goal_positions.size();
	for (int i = 0; i < num_goals; i ++){

		WheelchairState* goal = static_cast<WheelchairState*>(Allocate(i, 1.0 / num_goals));
		goal->path_idx = i;
		// goal->goal_door.x = goal_positions[i].x;
		// goal->goal_door.y = goal_positions[i].y;
		// goal->goal_door.z = goal_positions[i].z;
		// goal->state_goal_point.x = 0;
		// goal->state_goal_point.y = 0;
		// goal->state_goal_point.z = 0;
		// goal->state_goal_index = 0;

		// Set the dummy goal path by assigning a probability of 1 to a certain path and 0s to other paths 
		// if (i == 0)
		// 	goal->weight = 0.4;
		// else
		// 	goal->weight = 0.6;
		goal->wheelchair = current_wheelchair_status;
		particles.push_back(goal);
	}

	return new WheelchairParticleBelief(particles, this, (despot::Belief* )__null, false);	// ParticleBelief udpates the Belief as well!
}

/* ========================
 * Bound-related functions.
 * ========================*/
/*
Note: in the following bound-related functions, only GetMaxReward() and 
GetBestAction() functions are required to be implemented. The other 
functions (or classes) are for custom bounds. You don't need to write them
if you don't want to use your own custom bounds. However, it is highly 
recommended that you build the bounds based on the domain knowledge because
it often improves the performance. Read the tutorial for more details on how
to implement custom bounds.
*/
// The GetMaxReward function returns the maximum possible immediate reward Rmax
double WheelchairDSPOMDP::GetMaxReward() const
{
	return 100;
}

// Customized upper bound
// class WheelchairDSPOMDPParticleUpperBound: public ParticleUpperBound
// {
// protected:
// 	// upper_bounds_[pos][status]:
// 	//   max possible reward when rover_position = pos, and rock_status = status.
// 	vector<vector<double> > upper_bounds_;

// public:
// 	WheelchairDSPOMDPParticleUpperBound(const DSPOMDP* model)
// 	{
// 		upper_bounds_.resize(3);
// 		upper_bounds_[0].push_back(Globals::Discount(1) * 10);
// 		upper_bounds_[0].push_back(10 + Globals::Discount(2) * 10);
// 		upper_bounds_[1].push_back(10);
// 		upper_bounds_[1].push_back(Globals::Discount(1) * 10 + Globals::Discount(3) * 10);
// 		if (upper_bounds_[1][1] < 10)
// 			upper_bounds_[1][1] = 10;
// 		upper_bounds_[2].push_back(0);
// 		upper_bounds_[2].push_back(0);
// 	}

// 	double Value(const State& s) const {
// 		const WheelchairState& state = static_cast<const WheelchairState&>(s);
// 		return upper_bounds_[state.rover_position][state.rock_status];
// 	}
// };

 ScenarioUpperBound* WheelchairDSPOMDP::CreateScenarioUpperBound(string name, string particle_bound_name) const
{
 	ScenarioUpperBound* bound = NULL;
 	if (name == "TRIVIAL" || name == "DEFAULT") {
 		bound = new TrivialParticleUpperBound(this);
 	} 
//	 else if (name == "MAX") {
// 		bound = new WheelchairDSPOMDPParticleUpperBound(this);
// 	} 
else {
 		cerr << "Unsupported base upper bound: " << name << endl;
 		exit(0);
 	}
	 if (Globals::config.useGPU)
		InitGPUUpperBound(name,	particle_bound_name);
	return bound;
}

// The GetBestAction function returns (a, v),
// where a is an action with the largest minimum immediate reward when it is executed,
// and v is its worst-case immediate reward.
ValuedAction WheelchairDSPOMDP::GetBestAction() const
{
	return ValuedAction(LINEAR_MINUS, -1);
}

class WheelchairDSPOMDPPolicy: public DefaultPolicy
{
public:
	enum
	{
		// action
		LINEAR_PLUS = 0, LINEAR_MINUS = 1, ANGULAR_PLUS = 2, ANGULAR_MINUS = 3, ASK = 4, KEEP = 5
	};
	WheelchairDSPOMDPPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
	DefaultPolicy(model, bound)
	{}

	ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const
	{
		const WheelchairState *wheelchair_state_ = static_cast<const WheelchairState*>(particles[0]);
		float linear_v = wheelchair_state_->wheelchair.agent_velocity.linear.x;
		if (round(linear_v * 100) / 100 > 0)
			return LINEAR_MINUS;
		else if (round(linear_v * 100) / 100 < 0)
			return LINEAR_PLUS;
		else
			return -1;
	}
};

ScenarioLowerBound* WheelchairDSPOMDP::CreateScenarioLowerBound(string name, string particle_bound_name) const
{
	ScenarioLowerBound* bound = NULL;
	if (name == "TRIVIAL") {
		bound = new TrivialParticleLowerBound(this);
	} else if (name == "DEFAULT") {
		bound = new WheelchairDSPOMDPPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(0);
	}
	if (Globals::config.useGPU)
		InitGPULowerBound(name, particle_bound_name);

	return bound;
}

// /* =================
//  * Memory management
//  * =================*/

State* WheelchairDSPOMDP::Allocate(int state_id, double weight) const
{
	WheelchairState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	return state;
}

State* WheelchairDSPOMDP::Copy(const State* particle) const
{
	WheelchairState* state = memory_pool_.Allocate();
	*state = *static_cast<const WheelchairState*>(particle);
	state->SetAllocated();
	return state;
}

void WheelchairDSPOMDP::Free(State* particle) const
{
	memory_pool_.Free(static_cast<WheelchairState*>(particle));
}

int WheelchairDSPOMDP::NumActiveParticles() const
{
	return memory_pool_.num_allocated();
}

// /* =======
//  * Display
//  * =======*/

void WheelchairDSPOMDP::PrintState(const State& state, ostream& out) const
{
	const WheelchairState& wheelchair_state = static_cast<const WheelchairState&>(state);
	out << "Wheelchair position: x = " << wheelchair_state.wheelchair.agent_pose.position.x << "; y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
	out << "Wheelchair orientation: x = " << wheelchair_state.wheelchair.agent_pose.orientation.x << "; y = " << wheelchair_state.wheelchair.agent_pose.orientation.y 
		<< "; z = " << wheelchair_state.wheelchair.agent_pose.orientation.z << "; w = " << wheelchair_state.wheelchair.agent_pose.orientation.w << endl;
	out << "Wheelchair velocity: v = " << wheelchair_state.wheelchair.agent_velocity.linear.x << "; w = " << wheelchair_state.wheelchair.agent_velocity.angular.z << endl;
	out << "Joystick input: x = " << wheelchair_state.joystick_x << "; y = " << wheelchair_state.joystick_y << endl;
	out << "Goal position: x = " << goal_positions[wheelchair_state.path_idx].point.x << "; y = " << goal_positions[wheelchair_state.path_idx].point.y << endl;
	out << "Goal index = " << wheelchair_state.path_idx << endl;
}

void WheelchairDSPOMDP::PrintObs(const State& state, OBS_TYPE observation, ostream& out) const
{
	// Use HashMap to get observation
	std::map<uint64_t, WheelchairObs>::iterator it = obsHashMap_execution.find(observation);

	if (it == obsHashMap_execution.end())
	{
		it = obsHashMap.find(observation);
		if (it == obsHashMap.end())
		{
			std::cout << "Calling PrintObs." << std::endl;
			std::cout << "Observation not in hash map and execution hash map. This should not happen." << std::endl;
			// std::cout << "obs hash value " << obs << std::endl;
			// std::map<uint64_t, WheelchairObs>::iterator iter = obsHashMap.begin();
			// while(iter != obsHashMap.end())
			// {
			// 	std::cout << "obsHashMap value " << iter->first << std::endl;
			// 	iter++;
			// }
		}
		// std::cout << "Observation not in execution hash map. This should not happen." << std::endl;
	}

	WheelchairObs wheelchair_obs = it->second;

	string end_s = "\n";
	if(&out == &std::cout)
	{
		end_s = " ";
	}
	// if (wheelchair_obs.joystick_obs == OBS_LEFT)
	// {
	// 	out << "LEFT" << end_s;
	// }
	// else if (wheelchair_obs.joystick_obs  == OBS_RIGHT)
	// {
	// 	out << "RIGHT" << end_s;
	// }
	// else if (wheelchair_obs.joystick_obs  == OBS_NONE)
	// {
	// 	out << "NONE" << end_s;
	// }
	// else if (wheelchair_obs.joystick_obs  == OBS_FR_L)
	// {
	// 	out << "FRONT LEFT" << end_s;
	// }
	// else if (wheelchair_obs.joystick_obs  == OBS_FR_R)
	// {
	// 	out << "FRONT RIGHT" << end_s;
	// }
	// else if (wheelchair_obs.joystick_obs  == OBS_FRONT)
	// {
	// 	out << "FRONT" << end_s;
	// }
	// else
	// {
	// 	out << wheelchair_obs.joystick_obs << end_s;
	// }
	out << wheelchair_obs.joystick_obs << end_s;
}

void WheelchairDSPOMDP::PrintBelief(const Belief& belief, ostream& out) const
{
	const vector<State*>& particles = static_cast<const WheelchairParticleBelief&>(belief).particles();

	std::vector<double> g_weight;
	g_weight.clear();
	g_weight.resize(goal_positions.size(), 0);

	for (int i = 0; i < particles.size(); ++i)
	{
		const WheelchairState* state = static_cast<const WheelchairState*>(particles[i]);

		g_weight[state->path_idx] += state->weight;
	}

	// out << "Belief update: ";
	// for (int i = 0; i < g_weight.size(); ++i)
	// {
	// 	out << "Goal " << i + 1 << ", belief: " << g_weight[i] << "; ";
	// }
	// out << endl;
	cout << "Belief particles number: " << particles.size() << endl;
	for (int i = 0; i < g_weight.size(); ++i)
	{
		cout << "Goal " << i + 1 << ", position: x = " << this->goal_positions[i].point.x << ", y = " << this->goal_positions[i].point.y << ", belief: " << g_weight[i] << endl;
	}
}

void WheelchairDSPOMDP::PrintAction(ACT_TYPE action, ostream& out) const
{
	if (action == LINEAR_PLUS)
		out << "Linear acceleration" << endl;
	if (action == LINEAR_MINUS)
		out << "Linear deceleration" << endl;
	if (action == ANGULAR_PLUS)
		out << "Angular acceleration" << endl;
	if (action == ANGULAR_MINUS)
		out << "Angular deceleration" << endl;
	if (action == KEEP)
		out << "Keep" << endl;
}

float WheelchairDSPOMDP::CollisionCheck(const WheelchairStruct wheelchair_status) const
{
	float x_check = wheelchair_status.agent_pose.position.x;
	float y_check = wheelchair_status.agent_pose.position.y;
	float r_collision = ModelParams::inner_radius;
	float r_outer = ModelParams::outer_radius;
	float temp_dist = 0;
	float penalty_idx = 0;

	for (const auto &point : lidar_points)
	{
		//https://stackoverflow.com/a/7227057
		//Check through easy conditions first, getting distance with sqrt is computationally expensive
		double dx = fabs(point.x - x_check);
		double dy = fabs(point.y - y_check);

		if (dx > r_outer || dy > r_outer)
			continue;

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

int WheelchairDSPOMDP::ReachingCheck(WheelchairState& wheelchair_state) const
{	
	float dist2goal = 0;
	int n = 0;
	double agent_yaw = wheelchair_state.wheelchair.agent_pose_angle;
	double goal_yaw = 0;
	tf2::Quaternion goal_quat;
	// wheelchair_state.state_goal_point = intermediate_goal_list.paths[wheelchair_state.path_idx].poses[0].pose.position;
	// back sweep along the intermediate goals and do the reaching check
	for (int i = intermediate_goal_list.paths[wheelchair_state.path_idx].poses.size() - 1; i >= 0; --i)
	{
		dist2goal = sqrt(powf((wheelchair_state.wheelchair.agent_pose.position.x - intermediate_goal_list.paths[wheelchair_state.path_idx].poses[i].pose.position.x), 2) 
			+ powf((wheelchair_state.wheelchair.agent_pose.position.y - intermediate_goal_list.paths[wheelchair_state.path_idx].poses[i].pose.position.y), 2));
		tf2::convert(intermediate_goal_list.paths[wheelchair_state.path_idx].poses[i].pose.orientation, goal_quat);
		goal_yaw = tf2::getYaw(goal_quat);
		if (dist2goal <= 0.25 && fabs(goal_yaw - agent_yaw) <= M_PI / 6)
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

float WheelchairDSPOMDP::TransitWheelchair(WheelchairState& wheelchair_state) const
{
	float yaw = wheelchair_state.wheelchair.agent_pose_angle;
	float step_time = Globals::config.time_per_move;
	int loop_times = round(transition_time / step_time);
	float penalty_idx = 0;
	float accumulative_penalty = 0;
	for (int i = 0; i < loop_times; i ++)
	{
		penalty_idx = CollisionCheck(wheelchair_state.wheelchair);
		if (penalty_idx == 1)
		{
			return penalty_idx;	// A severe collision happens, transition fails, the task is terminated.
		}
		if (wheelchair_state.wheelchair.agent_velocity.angular.z == 0)
		{
			wheelchair_state.wheelchair.agent_pose.position.x += wheelchair_state.wheelchair.agent_velocity.linear.x * cos(yaw) * step_time;
			wheelchair_state.wheelchair.agent_pose.position.y += wheelchair_state.wheelchair.agent_velocity.linear.x * sin(yaw) * step_time;
		}
		else
		{
			wheelchair_state.wheelchair.agent_pose.position.x += wheelchair_state.wheelchair.agent_velocity.linear.x / wheelchair_state.wheelchair.agent_velocity.angular.z * 
				(sin(yaw + wheelchair_state.wheelchair.agent_velocity.angular.z * step_time) - sin(yaw));
			wheelchair_state.wheelchair.agent_pose.position.y += wheelchair_state.wheelchair.agent_velocity.linear.x / wheelchair_state.wheelchair.agent_velocity.angular.z * 
				(cos(yaw) - cos(yaw + wheelchair_state.wheelchair.agent_velocity.angular.z * step_time));
			yaw += wheelchair_state.wheelchair.agent_velocity.angular.z * step_time;
		}
		accumulative_penalty += penalty_idx;
	}
	penalty_idx = accumulative_penalty / static_cast<float>(loop_times);
	
	tf2::Quaternion wheelchair_quat;
	// tf2::Vector3 joystick_heading(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
	wheelchair_quat.setRPY(0, 0, yaw);
	// tf2::Matrix3x3 rotationMatrix(wheelchair_quat);
	// joystick_heading = rotationMatrix * joystick_heading;
	tf2::convert(wheelchair_quat, wheelchair_state.wheelchair.agent_pose.orientation);

	// Update yaw
	wheelchair_state.wheelchair.agent_pose_angle = yaw;
	// Update joystick input
	// wheelchair_state.joystick_x = joystick_heading.getX();
	// wheelchair_state.joystick_y = joystick_heading.getY();
	
	return penalty_idx;
}

int WheelchairDSPOMDP::RecedingPenalty(WheelchairState& wheelchair_state) const
{
	int receding_penalty = 0, penalty_scale = 0, penalty_norm = 0;
	double angle_diff = CalAngleDiff(wheelchair_state.wheelchair, wheelchair_state.path_idx, ModelParams::num_points_direction);
	if (fabs(angle_diff) <= ModelParams::angle_receding)		// no penalty
	{
		return receding_penalty;
	}
	else
	{
		penalty_norm = round(10 * (fabs(angle_diff) - ModelParams::angle_receding) / (M_PI - ModelParams::angle_receding));	// normalization (0~10)
		penalty_scale = round(fabs(wheelchair_state.wheelchair.agent_velocity.linear.x) * 5);	// scale the penalty based on linear velocity (0~5)
		receding_penalty = ModelParams::receding_basic_penalty * penalty_norm * penalty_scale;		// receding penalty (0~-50)
		return receding_penalty;
	}
}

int WheelchairDSPOMDP::SharedControlReward(WheelchairState& wheelchair_state, double& random_num) const
{
	if (wheelchair_state.joystick_x == 0 && wheelchair_state.joystick_y == 0)
	{
		return 0;
	}
	else
	{
		// float sample_1 = floor(random_num * 100) / 100;
		// float sample_2 = random_num * 100 - floor(random_num * 100);
		// // cout << "Random number " << random_num << ", sample_1 " << sample_1 << ", sample_2 " << sample_2 << endl;

		tf2::Vector3 joystick_heading(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
		// if (sample_1 >= 0.8)
		// {
		// 	std::vector<float> angle_vector, instant_prob;
		// 	float angle_diff = 0, cross_product = 0, angle_sum = 0;
		// 	tf2::Vector3 path_direction(0, 0, 0);
		// 	tf2::Quaternion wheelchair_quat;
		// 	// wheelchair_quat.setRPY(0, 0, wheelchair_state.wheelchair.agent_pose_angle);
		// 	// tf2::Matrix3x3 rotationMatrix(wheelchair_quat);
		// 	// joystick_heading = rotationMatrix * joystick_heading;

		// 	angle_vector.clear();
		// 	instant_prob.clear();
		// 	for (int i = 0; i < intermediate_goal_list.paths.size(); ++i)
		// 	{
		// 		path_direction.setX(goal_positions[i].point.x - wheelchair_state.wheelchair.agent_pose.position.x);
		// 		path_direction.setY(goal_positions[i].point.y - wheelchair_state.wheelchair.agent_pose.position.y);
		// 		angle_diff = path_direction.angle(joystick_heading);

		// 		angle_sum += M_PI - fabs(angle_diff);

		// 		cross_product = joystick_heading.getX() * path_direction.getY() - joystick_heading.getY() * path_direction.getX();
		// 		angle_diff = (cross_product >= 0) ? angle_diff : -angle_diff;
		// 		angle_vector.push_back(angle_diff);
				
		// 		// angle_vector.push_back(M_PI - path_direction.angle(joystick_heading));
		// 	}
		// 	for (int i = 0; i < angle_vector.size(); ++i)
		// 	{
		// 		instant_prob.push_back((M_PI - fabs(angle_vector[i])) / angle_sum);		// instant probability distribution based on the angle difference
		// 	}

		// 	// get the instantaneously desired path id based on the sampled random number		
		// 	int instant_path_idx = 0;
		// 	float prob_sum = 0;
		// 	for (int i = 0; i < instant_prob.size(); ++i)
		// 	{
		// 		prob_sum += instant_prob[i];
		// 		if (sample_2 <= prob_sum)
		// 		{
		// 			instant_path_idx = i;
		// 			break;
		// 		}
		// 	}

		// 	// use the middle direction between desired path direction and current joystick direction as the next joystick direction
		// 	tf2::Vector3 next_joystick;
		// 	// get the relative rotation angle (yaw) with respect to current joystick direction
		// 	float roatation_yaw = angle_vector[instant_path_idx];
		// 	wheelchair_quat.setRPY(0, 0, roatation_yaw);
		// 	tf2::Matrix3x3 rotationMatrix(wheelchair_quat);
		// 	// rotate the joystick input based on the original signals
		// 	// joystick_heading.setValue(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
		// 	next_joystick = rotationMatrix * joystick_heading;
		// 	wheelchair_state.joystick_x = next_joystick.getX();
		// 	wheelchair_state.joystick_y = next_joystick.getY();
		// }

		// reward part
		if (wheelchair_state.wheelchair.agent_velocity.linear.x == 0 && wheelchair_state.wheelchair.agent_velocity.angular.z == 0)
		{
			return ModelParams::stationary_penalty;
		}
		else
		{
			tf2::Vector3 moving_direction(wheelchair_state.wheelchair.agent_velocity.linear.x, wheelchair_state.wheelchair.agent_velocity.angular.z, 0);
			float angle_user_robot = joystick_heading.angle(moving_direction);

			// float velocity_index = fabs(wheelchair_state.wheelchair.agent_velocity.linear.x / wheelchair_state.joystick_x);
			float velocity_index = 1;
			// reward = reward + (0.5 * M_PI -  angle_user_robot) * 60 / M_PI;
			return velocity_index * (0.5 * M_PI -  angle_user_robot) * ModelParams::user_following_reward * 2 / M_PI;
		}
	}
}

double WheelchairDSPOMDP::CalAngleDiff(WheelchairStruct &wheelchair_status, int &goal_idx, int point_idx) const
{
	tf2::Vector3 agent_heading(1, 0, 0), agent2goal(0, 0, 0);
	double angle_difference;
	tf2::Quaternion wheelchair_quat;
	wheelchair_quat.setRPY(0, 0, wheelchair_status.agent_pose_angle);
	tf2::Matrix3x3 rotationMatrix(wheelchair_quat);
	agent_heading = rotationMatrix * agent_heading;
	// Obtain the angle between wheelchair heading and the vector pointing from wheelchair position to a certain point along the intermediate goal list
	if (point_idx == -1 || point_idx >= intermediate_goal_list.paths[goal_idx].poses.size())
	{
		agent2goal.setX(goal_positions[goal_idx].point.x - wheelchair_status.agent_pose.position.x);
		agent2goal.setY(goal_positions[goal_idx].point.y - wheelchair_status.agent_pose.position.y);
	}
	else
	{
		agent2goal.setX(intermediate_goal_list.paths[goal_idx].poses[point_idx - 1].pose.position.x - wheelchair_status.agent_pose.position.x);
		agent2goal.setY(intermediate_goal_list.paths[goal_idx].poses[point_idx - 1].pose.position.y - wheelchair_status.agent_pose.position.y);
	}
	
	angle_difference = agent_heading.angle(agent2goal);

	// use cross product to judge if the angle is positive or negative
	float cross_product = agent_heading.getX() * agent2goal.getY() - agent_heading.getY() * agent2goal.getX();
	angle_difference = (cross_product >= 0)? angle_difference : -angle_difference;

	return angle_difference;
}

float WheelchairDSPOMDP::calRotationValue(float x_input, float y_input, geometry_msgs::Point goal_point) const
{
	tf2::Vector3 agent2goal(goal_point.x, goal_point.y, 0);
	tf2::Vector3 agent_heading(1, 0, 0);
	tf2::Vector3 point2goal(x_input, y_input, 0);
	float V_current = agent_heading.angle(agent2goal);
	float V_next = agent2goal.angle(point2goal);
	float C_action = WheelchairDSPOMDP::calRotationActionCost(V_current);
	// cout << "x_input " << x_input << ", y_input " << y_input << endl;
	// cout << "V_current " << V_current << ", C_action " << C_action << ", V_next " << V_next << endl;
	return 3 * V_next + C_action + V_current;
}

float WheelchairDSPOMDP::calRotationActionCost(float angle_next) const
{
	float angle_threshold_local = M_PI / 2;
	if (angle_next <= angle_threshold_local)
	{
		return angle_next / angle_threshold_local;
	}
	else
	{
		return 1;
	}
}

void WheelchairDSPOMDP::clipBelief(std::vector<despot::State *>& particles_vector) const
{
	for (int i = 0; i < particles_vector.size(); i++)
	{
		if (particles_vector[i]->weight < ModelParams::lower_bound)
		{
			particles_vector[i]->weight = ModelParams::lower_bound;
			continue;
		}
		else if (particles_vector[i]->weight > ModelParams::upper_bound)
		{
			particles_vector[i]->weight = ModelParams::upper_bound;
		}
	}
}

void WheelchairDSPOMDP::normalize(std::vector<despot::State *>& particles_vector) const
{
	float sum_value = 0;
	for (int i = 0; i < particles_vector.size(); i++)
	{
		sum_value += particles_vector[i]->weight;
	}
	for (int i = 0; i < particles_vector.size(); i++)
	{
		particles_vector[i]->weight /= sum_value;
	}
}

void WheelchairDSPOMDP::PrintObs(WheelchairObs& wheelchair_obs, std::ostream& out) const
{
	if (wheelchair_obs.continuous_obs)
	{
		out <<
		"Joy: x " << wheelchair_obs.joystick_signal.x <<
		", y " << wheelchair_obs.joystick_signal.y <<
		"; pos: x " << round(wheelchair_obs.wheelchair_pose.position.x * 100) / 100 <<
		", y " << round(wheelchair_obs.wheelchair_pose.position.y * 100) / 100 <<
		"; ori: x " << round(wheelchair_obs.wheelchair_pose.orientation.x *100) / 100 << 
		", y " << round(wheelchair_obs.wheelchair_pose.orientation.y *100) / 100 <<
		", z " << round(wheelchair_obs.wheelchair_pose.orientation.z *100) / 100 <<
		", w " << round(wheelchair_obs.wheelchair_pose.orientation.w *100) / 100 <<
		"; vel: v " << round(wheelchair_obs.wheelchair_twist.linear.x * 100) / 100 <<
		", w " << round(wheelchair_obs.wheelchair_twist.angular.z * 100) / 100 << endl;
	}
	else
	{
		out << "Joy: " << wheelchair_obs.joystick_obs <<
		"; pos: x " << round(wheelchair_obs.wheelchair_pose.position.x * 100) / 100 <<
		", y " << round(wheelchair_obs.wheelchair_pose.position.y * 100) / 100 <<
		"; ori: x " << round(wheelchair_obs.wheelchair_pose.orientation.x *100) / 100 << 
		", y " << round(wheelchair_obs.wheelchair_pose.orientation.y *100) / 100 <<
		", z " << round(wheelchair_obs.wheelchair_pose.orientation.z *100) / 100 <<
		", w " << round(wheelchair_obs.wheelchair_pose.orientation.w *100) / 100 <<
		"; vel: v " << round(wheelchair_obs.wheelchair_twist.linear.x * 100) / 100 <<
		", w " << round(wheelchair_obs.wheelchair_twist.angular.z * 100) / 100 << endl;
	}
}

} // namespace despot