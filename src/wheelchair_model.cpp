#include "wheelchair_pomdp/wheelchair_model.h"


#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot
{

// action space
// const ACT_TYPE WheelchairDSPOMDP::LINEAR_PLUS = 0;
// const ACT_TYPE WheelchairDSPOMDP::LINEAR_MINUS = 1;
// const ACT_TYPE WheelchairDSPOMDP::ANGULAR_PLUS = 2;
// const ACT_TYPE WheelchairDSPOMDP::ANGULAR_MINUS = 3;
// const ACT_TYPE WheelchairDSPOMDP::KEEP = 4;
// const ACT_TYPE WheelchairDSPOMDP::STOP = 5;
// const ACT_TYPE WheelchairDSPOMDP::FOLLOW = 6;
// const ACT_TYPE WheelchairDSPOMDP::MOVE_TO_GOAL_1 = 7;
// const ACT_TYPE WheelchairDSPOMDP::MOVE_TO_GOAL_2 = 8;
// const ACT_TYPE WheelchairDSPOMDP::MOVE_TO_GOAL_3 = 9;
// const ACT_TYPE WheelchairDSPOMDP::MOVE_TO_GOAL_4 = 10;
// const ACT_TYPE WheelchairDSPOMDP::MOVE_TO_GOAL_5 = 11;

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
        std::cout << "Observation not in hash map and execution hash map. This should not happen. (Update function)" << std::endl;
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

	vector<State*> updated;
	/*	Use goal positions	*/

	if (wheelchair_model_->re_sort)
	{
		// check if the max number of paths have been found
		if (particles_.size() > wheelchair_model_->goal_positions.size() * ModelParams::num_adapt)
		{
			// remove extra particles
			for (int i = 0; i < particles_.size(); ++i)
			{
				State* particle = particles_[i];
				WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
				if (i < wheelchair_model_->goal_positions.size() * ModelParams::num_adapt)
				{
					wheelchair_particle->weight = static_cast<float> (1 / (wheelchair_model_->goal_positions.size() * ModelParams::num_adapt));
					wheelchair_particle->state_id = i;
					wheelchair_particle->path_idx = i / ModelParams::num_adapt;
					wheelchair_particle->adaptability = i % ModelParams::num_adapt;
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
			updated.clear();
		}
		else
		{
			for (int i = 0; i < particles_.size(); ++i)
			{
				State* particle = particles_[i];
				WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
				wheelchair_particle->state_id = i;
				wheelchair_particle->path_idx = i / ModelParams::num_adapt;
				wheelchair_particle->adaptability = i % ModelParams::num_adapt;
				wheelchair_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
				wheelchair_particle->wheelchair.agent_pose.position.x = 0;
				wheelchair_particle->wheelchair.agent_pose.position.y = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.x = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.y = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.z = 0;
				wheelchair_particle->wheelchair.agent_pose.orientation.w = 1;
			}
		}
		wheelchair_model_->re_sort = false;
	}
	
	if (particles_.size() < wheelchair_model_->goal_positions.size() * ModelParams::num_adapt)
	{
		// add new particles to current particles_
		for (int i = particles_.size(); i < wheelchair_model_->goal_positions.size() * ModelParams::num_adapt; ++i)
		{
			WheelchairState* new_particle = static_cast<WheelchairState*>(wheelchair_model_->Allocate(i, 1.0 / (ModelParams::num_adapt * wheelchair_model_->goal_positions.size())));
			new_particle->state_id = i;
			new_particle->path_idx = i / ModelParams::num_adapt;
			new_particle->adaptability = i % ModelParams::num_adapt;
			new_particle->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
			new_particle->wheelchair.agent_pose.position.x = 0;
			new_particle->wheelchair.agent_pose.position.y = 0;
			new_particle->wheelchair.agent_pose.orientation.x = 0;
			new_particle->wheelchair.agent_pose.orientation.y = 0;
			new_particle->wheelchair.agent_pose.orientation.z = 0;
			new_particle->wheelchair.agent_pose.orientation.w = 1;
			// new_particle->weight = 0;	// assign zero probabilities to the appended new paths
			particles_.push_back(new_particle);
		}
	}

	double reward;
	OBS_TYPE o;

	// cout << "User adapatability index = " << wheelchair_model_->adapt_idx << endl;
	// cout << "The action to step in Update function: " << action << endl;
	// cout << "Update: current v = " << wheelchair_obs.wheelchair_twist.linear.x << ", w = " << wheelchair_obs.wheelchair_twist.angular.z << endl;
	// cout << "Update: execution v = " << wheelchair_model_->v_execution << ", w = " << wheelchair_model_->w_execution << endl;
	// cout << "Update: pure DWA v = " << wheelchair_model_->pure_dwa_v << ", w = " << wheelchair_model_->pure_dwa_w << endl;
	// cout << "Update: belief DWA v = " << wheelchair_model_->belief_dwa_v << ", w = " << wheelchair_model_->belief_dwa_w << endl;
	// Update particles

	// Update one paricle first, the rest particles will be the same except for weight, path_idx
	State* particle_1 = particles_[0];
	WheelchairState *wheelchair_particle_1 = static_cast<WheelchairState*>(particle_1);
	wheelchair_particle_1->wheelchair.agent_pose.position.x = 0;
	wheelchair_particle_1->wheelchair.agent_pose.position.y = 0;
	wheelchair_particle_1->wheelchair.agent_pose.orientation.x = 0;
	wheelchair_particle_1->wheelchair.agent_pose.orientation.y = 0;
	wheelchair_particle_1->wheelchair.agent_pose.orientation.z = 0;
	wheelchair_particle_1->wheelchair.agent_pose.orientation.w = 1;
	wheelchair_particle_1->wheelchair.agent_pose_angle = 0;

	wheelchair_particle_1->wheelchair.agent_velocity = wheelchair_obs.wheelchair_twist;
	wheelchair_particle_1->joystick_x = wheelchair_obs.joystick_signal.x;
	wheelchair_particle_1->joystick_y = wheelchair_obs.joystick_signal.y;
	
	// wheelchair_particle_1->collision_idx = wheelchair_model_->collision_index;
	wheelchair_particle_1->num_intermediate_goals = 0;
	wheelchair_particle_1->action_length = 0;
	wheelchair_particle_1->path2waypoint = wheelchair_model_->intermediate_goal_list;
	wheelchair_particle_1->path_traversed.poses.clear(); //Ideally they should already be cleared

	// goal distribution
	wheelchair_particle_1->weight = wheelchair_model_->belief_goal[0] * (1 - wheelchair_model_->adapt_idx);
	// if (!wheelchair_model_->projected_cmd_collision)
	// {
	// 	wheelchair_particle_1->weight = wheelchair_model_->belief_goal[0] * (1 - wheelchair_model_->adapt_idx);
	// }
	// else
	// {
	// 	wheelchair_particle_1->weight = 0;
	// }

	// Call the transition function to update the first particle
	wheelchair_model_->TransitParticles(*wheelchair_particle_1, wheelchair_model_->map_quaternion,
		wheelchair_particle_1->wheelchair.agent_velocity.linear.x,
		wheelchair_model_->v_execution,
		wheelchair_particle_1->wheelchair.agent_velocity.angular.z,
		wheelchair_model_->w_execution);

	double total_weight = 0;

	total_weight += wheelchair_particle_1->weight;
	updated.push_back(particle_1);
	// Update the rest particles
	for (int i = 1; i < particles_.size(); i++)
	{
		State* particle = particles_[i];
		////////Resyncing particles in local frame
		WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);
		wheelchair_particle->wheelchair.agent_pose.position.x = wheelchair_particle_1->wheelchair.agent_pose.position.x;
		wheelchair_particle->wheelchair.agent_pose.position.y = wheelchair_particle_1->wheelchair.agent_pose.position.y;
		wheelchair_particle->wheelchair.agent_pose.orientation.x = wheelchair_particle_1->wheelchair.agent_pose.orientation.x;
		wheelchair_particle->wheelchair.agent_pose.orientation.y = wheelchair_particle_1->wheelchair.agent_pose.orientation.y;
		wheelchair_particle->wheelchair.agent_pose.orientation.z = wheelchair_particle_1->wheelchair.agent_pose.orientation.z;
		wheelchair_particle->wheelchair.agent_pose.orientation.w = wheelchair_particle_1->wheelchair.agent_pose.orientation.w;
		wheelchair_particle->wheelchair.agent_pose_angle = wheelchair_particle_1->wheelchair.agent_pose_angle;
		wheelchair_particle->wheelchair.agent_velocity = wheelchair_particle_1->wheelchair.agent_velocity;
		wheelchair_particle->joystick_x = wheelchair_particle_1->joystick_x;
		wheelchair_particle->joystick_y = wheelchair_particle_1->joystick_y;
		// wheelchair_particle->collision_idx = wheelchair_particle_1->collision_idx;
		wheelchair_particle->num_intermediate_goals = wheelchair_particle_1->num_intermediate_goals;
		wheelchair_particle->action_length = wheelchair_particle_1->action_length;
		wheelchair_particle->path2waypoint = wheelchair_particle_1->path2waypoint;
		wheelchair_particle->path_traversed = wheelchair_particle_1->path_traversed; //Ideally should be empty
		// different part, the weight
		wheelchair_particle->weight = i % ModelParams::num_adapt == 0 ? 
			wheelchair_model_->belief_goal[i / ModelParams::num_adapt] * (1 - wheelchair_model_->adapt_idx) :
			wheelchair_model_->belief_goal[i / ModelParams::num_adapt] * wheelchair_model_->adapt_idx;
		// if (wheelchair_model_->projected_cmd_collision)
		// {
		// 	if (wheelchair_particle->adaptability == 1 && i / ModelParams::num_adapt == wheelchair_model_->instant_index)
		// 	{
		// 		wheelchair_particle->weight = 1;
		// 	}
		// 	else
		// 	{
		// 		wheelchair_particle->weight = 0;
		// 	}
		// }
		// else
		// {
		// 	wheelchair_particle->weight = i % ModelParams::num_adapt == 0 ? 
		// 		wheelchair_model_->belief_goal[i / ModelParams::num_adapt] * (1 - wheelchair_model_->adapt_idx) :
		// 		wheelchair_model_->belief_goal[i / ModelParams::num_adapt] * wheelchair_model_->adapt_idx;
		// }
		total_weight += wheelchair_particle->weight;
		updated.push_back(wheelchair_particle);
	}

	if (wheelchair_model_->dummy_goal_collision)
	{
		wheelchair_model_->user_path = wheelchair_particle_1->path2waypoint.paths[wheelchair_model_->instant_index];
	}

	// update the intermediate goal list after step all particles
	wheelchair_model_->intermediate_goal_list = wheelchair_particle_1->path2waypoint;

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
		wheelchair_model_->re_sort = true;
	}
	// cout << "Updating belief..." << endl;
	// double weight_square_sum = 0;
	for (int i = 0; i < particles_.size(); i++)
	{
		State* particle = particles_[i];
		particle->weight /= total_weight;
		// weight_square_sum += particle->weight * particle->weight;
	}

	// wheelchair_model_->clipBelief(particles_);
	// wheelchair_model_->normalize(particles_);

	// Resample if the effective number of particles is "small"
	// double num_effective_particles = 1.0 / weight_square_sum;
	// if (num_effective_particles < num_particles_ / 2.0)
	// {
	// 	vector<State*> new_belief = Sample(num_particles_, particles_, model_);
	// 	for (int i = 0; i < particles_.size(); i++)
	// 		model_->Free(particles_[i]);

	// 	particles_ = new_belief;
	// }

	// cout << "Short pointing collision check = " << wheelchair_model_->projected_cmd_collision << endl;
	cout << "Pointing collision check = " << wheelchair_model_->dummy_goal_collision << endl;
    cout << "Remapped goal index = " << wheelchair_model_->instant_index + 1 << endl;
    // cout << "User path, size = "<<  wheelchair_model_->user_path.poses.size() << endl;
	// for (int i = 0; i <  wheelchair_model_->user_path.poses.size(); ++i)
	// {
	// 	cout <<  wheelchair_model_->user_path.poses[i].pose.position.x << " " <<  wheelchair_model_->user_path.poses[i].pose.position.y << endl;
	// }

	cout << "Final particles:" << endl;
	for (int i = 0; i < particles_.size(); ++i)
	{
		WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particles_[i]);
		cout << "Goal " << i / ModelParams::num_adapt + 1 << ", adapatability = " << wheelchair_particle->adaptability << ", weight = " << wheelchair_particle->weight << endl;
	}
	// cout << "Belief:" << endl;
	// for (int i = 0; i < wheelchair_model_->belief_goal.size(); ++i)
	// {
	// 	cout << "Goal " << i + 1 << ", position: x = " << wheelchair_model_->goal_positions[i].point.x << ", y = " << wheelchair_model_->goal_positions[i].point.y << ", belief: " << wheelchair_model_->belief_goal[i] << endl;
	// }
	// wheelchair_model_->adapt_idx = temp_adapt_idx;
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
	
	
	tf2::Quaternion wheelchair_quat;
	tf2::convert(current_wheelchair_status.agent_pose.orientation, wheelchair_quat);
	current_wheelchair_status.agent_pose_angle = tf2::getYaw(wheelchair_quat);
	
	current_wheelchair_status.agent_velocity.linear.x = 0;
	current_wheelchair_status.agent_velocity.linear.y = 0;
	current_wheelchair_status.agent_velocity.linear.z = 0;
	current_wheelchair_status.agent_velocity.angular.x = 0;
	current_wheelchair_status.agent_velocity.angular.y = 0;
	current_wheelchair_status.agent_velocity.angular.z = 0;
	goal_positions.reserve(ModelParams::num_paths);
	intermediate_goal_list.paths.reserve(ModelParams::num_paths);
	// pre_positions.clear();
}

WheelchairDSPOMDP::WheelchairDSPOMDP(std::string config_file)
{
	//TODO
}

/* ======
 * Action
 * ======*/

int WheelchairDSPOMDP::NumActions() const
{
	int dwa_actions = 0;
	if(ModelParams::use_dwa_actions)
	{
		dwa_actions = ModelParams::num_dwa_actions;
	}
	return dwa_actions + ModelParams::num_normal_actions + ModelParams::num_paths;
}

/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool WheelchairDSPOMDP::Step(State& state, double rand_num, ACT_TYPE action, double& reward, OBS_TYPE& obs) const
{
    WheelchairState& wheelchair_state = static_cast < WheelchairState& >(state);
	WheelchairStruct& wheelchair_status = wheelchair_state.wheelchair;
	int& goal_point_idx = wheelchair_state.path_idx;

	// check whether the goal is reached
	bool reaching_goal = false;
	// check how many steps required for transition (especially for stop action)
	int transition_steps = 1;

	// tf2::Quaternion map_quat;
	// map_quat.setRPY(0, 0, -agent2map_yaw);

	tf2::Quaternion wheelchair_quat;
	tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);

	// the current wheelchair heading
	tf2::Vector3 agent_heading(1, 0, 0);
	agent_heading = tf2::quatRotate(wheelchair_quat, agent_heading);
	// the current joystick heading
	tf2::Vector3 joystick_heading(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
	// the state's path to the waypoint during the rollout
	voronoi_msgs_and_types::PathList& wheelchair_path = wheelchair_state.path2waypoint;

	double angle_diff;

	std::ostringstream obs_string;
	WheelchairObs wheelchair_obs;
	wheelchair_obs.continuous_obs = false;

	float max_linear_speed = external_joy_x > 0 ? fabs(external_joy_x) : ModelParams::backing_ratio * fabs(external_joy_x);
	max_linear_speed = std::min(max_linear_speed, ModelParams::max_linear_speed);
	// float max_linear_speed = 1.0;
	float v_before = 0, w_before = 0, v_after = 0, w_after = 0, v_follow = 0, w_follow = 0;
	// v_before = v_after = round(wheelchair_status.agent_velocity.linear.x * 100) / 100;
	// w_before = w_after = round(wheelchair_status.agent_velocity.angular.z * 100) / 100;
	v_before = v_after = wheelchair_status.agent_velocity.linear.x;
	w_before = w_after = wheelchair_status.agent_velocity.angular.z;

	float follow_cost = FollowingVel(joystick_heading, v_before, w_before, v_follow, w_follow, max_linear_speed, ModelParams::transition_time);
	tf2::Vector3 follow_heading(v_follow, w_follow, 0);

	float linear_step_size = ModelParams::transition_time * ModelParams::std_v_acceleration * ModelParams::step_scale;
	float angular_step_size = ModelParams::transition_time * ModelParams::std_w_acceleration * ModelParams::step_scale;

	float external_linear_step = ModelParams::planning_time * ModelParams::std_v_acceleration * ModelParams::step_scale;
	float external_angular_step = ModelParams::planning_time * ModelParams::std_w_acceleration * ModelParams::step_scale;

	// cout << "v_before = " << v_before << ", w_before = " << w_before << endl;
	// cout << "action = " << action << endl;
	// cout << "v_follow = " << v_follow << ", w_follow = " << w_follow << endl;
	int total_normal_actions = ModelParams::num_normal_actions + ModelParams::num_dwa_actions;

	float scale_factor = 1.0; // wheelchair_state.collision_idx > 0 ? ModelParams::step_scale : 1.0;

	switch (action)
	{
	case 0:	// linear acceleration
	{
		if (v_before >= ModelParams::max_linear_speed)
		{
			reward = ModelParams::excess_speed_penalty;
			v_after = ModelParams::max_linear_speed;
		}
		else
		{
			v_after = v_before + linear_step_size * scale_factor; // Increase the linear speed by ModelParams::step_size
			v_after = v_after >= ModelParams::max_linear_speed ? ModelParams::max_linear_speed : v_after;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 1:	// linear deceleration
	{
		if (v_before <= - ModelParams::max_linear_speed)
		{
			reward = ModelParams::excess_speed_penalty;
			v_after = - ModelParams::max_linear_speed;
		}
		else
		{
			v_after = v_before - linear_step_size * scale_factor; // Decrease the linear speed by ModelParams::step_size
			v_after = v_after <= - ModelParams::max_linear_speed ? - ModelParams::max_linear_speed : v_after;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 2:	// angular acceleration
	{
		if (w_before >= ModelParams::max_angular_speed)
		{
			reward = ModelParams::excess_speed_penalty;
			w_after = ModelParams::max_angular_speed;
		}
		else
		{
			w_after = w_before + angular_step_size * scale_factor; // Increase the angular speed by ModelParams::step_size
			w_after = w_after >= ModelParams::max_angular_speed ? ModelParams::max_angular_speed : w_after;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 3: // angular deceleration
	{
		if (w_before <= - ModelParams::max_angular_speed)
		{
			reward = ModelParams::excess_speed_penalty;
			w_after = - ModelParams::max_angular_speed;
		}
		else
		{
			w_after = w_before - angular_step_size * scale_factor; // Decrease the angular speed by ModelParams::step_size
			w_after = w_after <= - ModelParams::max_angular_speed ? - ModelParams::max_angular_speed : w_after;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		break;
	}
	case 4:	// keep
	{
		reward = ModelParams::step_penalty; // Penalize the step
		if (fabs(external_joy_x) > 0.1 && fabs(v_before) < 0.1 && fabs(w_before) < 0.1)
		{
			reward += fabs(external_joy_x) * ModelParams::stationary_penalty / external_linear_step;
		}
		break;
	}
	case 5:	// stop
	{
		// stop the wheelchair
		v_after = 0.0;
		w_after = 0.0;
		if (fabs(v_before) > linear_step_size)
		{
			reward = ModelParams::step_penalty + ModelParams::stationary_penalty * (fabs(v_before) - linear_step_size);
		}
		else
		{
			reward = ModelParams::step_penalty;
		}
		if (fabs(w_before) > angular_step_size)
		{
			reward += ModelParams::stationary_penalty * (fabs(w_before) - angular_step_size);
		}
		if (fabs(external_joy_x) > 0.1)
		{
			reward += fabs(external_joy_x) * ModelParams::stationary_penalty / external_linear_step;
		}
		break;
	}
	case 6:	// follow
	{
		v_after = v_follow;
		w_after = w_follow;

		reward = ModelParams::step_penalty; // Penalize the step
		break;
	}
	case 7: // pure dwa action
	{
		// cout << "cASE 12: " << endl;
		// cout << "wheelchair_state.action_length = " << wheelchair_state.action_length << ", wheelchair_state.action_before = " << wheelchair_state.action_before << endl;
		if(wheelchair_state.action_length == 0 || wheelchair_state.action_before == action)
		{
			v_after = pure_dwa_v;
			w_after = pure_dwa_w;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		else // Cannot execute dwa action after any other action because the v and w won't be valid
		{
			reward = ModelParams::collision_penalty / (1 - ModelParams::reward_discount);
			return true;
		}
		break;
	}
	case 8: // belief dwa action
	{
		if(wheelchair_state.action_length == 0 || wheelchair_state.action_before == action)
		{
			v_after = belief_dwa_v;
			w_after = belief_dwa_w;
			reward = ModelParams::step_penalty; // Penalize the step
		}
		else // Cannot execute dwa action after any other action because the v and w won't be valid
		{
			reward = ModelParams::collision_penalty / (1 - ModelParams::reward_discount);
			return true;
		}
		break;
	}
	default:	// move to goals
	{
		// cout << "Move to goal " << action - total_normal_actions + 1 << "..." << endl;
		// first, contract the path for move to goal X

		// if (action == total_normal_actions)
		// {
		// 	cout << "Original path, size = "<< intermediate_goal_list.paths[action - total_normal_actions].poses.size() << endl;
		// 	for (int i = 0; i < intermediate_goal_list.paths[action - total_normal_actions].poses.size(); ++i)
		// 	{
		// 		cout << intermediate_goal_list.paths[action - total_normal_actions].poses[i].pose.position.x << " "
		// 			<< intermediate_goal_list.paths[action - total_normal_actions].poses[i].pose.position.y << endl;
		// 	}

		// 	cout << "Before contraction, wheelchair path size = "<< wheelchair_path.paths[action - total_normal_actions].poses.size() << endl;
		// 	for (int i = 0; i < wheelchair_path.paths[action - total_normal_actions].poses.size(); ++i)
		// 	{
		// 		cout << wheelchair_path.paths[action - total_normal_actions].poses[i].pose.position.x << " "
		// 			<< wheelchair_path.paths[action - total_normal_actions].poses[i].pose.position.y << endl;
		// 	}
		// }
		// float initial_x = wheelchair_status.agent_pose.position.x;
		// float initial_y = wheelchair_status.agent_pose.position.y;
		// float yaw_M2G = wheelchair_status.agent_pose_angle;
		// tf2::Quaternion quat_M2G;
		// joystick_heading = tf2::quatRotate(wheelchair_quat, joystick_heading);
		float after_x = 0, after_y = 0;
		int total_steps2turn = 0;

		// cout << "initial_x = " << initial_x << ", initial_y = " << initial_y << endl;

		// cout << "initial_v = " << v_before << ", initial_w = " << w_before << endl;

		// cout << "Finish path contraction for move to goal " << action - total_normal_actions << endl;

		// bool stop_wheechair = false;
		reward = 0;
		double reward_discount = 1;
		int point_along_path = 0;	// the next point along the contracted path
		tf2::Vector3 agent2path(0, 0, 0);
		// bool compute_user_reward = true;

		if (wheelchair_state.action_length == 0)
		{
			reward += ModelParams::collision_penalty;
		}

		// if the particle is to follow user and the dummy path is not remapped to an existing path, do the reaching check
		if (wheelchair_state.adaptability == false)
		{
			reward += InstantGoalReachingCheck(wheelchair_state, reaching_goal);
			// Reach the final goal
			if (reaching_goal)
			{
				return true;	// Only terminate the task when a final goal is reached
			}
		}

		// first stop the wheelchair if it's still moving
		// the vector linked to the next point along the path
		// agent2path.setValue(wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.x
		// 	- wheelchair_status.agent_pose.position.x,
		// 	wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.y
		// 	- wheelchair_status.agent_pose.position.y, 0);
		// while (agent2path.length() == 0)
		// {
		// 	point_along_path++;
		// 	agent2path.setValue(wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.x
		// 		- wheelchair_status.agent_pose.position.x,
		// 		wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.y
		// 		- wheelchair_status.agent_pose.position.y, 0);
		// }
		if (wheelchair_state.path_traversed.poses.size() > 0)
		{
			// cout << "Contracting path..." << endl;
			// only contract the path when new points are added to the current path
			ContractAndInterpolatePath(wheelchair_status, wheelchair_state.path_traversed, wheelchair_path.paths[action - total_normal_actions], map_quaternion);
		}

		// cout << "Goal " << action - total_normal_actions + 1 << ", position x = "
		// 	<< wheelchair_path.paths[action - total_normal_actions].poses[wheelchair_path.paths[action - total_normal_actions].poses.size() - 1].pose.position.x
		// 	<< ", position y = "
		// 	<< wheelchair_path.paths[action - total_normal_actions].poses[wheelchair_path.paths[action - total_normal_actions].poses.size() - 1].pose.position.y
		// 	<< endl;
		
		// // the angle between current heading and the direction facing the path
		// float angle2turn = agent_heading.angle(agent2path);
		// float transition_penalty = 0;
		float user_reward = 0, path_reward = 0, collision_reward = 0;

		// if (fabs(v_before) > 0.1)
		// {
		// 	// cout << "Stop the wheelchair because it's moving " << endl;
		// 	stop_wheechair = true;
		// 	v_after = 0.0;
		// 	w_after = 0.0;
		// 	if (fabs(v_before) > linear_step_size)
		// 	{
		// 		reward += ModelParams::step_penalty + ModelParams::stationary_penalty * (fabs(v_before) - linear_step_size);
		// 	}
		// 	else
		// 	{
		// 		reward += ModelParams::step_penalty;
		// 	}

		// 	// cout << "stop moving v_before = " << v_before << ", v_after = " << v_after << endl;
		// 	// cout << "stop moving w_before = " << w_before << ", w_after = " << w_after << endl;

		// 	//GenerateNewPath(wheelchair_status, wheelchair_path.paths[action - total_normal_actions]);
		// 	GenerateNewPath(wheelchair_status, wheelchair_state.path_traversed);

		// 	collision_reward = TransitWheelchair(wheelchair_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);

		// 	if (collision_reward < ModelParams::collision_penalty)
		// 	{
		// 		collision_reward = 0;
		// 	}
		// 	// transition_penalty = transition_reward;
		// 	reward += collision_reward * reward_discount;
		// 	// float transition_reward = TransitWheelchair(wheelchair_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
		// 	// // cout << "transition_reward = " << transition_reward << endl;
		// 	// // cout << "Reward = " << reward << endl;
		// 	// reward += transition_reward;
		// 	// transition_penalty = transition_reward;

		// 	// if (transition_reward <= ModelParams::collision_penalty)
		// 	// {
		// 	// 	// a collision happened
		// 	// 	return true;
		// 	// }

		// 	// compute the reward
		// 	// cout << "discount_factor = " << transition_steps - 1 << endl;
		// 	// path_reward = ReachingCheck(wheelchair_state, reaching_goal);
		// 	// cout << "Reward for following path = " << path_reward << endl;
		// 	if (wheelchair_state.adaptability)	// follow the path
		// 	{
		// 		path_reward = ReachingCheck(wheelchair_state, reaching_goal);
		// 		reward += path_reward * powf(ModelParams::reward_discount, transition_steps - 1);
		// 		// Reach the final goal
		// 		if (reaching_goal)
		// 		{
		// 			return true;	// Only terminate the task when a final goal is reached
		// 		}
		// 	}
		// 	else	// follow the user
		// 	{
		// 		after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
		// 		after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
		// 		user_reward = FollowUserReward(v_follow, w_follow, after_x, after_y, follow_cost);
		// 		// cout << "Reward for following user = " << user_reward << endl;
		// 		reward += user_reward * powf(ModelParams::reward_discount, transition_steps - 1);
		// 		// reward += FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * powf(ModelParams::reward_discount, transition_steps - 1);
		// 	}

		// 	// cout << "Reward = " << reward << endl;
		// 	// cout << "Finish path contraction for move to goal after stopping the wheelchair" << action - total_normal_actions << endl;
		// 	v_before = 0;
		// 	w_before = 0;
		// }

		// if (wheelchair_state.path_traversed.poses.size() > 0)
		// {
		// 	// cout << "Contracting path..." << endl;
		// 	// only contract the path when new points are added to the current path
		// 	ContractAndInterpolatePath(wheelchair_status, wheelchair_state.path_traversed, wheelchair_path.paths[action - total_normal_actions], map_quat);
		// }

		// if (action == total_normal_actions)
		// {
		// 	cout << "After contraction, wheelchair path size = "<< wheelchair_path.paths[action - total_normal_actions].poses.size() << endl;
		// 	for (int i = 0; i < wheelchair_path.paths[action - total_normal_actions].poses.size(); ++i)
		// 	{
		// 		cout << wheelchair_path.paths[action - total_normal_actions].poses[i].pose.position.x << " "
		// 			<< wheelchair_path.paths[action - total_normal_actions].poses[i].pose.position.y << endl;
		// 	}
		// }

		// simulate the rest steps after stopping the wheelchair
		// int num2simulate = stop_wheechair ? ModelParams::num_simulation_m2g - 1 : ModelParams::num_simulation_m2g;
		int num2simulate = ModelParams::num_simulation_m2g;
		// cout << "Simulating " << num2simulate << " steps for move to goal " << action - total_normal_actions << endl;
		// cout << "Path " << action - total_normal_actions + 1 << ", size = " << wheelchair_path.paths[action - total_normal_actions].poses.size() << endl;
		for (int i = 0; i < ModelParams::num_simulation_m2g;)
		{
			if (point_along_path >= wheelchair_path.paths[action - total_normal_actions].poses.size())
			{
				// cout << "Simulated " << i + 1 << " steps." << endl;
				num2simulate = i + 1;
				break;
			}
			// reward_discount = stop_wheechair ? pow(ModelParams::reward_discount, i + transition_steps) : pow(ModelParams::reward_discount, i);
			reward_discount = powf(ModelParams::reward_discount, i);
			// wheechair is not moving, turn in place to face towards the contracted path
			agent2path.setValue(wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.x
				- wheelchair_status.agent_pose.position.x,
				wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.y
				- wheelchair_status.agent_pose.position.y, 0);
			
			// cout << "agent2path.length() = " << agent2path.length() << endl;
			// cout << "point_along_path = " << point_along_path << endl;

			while (agent2path.length() <= 0.1 && point_along_path < wheelchair_path.paths[action - total_normal_actions].poses.size() - 1)
			{
				point_along_path++;
				agent2path.setValue(wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.x
					- wheelchair_status.agent_pose.position.x,
					wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.y
					- wheelchair_status.agent_pose.position.y, 0);
			}

			// cout << "agent2path.length() after while loop = " << agent2path.length() << endl;
			// cout << "point_along_path after while loop = " << point_along_path << endl;
			// the angle between current heading and the direction facing the path

			tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);

			// the current wheelchair heading
			agent_heading.setValue(1, 0, 0);
			agent_heading = tf2::quatRotate(wheelchair_quat, agent_heading);
			float angle2turn = agent_heading.angle(agent2path);

			float cross_product = agent_heading.getX() * agent2path.getY() - agent_heading.getY() * agent2path.getX();
			angle2turn = (cross_product >= 0)? angle2turn : -angle2turn;

			// if (wheelchair_state.action_length == 0)
			// {
			// 	cout << "Moving to goal " << action - total_normal_actions + 1 << endl;
			// 	cout << "point_along_path = " << point_along_path << endl;
			// 	cout << "w_before = " << w_before << endl;
			// 	cout << "angle2turn = " << angle2turn * 180 / M_PI << endl;
			// 	cout << "agent_pose_angle = " << wheelchair_status.agent_pose_angle * 180 / M_PI << endl;
			// }

			// cout << "point_along_path = " << point_along_path << endl;
			// cout << "w_before = " << w_before << endl;
			// cout << "angle2turn = " << angle2turn * 180 / M_PI << endl;
			// cout << "agent_pose_angle = " << wheelchair_status.agent_pose_angle << endl;

			// the wheelchair is facing the path
			if (fabs(angle2turn) <= ModelParams::facing_angle)
			{
				// the wheelchair is facing the path, but still turning, stop the wheelchair
				if (fabs(w_before) > 0.1)
				{
					// cout << "Stop the wheelchair because it's turning " << endl;
					v_after = 0.0;
					w_after = 0.0;
					if (fabs(v_before) > linear_step_size)
					{
						reward += ModelParams::step_penalty + ModelParams::stationary_penalty * (fabs(v_before) - linear_step_size);
					}
					else
					{
						reward += ModelParams::step_penalty;
					}
					if (fabs(w_before) > angular_step_size)
					{
						reward += ModelParams::stationary_penalty * (fabs(w_before) - angular_step_size);
					}

					// cout << "stop moving v_before = " << v_before << ", v_after = " << v_after << endl;
					// cout << "stop moving w_before = " << w_before << ", w_after = " << w_after << endl;

					//GenerateNewPath(wheelchair_status, wheelchair_path.paths[action - total_normal_actions]);
					// GenerateNewPath(wheelchair_status, wheelchair_state.path_traversed);

					// yaw_M2G = wheelchair_status.agent_pose_angle;

					collision_reward = TransitWheelchair(wheelchair_state, map_quaternion, transition_steps, v_before, v_after, w_before, w_after);

					// yaw_M2G = yaw_M2G - wheelchair_status.agent_pose_angle;
					// quat_M2G.setRPY(0, 0, yaw_M2G);
					// follow_heading = tf2::quatRotate(quat_M2G, follow_heading);

					// if (collision_reward < ModelParams::collision_penalty)
					// {
					// 	collision_reward = 0;
					// }
					// collision_reward = 0;
					// transition_penalty = transition_reward;
					reward += collision_reward * reward_discount;
					// float transition_reward = TransitWheelchair(wheelchair_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
					// // cout << "transition_reward = " << transition_reward << endl;
					// // cout << "Reward = " << reward << endl;
					// reward += transition_reward;
					// transition_penalty = transition_reward;

					// if (transition_reward <= ModelParams::collision_penalty)
					// {
					// 	// a collision happened
					// 	return true;
					// }

					// compute the reward
					// cout << "discount_factor = " << reward_discount << endl;
					// path_reward = ReachingCheck(wheelchair_state, reaching_goal);
					// cout << "Reward for following path = " << path_reward << endl;
					if (wheelchair_state.adaptability)	// follow the path
					{
						path_reward = ReachingCheck(wheelchair_state, reaching_goal);
						reward += reward_discount * path_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// Reach the final goal
						if (reaching_goal)
						{
							return true;	// Only terminate the task when a final goal is reached
						}
					}
					else	// follow the user
					{
						// if (compute_user_reward)
						// {
						// 	// after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
						// 	// after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
						// 	user_reward = FollowUserReward(follow_heading, v_after, w_after, follow_cost);
						// 	// cout << "Reward for following user = " << user_reward << endl;
						// 	reward += reward_discount * user_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// 	// reward += reward_discount * FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * powf(ModelParams::reward_discount, transition_steps - 1);
						// 	compute_user_reward = false;
						// }
						user_reward = InstantGoalReachingCheck(wheelchair_state, reaching_goal);
						reward += reward_discount * user_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// Reach the final goal
						if (reaching_goal)
						{
							return true;	// Only terminate the task when a final goal is reached
						}
					}
					// cout << "Reward = " << reward << endl;
					v_before = v_after;
					w_before = w_after;
					i++;
				}

				// the wheelchair is facing the path, and not turning, move along the path at a slow speed
				else
				{
					// v_after = ModelParams::planning_time * ModelParams::max_v_acceleration * ModelParams::step_scale;
					v_after = 0;
					w_after = 0;

					wheelchair_status.agent_velocity.linear.x = v_after;
					wheelchair_status.agent_velocity.angular.z = 0;
					// cout << "Move the wheelchair along the path " << endl;
					wheelchair_status.agent_pose.position.x = wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.x;
					wheelchair_status.agent_pose.position.y = wheelchair_path.paths[action - total_normal_actions].poses[point_along_path].pose.position.y;

					wheelchair_status.agent_velocity.linear.x = 0;
					wheelchair_status.agent_velocity.angular.z = 0;
					// cout << "x = " << wheelchair_status.agent_pose.position.x << endl;
					// cout << "y = " << wheelchair_status.agent_pose.position.y << endl;
					// max linear speed increment is max_acceleration * transition_time, namely 1 * 0.3 = 0.3
					// count how many steps are required for reaching the next point along the path
					// area of one rectangle is a*t^2
					float base_area = ModelParams::max_v_acceleration * ModelParams::transition_time * ModelParams::transition_time;

					v_after = ModelParams::planning_time * ModelParams::std_v_acceleration * ModelParams::step_scale;

					if (fabs(v_before - v_after) > linear_step_size)
					{
						reward += ModelParams::stationary_penalty * (fabs(v_before - v_after) - linear_step_size) / linear_step_size;
					}

					// stop after the moving
					// v_after = 0;
					// w_after = 0;

					// after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
					// after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
					// yaw_M2G = yaw_M2G - wheelchair_status.agent_pose_angle;
					// quat_M2G.setRPY(0, 0, yaw_M2G);
					// follow_heading = tf2::quatRotate(wheelchair_quat, follow_heading);

					user_reward = 0;
					path_reward = 0;
					if (wheelchair_state.adaptability)
					{
						path_reward = ReachingCheck(wheelchair_state, reaching_goal);
					}
					else
					{
						// if (compute_user_reward)
						// {
						// 	user_reward = FollowUserReward(follow_heading, v_after, w_after, follow_cost);
						// 	// cout << "v_follow = " << follow_heading.getX() << ", w_follow = " << follow_heading.getY() << endl;
						// 	// cout << "v_after = " << v_after << ", w_after = " << w_after << endl;
						// 	// cout << "follow_cost = " << follow_cost << endl;
						// 	// cout << "user_reward = " << user_reward << endl;
						// 	reward += user_reward * reward_discount;
						// 	compute_user_reward = false;
						// }
						user_reward = InstantGoalReachingCheck(wheelchair_state, reaching_goal);
					}

					v_after = 0;

					// float M2G_collision = CollisionCheck(wheelchair_status, map_quaternion);
					// if (M2G_collision == 1)
					// {
					// 	collision_reward = ModelParams::collision_penalty;
					// }
					// else
					// {
					// 	collision_reward = 0;
					// }
					collision_reward = 0;
					// cout << "Reward for following user = " << user_reward << endl;
					// cout << "Reward for following path = " << path_reward << endl;

					// 2 steps are required: linear+, linear-
					// cout << "agent2path.length() = " << agent2path.length() << endl;
					if (agent2path.length() <= base_area)
					{
						// compute the reward
						for (int j = 0; j < 1; ++j)
						{
							float extra_discount = pow(ModelParams::reward_discount, j);
							// step penalty
							reward += (ModelParams::step_penalty + collision_reward) * reward_discount * extra_discount;
							// follow user reward
							// if (!wheelchair_state.adaptability)
							// {
							// 	// v_after = 0.5 * ModelParams::max_v_acceleration * ModelParams::transition_time;
							// 	// w_after = 0;
							// 	reward += user_reward * reward_discount * extra_discount;
							// 	// cout << "2 steps, reward = " << reward << endl;
							// 	// reward += FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * reward_discount * extra_discount;
							// }
						}

						// follow the path reward
						if (wheelchair_state.adaptability)
						{
							reward_discount *= pow(ModelParams::reward_discount, 0);	// 1 more step
							reward += reward_discount * path_reward;
						}

						i = i + 1;	// 2 steps
					}
					// 3 steps are required: linear+, keep, linear-
					else if (agent2path.length() <= 2 * base_area)
					{
						// compute the reward
						for (int j = 0; j < 2; ++j)
						{
							float extra_discount = pow(ModelParams::reward_discount, j);
							// step penalty
							reward += (ModelParams::step_penalty + collision_reward) * reward_discount * extra_discount;
							// follow user reward
							// if (!wheelchair_state.adaptability)
							// {
							// 	// v_after = 0.6667 * ModelParams::max_v_acceleration * ModelParams::transition_time;
							// 	// w_after = 0;
							// 	reward += user_reward * reward_discount * extra_discount;
							// 	// cout << "3 steps, reward = " << reward << endl;
							// 	// reward += FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * reward_discount * extra_discount;
							// }
						}

						// follow the path reward
						if (wheelchair_state.adaptability)
						{
							reward_discount *= pow(ModelParams::reward_discount, 1);	// 2 more steps
							reward += reward_discount * path_reward;
						}
						
						i = i + 2;	// 3 steps
					}
					// 4 steps are required: linear+, linear+/keep, linear-/keep, linear-
					else
					{
						// compute the reward
						for (int j = 0; j < 3; ++j)
						{
							float extra_discount = pow(ModelParams::reward_discount, j);
							// step penalty
							reward += (ModelParams::step_penalty + collision_reward) * reward_discount * extra_discount;
							// follow user reward
							// if (!wheelchair_state.adaptability)
							// {
							// 	// v_after = 0.75 * ModelParams::max_v_acceleration * ModelParams::transition_time;
							// 	// w_after = 0;
							// 	reward += user_reward * reward_discount * extra_discount;
							// 	// cout << "4 steps, reward = " << reward << endl;
							// 	// reward += FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * reward_discount * extra_discount;
							// }
						}

						// follow the path reward
						if (wheelchair_state.adaptability)
						{
							reward_discount *= pow(ModelParams::reward_discount, 2);	// 3 more steps
							reward += reward_discount * path_reward;
						}
						
						i = i + 3;	// 4 steps
					}
					// Reach the final goal
					if (reaching_goal)
					{
						return true;	// Only terminate the task when a final goal is reached
					}
					v_before = v_after;
					w_before = w_after;
					point_along_path++;	// move to the next point
				}

			}

			// the wheelchair needs to turn to face the path
			else
			{
				// stop the wheelchair because it's moving
				if (fabs(v_before) > 0.1)
				{
					// cout << "Stop the wheelchair because it's moving " << endl;
					v_after = 0.0;
					w_after = 0.0;
					if (fabs(v_before) > linear_step_size)
					{
						reward += ModelParams::step_penalty + ModelParams::stationary_penalty * (fabs(v_before) - linear_step_size);
					}
					else
					{
						reward += ModelParams::step_penalty;
					}
					if (fabs(w_before) > angular_step_size)
					{
						reward += ModelParams::stationary_penalty * (fabs(w_before) - angular_step_size);
					}

					// cout << "stop moving v_before = " << v_before << ", v_after = " << v_after << endl;
					// cout << "stop moving w_before = " << w_before << ", w_after = " << w_after << endl;

					//GenerateNewPath(wheelchair_status, wheelchair_path.paths[action - total_normal_actions]);
					// GenerateNewPath(wheelchair_status, wheelchair_state.path_traversed);

					// yaw_M2G = wheelchair_status.agent_pose_angle;

					collision_reward = TransitWheelchair(wheelchair_state, map_quaternion, transition_steps, v_before, v_after, w_before, w_after);

					// yaw_M2G = yaw_M2G - wheelchair_status.agent_pose_angle;
					// quat_M2G.setRPY(0, 0, yaw_M2G);
					// follow_heading = tf2::quatRotate(yaw_M2G, follow_heading);

					// if (collision_reward < ModelParams::collision_penalty)
					// {
					// 	collision_reward = 0;
					// }
					// collision_reward = 0;
					// transition_penalty = transition_reward;
					reward += collision_reward * reward_discount;
					// float transition_reward = TransitWheelchair(wheelchair_state, map_quat, transition_steps, v_before, v_after, w_before, w_after);
					// // cout << "transition_reward = " << transition_reward << endl;
					// // cout << "Reward = " << reward << endl;
					// reward += transition_reward;
					// transition_penalty = transition_reward;

					// if (transition_reward <= ModelParams::collision_penalty)
					// {
					// 	// a collision happened
					// 	return true;
					// }

					// compute the reward
					// cout << "discount_factor = " << reward_discount << endl;
					// path_reward = ReachingCheck(wheelchair_state, reaching_goal);
					// cout << "Reward for following path = " << path_reward << endl;
					if (wheelchair_state.adaptability)	// follow the path
					{
						path_reward = ReachingCheck(wheelchair_state, reaching_goal);
						reward += reward_discount * path_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// Reach the final goal
						if (reaching_goal)
						{
							return true;	// Only terminate the task when a final goal is reached
						}
					}
					else	// follow the user
					{
						// if (compute_user_reward)
						// {
						// 	// after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
						// 	// after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
						// 	user_reward = FollowUserReward(follow_heading, v_after, w_after, follow_cost);
						// 	// cout << "Reward for following user = " << user_reward << endl;
						// 	reward += reward_discount * user_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// 	// reward += reward_discount * FollowUserReward(v_follow, w_follow, v_after, w_after, follow_cost) * powf(ModelParams::reward_discount, transition_steps - 1);
						// 	compute_user_reward = false;
						// }
						user_reward = InstantGoalReachingCheck(wheelchair_state, reaching_goal);
						reward += reward_discount * user_reward * powf(ModelParams::reward_discount, transition_steps - 1);
						// Reach the final goal
						if (reaching_goal)
						{
							return true;	// Only terminate the task when a final goal is reached
						}
					}
					// cout << "Reward = " << reward << endl;
					v_before = v_after;
					w_before = w_after;
					i++;
				}
				// the wheelchair is not moving, but needs to turn to face the path
				else
				{
					// Compute the move to goal collision penalty here
					// cout << "M2G_CollisionCheck" << endl;
					// collision_reward = M2G_CollisionCheck(wheelchair_status, map_quaternion, angle2turn);

					// if (wheelchair_state.action_length == 0)
					// {
					// 	cout << "Moving to goal " << action - total_normal_actions + 1 << endl;
					// 	cout << "collision_reward = " << collision_reward << endl;
					// 	cout << "w_before = " << w_before << endl;
					// 	cout << "angle2turn = " << angle2turn * 180 / M_PI << endl;
					// 	cout << "agent_pose_angle = " << wheelchair_status.agent_pose_angle * 180 / M_PI << endl;
					// }
					// cout << "Turn the wheelchair to face the path" << endl;

					// cout << "angle2turn = " << angle2turn * 180 / M_PI << endl;

					// cout << "agent_pose_angle before = " << wheelchair_status.agent_pose_angle << endl;

					float angular_vel = 0;

					int steps2turn = TurningSteps(angle2turn, w_before, angular_vel);
					// cout << "steps2turn = " << steps2turn << endl;

					total_steps2turn += steps2turn;

					// stop after the turning
					v_after = 0;
					w_after = ModelParams::planning_time * ModelParams::std_w_acceleration * ModelParams::step_scale;

					w_after = angle2turn > 0 ? w_after : - w_after;

					// after turning, update the wheelchair status
					// the orientation now is aligned with angle2path

					// yaw_M2G = wheelchair_status.agent_pose_angle;

					wheelchair_status.agent_pose_angle += angle2turn;
					wheelchair_quat.setRPY(0, 0, wheelchair_status.agent_pose_angle);
					tf2::convert(wheelchair_quat, wheelchair_status.agent_pose.orientation);

					// wheelchair_quat.setRPY(0, 0, -angle2turn);
					// joystick_heading = tf2::quatRotate(wheelchair_quat, joystick_heading);

					// cout << "agent_pose_angle after = " << wheelchair_status.agent_pose_angle << endl;

					wheelchair_status.agent_velocity.linear.x = 0;
					wheelchair_status.agent_velocity.angular.z = 0;

					// yaw_M2G = yaw_M2G - wheelchair_status.agent_pose_angle;
					// quat_M2G.setRPY(0, 0, yaw_M2G);
					// follow_heading = tf2::quatRotate(quat_M2G, follow_heading);
					
					// tf2::Quaternion joystick_quat;
					// joystick_quat.setRPY(0, 0, - angle2turn);
					// joystick_heading.setValue(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
					// joystick_heading = tf2::quatRotate(joystick_quat, joystick_heading);
					// wheelchair_state.joystick_x = round(joystick_heading.getX() * 100) / 100;
					// wheelchair_state.joystick_y = round(joystick_heading.getY() * 100) / 100;

					// compute the reward
					// for (int j = 0; j < steps2turn; ++j)
					// {
					// 	float extra_discount = pow(ModelParams::reward_discount, j);
					// 	// step penalty
					// 	reward += (ModelParams::step_penalty + collision_reward) * reward_discount * extra_discount;
					// 	// follow user reward
					// 	if (!wheelchair_state.adaptability)
					// 	{
					// 		if (compute_user_reward)
					// 		{
					// 			user_reward = FollowUserReward(follow_heading, v_after, w_after, follow_cost);
					// 			reward += user_reward * reward_discount * extra_discount;
					// 			compute_user_reward = false;
					// 		}
					// 		// v_after = 0;
					// 		// w_after = 0;
					// 		// tf2::Vector3 rotating_vector(v_after, w_after, 0);
					// 		// after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
					// 		// after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
					// 		// reward += FollowUserReward(v_follow, w_follow, after_x, after_y, follow_cost) * reward_discount * extra_discount;
					// 		// reward += M2G_FollowUserReward(joystick_heading, rotating_vector) * reward_discount * extra_discount;
					// 	}
					// }
					collision_reward = 0;
					reward += (ModelParams::step_penalty + collision_reward) * reward_discount;
					// follow user reward
					// if (!wheelchair_state.adaptability)
					// {
					// 	if (compute_user_reward)
					// 	{
					// 		user_reward = FollowUserReward(follow_heading, v_after, w_after, follow_cost);
					// 		reward += user_reward * reward_discount;
					// 		compute_user_reward = false;
					// 	}
					// }
					// no moving, so the reaching check is not needed

					v_after = 0;
					w_after = 0;
					v_before = v_after;
					w_before = w_after;

					i = i + steps2turn;
				}
			}

		}

		// cout << "Finish 15 steps of simulation" << endl;

		// follow user reward
		// if (!wheelchair_state.adaptability)
		// {
		// 	after_x = wheelchair_state.wheelchair.agent_pose.position.x - initial_x;
		// 	after_y = wheelchair_state.wheelchair.agent_pose.position.y - initial_y;
		// 	tf2::Vector3 M2G_vector(after_x, after_y, 0);

		// 	// cout << "initial_x = " << initial_x << " initial_y = " << initial_y << endl;

		// 	// cout << "final_x = " << wheelchair_state.wheelchair.agent_pose.position.x << " final_y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
			
		// 	// cout << "after_x = " << after_x << " after_y = " << after_y << endl;

		// 	// cout << "joystick_x = " << joystick_heading.getX() << " joystick_y = " << joystick_heading.getY() << endl;

		// 	// cout << "current_v = " << after_x << " after_y = " << after_y << endl;

		// 	// wheelchair_quat.setRPY(0, 0, - wheelchair_status.agent_pose_angle);
		// 	// joystick_heading.setX(initial_joy_x - wheelchair_status.agent_pose.position.x);
		// 	// joystick_heading.setY(initial_joy_y - wheelchair_status.agent_pose.position.y);
		// 	// joystick_heading = tf2::quatRotate(wheelchair_quat, joystick_heading);
		// 	// float joystick_length = joystick_heading.length();
		// 	// if (joystick_length > 1)
		// 	// {
		// 	// 	joystick_heading.setX(joystick_heading.getX() / joystick_length);
		// 	// 	joystick_heading.setY(joystick_heading.getY() / joystick_length);
		// 	// }
		// 	// wheelchair_state.joystick_x = round(joystick_heading.getX() * 100) / 100;
		// 	// wheelchair_state.joystick_y = round(joystick_heading.getY() * 100) / 100;

		// 	float M2G_vec_length = M2G_vector.length();

		// 	float base_ratio = ModelParams::planning_time * ModelParams::max_v_acceleration * ModelParams::step_scale;

		// 	if (M2G_vec_length > base_ratio)
		// 	{
		// 		M2G_vector.setX(after_x * base_ratio / M2G_vec_length);
		// 		M2G_vector.setY(after_y * base_ratio / M2G_vec_length);
		// 	}
			
		// 	user_reward = M2G_FollowUserReward(joystick_heading, M2G_vector);

		// 	// cout << "user_reward = " << user_reward << endl;
		// 	reward += user_reward * powf(ModelParams::reward_discount, total_steps2turn + transition_steps + 1); // num2simulate
		// }
		// Penalize the angular velocities that are different from the joystick input while turing in place for move to goal actions
		// if (external_joy_x == 0)
		// {
		// 	reward += fabs(wheelchair_state.joystick_y - w_after) * ModelParams::excess_speed_penalty / ModelParams::step_size;
		// }
		// cout << "Finish move to goal action." << endl;
		// cout << "Reward is " << reward << endl;

		// if (wheelchair_state.action_length == 0)
		// {
		// 	reward += ModelParams::inflation_max_penalty;
		// }
		return true;
	}
	}
	
	// Penalize the linear velocities that exceed the joystick input
	if (fabs(v_after) > max_linear_speed + 0.1)
	{
		reward += (1 + (fabs(v_after) - max_linear_speed) / linear_step_size) * ModelParams::excess_speed_penalty;
	}

	// add white noise to the current velocity
	// according to the log file, the velocity diff can be as high as 0.04, so the tolerance will be ±0.04
	if (ModelParams::using_probabilistic_model)
	{
		wheelchair_status.agent_velocity.linear.x += (rand_num - 0.5) * 2 * ModelParams::noise_amplitude;
		wheelchair_status.agent_velocity.angular.z += (rand_num - 0.5) * 2 * ModelParams::noise_amplitude;
	}
	
	// Before trainsition, add the current position to the path if the wheelchair moves (velocity is non-zero)
	if (v_before != 0 || v_after != 0)
	{
		if (action < total_normal_actions)	// only do it for normal actions
		{
			GenerateNewPath(wheelchair_status, wheelchair_state.path_traversed);
		}
		//for (int i = 0; i < ModelParams::num_paths; ++i)
		//{
			//GenerateNewPath(wheelchair_status, wheelchair_path.paths[i]);
			// ContractAndInterpolatePath(wheelchair_status, wheelchair_path.paths[i], map_quat);
		//}
	}

	// cout << "Reward after applying action: " << reward << endl;
	// Transit to the next state with a penalty_idx [0, 1] as a return value

	// cout << "action = " << action << endl;
	// cout << "normal v_before = " << v_before << ", v_after = " << v_after << endl;
	// cout << "normal w_before = " << w_before << ", w_after = " << w_after << endl;
	// cout << "normal v_follow = " << v_follow << ", w_follow = " << w_follow << endl;

	// cout << "follow_cost = " << follow_cost << endl;
	float transition_reward = TransitWheelchair(wheelchair_state, map_quaternion, transition_steps, v_before, v_after, w_before, w_after);
	reward += transition_reward;

	wheelchair_state.action_length++;
	wheelchair_state.action_before = action;
	// cout << "After changing atcion.." << endl;
	// cout << "wheelchair_state.action_length = " << wheelchair_state.action_length << ", wheelchair_state.action_before = " << wheelchair_state.action_before << endl;

	// if (transition_reward <= ModelParams::collision_penalty)
	// {
	// 	// a collision happened
	// 	return true;
	// }	

	// else penalty_idx == 0, transition succeeds with no collision, no basic penalty added

	// reward += ModelParams::collision_penalty * penalty_idx;

	// cout << "Reward after collision check: " << reward << endl;

	// Compute the reward after transition

	if (wheelchair_state.adaptability)	// follow the path
	{
		reward += ReachingCheck(wheelchair_state, reaching_goal) * powf(ModelParams::reward_discount, transition_steps - 1);
		// Reach the final goal
		if (reaching_goal)
		{
			return true;	// Only terminate the task when a final goal is reached
		}
	}
	else	// follow the user
	{
		if (ModelParams::using_reaching_check)
		{
			reward += InstantGoalReachingCheck(wheelchair_state, reaching_goal) * powf(ModelParams::reward_discount, transition_steps - 1);
			if (reaching_goal)
			{
				return true;	// Only terminate the task when a final goal is reached
			}
		}
		else
		{
			reward += FollowUserReward(follow_heading, v_after, w_after, follow_cost) * powf(ModelParams::reward_discount, transition_steps - 1);
		}
	}
	/* THE SHARED CONTROL PART */

	// if (!wheelchair_state.adaptability)	// not adapt to the robot
	// {
	// 	if (action == 6)
	// 	{
	// 		reward += ModelParams::user_following_reward;
	// 	}
	// 	else
	// 	{
	// 		reward += FollowUserReward(v_follow, w_follow, v_after, w_after);
	// 	}
	// }
	// cout << "Reward after reaching check: " << reward << endl;
	// Receding penalty

	// reward += RecedingPenalty(wheelchair_state);

	// cout << "Reward after receding penalty: " << reward << endl;

	// cout << "Reward after shared control: " << reward << endl;

	tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);

	// the current wheelchair heading
	agent_heading.setValue(1, 0, 0);
	agent_heading = tf2::quatRotate(wheelchair_quat, agent_heading);
	
	// Observation part
	wheelchair_obs.getObsFromWheelchairStatus(wheelchair_status);
	
	if (wheelchair_state.adaptability)
	{
		angle_diff = CalAngleDiff(wheelchair_status, agent_heading, goal_point_idx);

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
	}
	else
	{
		if (joystick_heading.length() < 0.1)
		{
			joystick_heading.setX(0.0001);
			joystick_heading.setY(0.0);
		}
		angle_diff = joystick_heading.angle(tf2::Vector3(1, 0, 0));
		angle_diff = (joystick_heading.getY() >= 0)? angle_diff : 2 * M_PI - angle_diff;
		int direction_index = round(angle_diff * 8 / M_PI);
		direction_index = direction_index == 16 ? 0 : direction_index;
		wheelchair_obs.joystick_obs = direction_index;
	}
	// Updates hashmap:
	// takes hash value as key and stores wheelchair_obs as value in map
	// First check if hash value is already stored
	PrintObs(wheelchair_obs, obs_string);
	// takes printed string to obs_string and converts to a hash value
	// uint64_t hashValue = obsHash(obs_string.str());
	// takes hashValue and updates value of obs in ObservationClass
	// wheelchair_obs.SetIntObs(hashValue);
	// obs = (OBS_TYPE) hashValue;
	obs = wheelchair_obs.GetHash();
	// if (obsHashMap.find(hashValue) == obsHashMap.end())
	// {
	// 	obsHashMap[hashValue] = wheelchair_obs;
	// }

	// cout << "action = " << action << ", reward = " << reward << endl;
	
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
	tf2::Quaternion wheelchair_quat;
	tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);

	// the current wheelchair heading
	tf2::Vector3 agent_heading(1, 0, 0);
	agent_heading = tf2::quatRotate(wheelchair_quat, agent_heading);

	// Use HashMap to get observation

	std::map<uint64_t, WheelchairObs>::iterator it = obsHashMap_execution.find(obs);
	WheelchairObs wheelchair_obs;
	if (it == obsHashMap_execution.end())
	{
		// it = obsHashMap.find(obs);
		// if (it == obsHashMap.end())
		// {
			// std::cout << "Calling ObsProb." << std::endl;
			// std::cout << "Observation not in hash map and execution hash map. This should not happen." << std::endl;
			// std::cout << "obs hash value " << obs << std::endl;
			// std::map<uint64_t, WheelchairObs>::iterator iter = obsHashMap.begin();
			// while(iter != obsHashMap.end())
			// {
			// 	std::cout << "obsHashMap value " << iter->first << std::endl;
			// 	iter++;
			// }
			
			// cout << "joystick: " << wheelchair_obs.joystick_obs << ", pos_x " << pos_x << ", pos_y " << pos_y << ", theta " << theta << ", vel_v " << vel_v << ", vel_w " << vel_w << endl;
			// cout << "PrintObs: " << obs_int << endl;

		wheelchair_obs.getObsFromInteger(obs);
		// }
		// std::cout << "Observation not in execution hash map. This should not happen." << std::endl;
	}	
	else
	{
		wheelchair_obs = it->second;
	}
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

	double angle_diff = CalAngleDiff(wheelchair_status, agent_heading, goal_point_idx);

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
	// const WheelchairState *wheelchair_start = static_cast<const WheelchairState*>(start);
	vector<State*> particles;
	Globals::config.action_values.clear();
	Globals::config.action_values.resize(NumActions(), 0);
	// Allocate() function allocates some space for creating new state;
	// Equal weights for each possible goal
	for (int i = 0; i < ModelParams::num_paths; i ++)
	{
		for (int j = 0; j < ModelParams::num_adapt; j++)
		{
			WheelchairState* goal = static_cast<WheelchairState*>(Allocate(j + i * ModelParams::num_adapt, 1.0 / (ModelParams::num_adapt * ModelParams::num_paths)));
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
			goal->adaptability = j;
			goal->wheelchair = current_wheelchair_status;
			goal->joystick_x = 0;
			goal->joystick_y = 0;
			// goal->collision_idx = wheelchair_start->collision_idx;
			goal->num_intermediate_goals = 0;
			goal->path2waypoint = intermediate_goal_list;
			goal->path_traversed = nav_msgs::Path();
			goal->action_length = 0;
			if (goal->adaptability == false)
			{
				goal->weight = goal->weight * 2 * 0.9;
			}
			else
			{
				goal->weight = goal->weight * 2 * 0.1;
			}
			particles.push_back(goal);
		}
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
	return ModelParams::reaching_reward;
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
	return ValuedAction(5, ModelParams::step_penalty - 1.2 * ModelParams::user_following_reward);
}

class WheelchairDSPOMDPPolicy: public DefaultPolicy
{
public:
	// enum
	// {
	// 	// action
	// 	LINEAR_PLUS = 0, LINEAR_MINUS = 1, ANGULAR_PLUS = 2, ANGULAR_MINUS = 3, KEEP = 4, STOP = 5, FOLLOW = 6
	// };
	WheelchairDSPOMDPPolicy(const DSPOMDP* model, ParticleLowerBound* bound):
	DefaultPolicy(model, bound)
	{}

	ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams, History& history) const
	{
		const WheelchairState *wheelchair_state_ = static_cast<const WheelchairState*>(particles[0]);
		float linear_v = wheelchair_state_->wheelchair.agent_velocity.linear.x;

		if (linear_v != 0)
			return 5;
		// if (round(linear_v * 100) / 100 > 0)
		// 	return LINEAR_MINUS;
		// else if (round(linear_v * 100) / 100 < 0)
		// 	return LINEAR_PLUS;
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
	out << "Goal position: x = " << goal_positions[wheelchair_state.path_idx].pose.position.x << "; y = " << goal_positions[wheelchair_state.path_idx].pose.position.y << endl;
	out << "Goal index = " << wheelchair_state.path_idx << endl;
}

void WheelchairDSPOMDP::PrintParticles(const std::vector<State*> particles, std::ostream& out) const
{
	// declare a counter to count particles follow each path and the user
	std::vector<int> sampled_particles;
	// size = number of paths + 1, the last "1" container is used to count particles following user
	sampled_particles.resize(ModelParams::num_paths + 1, 0); 
	for (int i = 0; i < particles.size(); ++i)
	{
		State* particle = particles[i];
		WheelchairState *wheelchair_particle = static_cast<WheelchairState*>(particle);

		// if following user, do not concider which path to follow, the last container++
		if (wheelchair_particle->adaptability == false)
		{
			sampled_particles[sampled_particles.size() - 1]++;
		}
		// if following path, add 1 to the corresponding container
		else
		{
			sampled_particles[wheelchair_particle->path_idx]++;
		}
	}
	for (int i = 0; i < sampled_particles.size(); ++i)
	{
		if (i == sampled_particles.size() - 1)	// last container, storing the number of particles following user
		{
			out << sampled_particles[i] << " paricles following user." << endl;
		}
		else	// not the last container, storing particles following each path
		{
			out << sampled_particles[i] << " paricles following path " << i + 1 << "." << endl;
		}
	}
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
		cout << "Goal " << i + 1 << ", position: x = " << this->goal_positions[i].pose.position.x << ", y = " << this->goal_positions[i].pose.position.y << ", belief: " << g_weight[i] << endl;
	}
}

void WheelchairDSPOMDP::PrintAction(ACT_TYPE action, ostream& out) const
{
	switch (action)
	{
	case 0:
	{
		out << "Linear acceleration" << endl;
		break;
	}
	case 1:
	{
		out << "Linear deceleration" << endl;
		break;
	}
	case 2:
	{
		out << "Angular acceleration" << endl;
		break;
	}
	case 3:
	{
		out << "Angular deceleration" << endl;
		break;
	}
	case 4:
	{
		out << "Keep" << endl;
		break;
	}
	case 5:
	{
		out << "Stop" << endl;
		break;
	}
	case 6:
	{
		out << "Follow" << endl;
		break;
	}
	case 7:
	{
		out << "Pure DWA" << endl;
		break;
	}
	case 8:
	{
		out << "Belief DWA" << endl;
		break;
	}
	default:
	{
		out << "Move to goal " << action - ModelParams::num_normal_actions - ModelParams::num_dwa_actions + 1 << endl;
		break;			
	}
	}
}

float WheelchairDSPOMDP::CollisionCheck(const WheelchairStruct& wheelchair_status, const tf2::Quaternion& map_quat) const
{
	// cout << "Collision checking..." << endl;
	float x_base = wheelchair_status.agent_pose.position.x;
	float y_base = wheelchair_status.agent_pose.position.y;

	// cout << "x_base = " << x_base << ", y_base = " << y_base << endl;

	// add an extra check point behind the wheelchair to solve the issue that base_link is not at the center of the wheelchair
	tf2::Quaternion wheelchair_quat;
	tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);
	tf2::Vector3 back2base(ModelParams::back2base_dist, 0, 0);
	// cout << "back2base x = " << back2base.getX() << ", y = " << back2base.getY() << endl;

	back2base = tf2::quatRotate(wheelchair_quat, back2base);

	// cout << "After rotation, back2base x = " << back2base.getX() << ", y = " << back2base.getY() << endl;

	float x_back = x_base - back2base.getX();
	float y_back = y_base - back2base.getY();

	// cout << "x_back = " << x_back << ", y_back = " << y_back << endl;


	// cout << "x_base in base_link frame: " << x_base << endl;
	// cout << "y_base in base_link frame: " << y_base << endl;

	// cout << "x_back in base_link frame: " << x_back << endl;
	// cout << "y_back in base_link frame: " << y_back << endl;

	// tf2::Matrix3x3 rotation_matrix;

	// rotation_matrix.setRPY(0, 0, -agent2map_yaw);
	// cout << "agent2map_yaw: " << agent2map_yaw * 180 / M_PI << endl;
	// cout << "map_quat x = " << map_quat.getX() << ", y = " << map_quat.getY() << ", z = " << map_quat.getZ() << ", w = " << map_quat.getW() << endl;

    tf2::Vector3 check_vector_base(x_base, y_base, 0);
	tf2::Vector3 check_vector_back(x_back, y_back, 0);
	check_vector_base = tf2::quatRotate(map_quat, check_vector_base);
	check_vector_back = tf2::quatRotate(map_quat, check_vector_back);
    // check_vector = rotation_matrix * check_vector;

	x_base = check_vector_base.getX();
    y_base = check_vector_base.getY();

	x_back = check_vector_back.getX();
    y_back = check_vector_back.getY();

	// cout << "x_base in costmap frame: " << x_base << endl;
	// cout << "y_base in costmap frame: " << y_base << endl;

	// cout << "x_back in costmap frame: " << x_back << endl;
	// cout << "y_back in costmap frame: " << y_back << endl;

	int col_base = x_base >= 0 ? ceil(x_base / map_resolution) : floor(x_base / map_resolution);
	int row_base = y_base >= 0 ? ceil(y_base / map_resolution) : floor(y_base / map_resolution);

	int col_back = x_back >= 0 ? ceil(x_back / map_resolution) : floor(x_back / map_resolution);
	int row_back = y_back >= 0 ? ceil(y_back / map_resolution) : floor(y_back / map_resolution);

	// cout << "col_base: " << col_base << endl;
	// cout << "row_base: " << row_base << endl;

	// cout << "col_back: " << col_back << endl;
	// cout << "row_back: " << row_back << endl;

	int base_pixel = local_costmap.at<uint8_t>(y_center + row_base, x_center + col_base);
	int back_pixel = local_costmap.at<uint8_t>(y_center + row_back, x_center + col_back);

	// cout << "base_pixel = " << base_pixel << ", back_pixel = " << back_pixel << endl;

	if (base_pixel >= ModelParams::outer_pixel_thres || back_pixel >= ModelParams::outer_pixel_thres)
	{
		return 1;
	}
	else if (base_pixel < ModelParams::inner_pixel_thres && back_pixel < ModelParams::inner_pixel_thres)
	{
		return 0;
	}
	else
	{
		if (base_pixel < ModelParams::inner_pixel_thres)
		{
			return 0.5 * (static_cast<float>(back_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
		}
		else if (back_pixel < ModelParams::inner_pixel_thres)
		{
			return 0.5 * (static_cast<float>(base_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
		}
		else
		{
			float double_collision = 0;
			double_collision = (static_cast<float>(base_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
			double_collision += (static_cast<float>(back_pixel) - ModelParams::inner_pixel_thres) / (ModelParams::outer_pixel_thres - ModelParams::inner_pixel_thres);
			return 0.5 * double_collision;
		}
	}
	// float x_check = wheelchair_status.agent_pose.position.x;
	// float y_check = wheelchair_status.agent_pose.position.y;
	// float r_collision = ModelParams::inner_radius;
	// float r_outer = ModelParams::outer_radius;
	// float temp_dist = 0;
	// float penalty_idx = 0;

	// for (const auto &point : lidar_points)
	// {
	// 	//https://stackoverflow.com/a/7227057
	// 	//Check through easy conditions first, getting distance with sqrt is computationally expensive
	// 	double dx = fabs(point.x - x_check);
	// 	double dy = fabs(point.y - y_check);

	// 	if (dx > r_outer || dy > r_outer)
	// 		continue;

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

	// 		if (temp_dist < r_outer)
	// 		{
	// 			penalty_idx = 1 - (temp_dist - r_collision) / (r_outer - r_collision);
	// 		}
	// 	}
	// }
	// return penalty_idx;
}

float WheelchairDSPOMDP::M2G_CollisionCheck(const WheelchairStruct& wheelchair_status, const tf2::Quaternion& map_quat, const float& angle2turn) const
{
	// cout << "Move to goal collision checking..." << endl;
	int collision_count = 0;
	float x_base = wheelchair_status.agent_pose.position.x;
	float y_base = wheelchair_status.agent_pose.position.y;

	// cout << "angle2turn = " << angle2turn * 180 / M_PI << endl;
	// cout << "x_base = " << x_base << ", y_base = " << y_base << endl;

	tf2::Quaternion wheelchair_quat;
	tf2::convert(wheelchair_status.agent_pose.orientation, wheelchair_quat);
	tf2::Vector3 back2base(ModelParams::back2base_dist, 0, 0);
	// cout << "back2base x = " << back2base.getX() << ", y = " << back2base.getY() << endl;

	back2base = tf2::quatRotate(wheelchair_quat, back2base);

	back2base = tf2::quatRotate(map_quat, back2base);

	// cout << "After map rotation, back2base x = " << back2base.getX() << ", y = " << back2base.getY() << endl;

	// first check the base link point

	tf2::Vector3 check_vector(x_base, y_base, 0);
	check_vector = tf2::quatRotate(map_quat, check_vector);

	x_base = check_vector.getX();
    y_base = check_vector.getY();

	// cout << "After map rotation, x_base = " << x_base << ", y_base = " << y_base << endl;

	int col_check = x_base >= 0 ? ceil(x_base / map_resolution) : floor(x_base / map_resolution);
	int row_check = y_base >= 0 ? ceil(y_base / map_resolution) : floor(y_base / map_resolution);

	int check_pixel = local_costmap.at<uint8_t>(y_center + row_check, x_center + col_check);

	if (check_pixel >= ModelParams::outer_pixel_thres)
	{
		collision_count++;
	}

	// cout << "Base pixel = " << check_pixel << endl;

	// x_base = wheelchair_status.agent_pose.position.x;
	// y_base = wheelchair_status.agent_pose.position.y;

	// then check the collision in the sector of angle2turn and do the collision check every PI/8

	int check_times = round(fabs(angle2turn) * 8 / M_PI);
	int rotation_sign = angle2turn >= 0 ? 1 : -1;

	// cout << "check_times = " << check_times << endl;

	// then check 8 points around the base_link that are back2base_dist away from the base_link that represent 8 directions
	tf2::Vector3 rotation_vector;
	tf2::Matrix3x3 rotation_matrix;
	float rotation_yaw = 0, x_check = 0, y_check = 0;
	for (int i = 0; i < check_times; ++i)
	{
		rotation_yaw = rotation_sign * (i + 1) * M_PI / 8;
		rotation_matrix.setRPY(0, 0, rotation_yaw);
		rotation_vector = rotation_matrix * back2base;

		// cout << "rotation_yaw = " << rotation_yaw * 180 / M_PI << endl;
		// cout << "rotation_vector x = " << rotation_vector.getX() << ", y = " << rotation_vector.getY() << endl;

		check_vector.setValue(x_base - rotation_vector.getX(), y_base - rotation_vector.getY(), 0);
		
		// cout << "check_vector x = " << check_vector.getX() << ", y = " << check_vector.getY() << endl;
		// check_vector = tf2::quatRotate(map_quat, check_vector);

		x_check = check_vector.getX();
		y_check = check_vector.getY();

		// cout << "After map rotation, check_vector x = " << x_check << ", y = " << y_check << endl;

		col_check = x_base >= 0 ? ceil(x_check / map_resolution) : floor(x_check / map_resolution);
		row_check = y_base >= 0 ? ceil(y_check / map_resolution) : floor(y_check / map_resolution);

		check_pixel = local_costmap.at<uint8_t>(y_center + row_check, x_center + col_check);

		// cout << "check_pixel pixel = " << check_pixel << endl;

		if (check_pixel >= ModelParams::outer_pixel_thres)
		{
			collision_count++;
		}

		// cout << "collision_count = " << collision_count << endl;
	}

	return collision_count * ModelParams::collision_penalty;
}

float WheelchairDSPOMDP::ReachingCheck(WheelchairState& wheelchair_state, bool& reaching_goal) const
{
	float dist2goal = 0, reward = 0, goal_yaw = 0;
	int n = 0;
	double agent_yaw = wheelchair_state.wheelchair.agent_pose_angle;
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

	if (n == intermediate_goal_list.paths[wheelchair_state.path_idx].poses.size())
	{
		reaching_goal = true;
		reward += ModelParams::reaching_reward;
	}
	else
	{
		reaching_goal = false;
	}

	if (wheelchair_state.num_intermediate_goals < n)
	{
		for (int i = wheelchair_state.num_intermediate_goals; i < n; ++i)
		{
			reward += (i + 1) * ModelParams::inter_goal_reward;
		}
	}
	wheelchair_state.num_intermediate_goals = n;

	return reward;
}

float WheelchairDSPOMDP::InstantGoalReachingCheck(WheelchairState& wheelchair_state, bool& reaching_goal) const
{
	float dist2goal = 0, reward = 0, goal_yaw = 0;
	int n = 0;
	double agent_yaw = wheelchair_state.wheelchair.agent_pose_angle;
	tf2::Quaternion goal_quat;
	for (int i = user_path.poses.size() - 1; i >= 0; --i)
	{
		dist2goal = sqrt(powf((wheelchair_state.wheelchair.agent_pose.position.x - user_path.poses[i].pose.position.x), 2) 
			+ powf((wheelchair_state.wheelchair.agent_pose.position.y - user_path.poses[i].pose.position.y), 2));
		tf2::convert(user_path.poses[i].pose.orientation, goal_quat);
		goal_yaw = tf2::getYaw(goal_quat);
		if (dist2goal <= 0.25 && fabs(goal_yaw - agent_yaw) <= M_PI / 6)
		{
			// check which intermediate goal can be reached
			n = i + 1;
			// wheelchair_state.state_goal_point = user_path.poses[i].pose.position;
			break;
		}
	}

	if (n == user_path.poses.size())
	{
		reaching_goal = true;
		reward += ModelParams::reaching_reward;
	}
	else
	{
		reaching_goal = false;
	}

	if (wheelchair_state.num_intermediate_goals < n)
	{
		for (int i = wheelchair_state.num_intermediate_goals; i < n; ++i)
		{
			reward += (i + 1) * ModelParams::inter_goal_reward;
		}
	}
	wheelchair_state.num_intermediate_goals = n;

	return reward;	
}

float WheelchairDSPOMDP::TransitWheelchair(WheelchairState& wheelchair_state, tf2::Quaternion& map_quat, int& transition_steps, float v_before, float v_after, float w_before, float w_after) const
{
	// cout << "v_before " << v_before << ", v_after " << v_after << endl;
	float reward = 0;
	transition_steps = 1;	// default number of transition steps, except for stop action

	// check if it requires more than 1 step to finish the velocity change
	int linear_steps = ceil(fabs(v_after - v_before) / (ModelParams::transition_time * ModelParams::max_v_acceleration));
	int angular_steps = ceil(fabs(w_after - w_before) / (ModelParams::transition_time * ModelParams::max_w_acceleration));
	if (linear_steps > 1 || angular_steps > 1)
		transition_steps = linear_steps > angular_steps ? linear_steps : angular_steps;

	// response time is 0.2s, total transition time is 0.3s, so use v,w before for the first half of response time, namely 0.1s, and v,w after for the rest 0.2s
	float yaw = wheelchair_state.wheelchair.agent_pose_angle;
	// float step_time = Globals::config.time_per_move;
	// int loop_times = round(transition_time / step_time);
	float penalty_idx = 0, highest_penalty = 0;
	float first_phase = 0.5 * ModelParams::repsonse_time;
	float second_phase = ModelParams::transition_time - first_phase;

	float v_temp = 0, w_temp = 0, v_step = 0, w_step = 0;

	if (transition_steps == 1)	// only requires 1 step to finish the velocity change
	{
		// the first phase, namely response phase, 0.1s
		if (fabs(w_before) < 0.1)
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_before * cos(yaw) * first_phase;
			wheelchair_state.wheelchair.agent_pose.position.y += v_before * sin(yaw) * first_phase;
		}
		else
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_before / w_before * (sin(yaw + w_before * first_phase) - sin(yaw));
			wheelchair_state.wheelchair.agent_pose.position.y += v_before / w_before * (cos(yaw) - cos(yaw + w_before * first_phase));
			yaw += w_before * first_phase;
		}
		// do the collision check
		// cout << "1 x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
		// cout << "1 v = " << v_before << ", w = " << w_before << endl;
		penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;

		// the second phase, 0.2s
		v_temp = 0.5 * (v_before + v_after);
		w_temp = 0.5 * (w_before + w_after);
		// cout << "v_before " << v_before << ", v_after " << v_after << endl;

		if (fabs(w_temp) < 0.1)
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_temp * cos(yaw) * second_phase;
			wheelchair_state.wheelchair.agent_pose.position.y += v_temp * sin(yaw) * second_phase;
		}
		else
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_temp / w_temp * (sin(yaw + w_temp * second_phase) - sin(yaw));
			wheelchair_state.wheelchair.agent_pose.position.y += v_temp / w_temp * (cos(yaw) - cos(yaw + w_temp * second_phase));
			yaw += w_temp * second_phase;
		}
		// do the collision check
		// cout << "2 x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
		// cout << "2 v = " << v_temp << ", w = " << w_temp << endl;
		penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
	}
	else	// requires multiple steps to finish the velocity change
	{
		float v_sign = v_after > v_before ? 1 : -1;
		float w_sign = w_after > w_before ? 1 : -1;
		// the first phase, namely response phase, 0.1s
		if (fabs(w_before) < 0.1)
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_before * cos(yaw) * first_phase;
			wheelchair_state.wheelchair.agent_pose.position.y += v_before * sin(yaw) * first_phase;
		}
		else
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_before / w_before * (sin(yaw + w_before * first_phase) - sin(yaw));
			wheelchair_state.wheelchair.agent_pose.position.y += v_before / w_before * (cos(yaw) - cos(yaw + w_before * first_phase));
			yaw += w_before * first_phase;
		}
		// do the collision check
		// cout << "3 x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
		// cout << "3 v = " << v_before << ", w = " << w_before << endl;
		penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		
		// the second phase, 0.2s
		// first check if v and w has been changed to v_after and w_after
		if (linear_steps > 1)
		{
			v_temp = 0.5 * (2 * v_before + ModelParams::transition_time * ModelParams::max_v_acceleration * v_sign);
			v_step = v_before + ModelParams::transition_time * ModelParams::max_v_acceleration * v_sign;
			linear_steps--;
		}
		else
		{
			v_temp = 0.5 * (v_before + v_after);
			v_step = v_after;
		}
		if (angular_steps > 1)
		{
			w_temp = 0.5 * (2 * w_before + ModelParams::transition_time * ModelParams::max_w_acceleration * w_sign);
			w_step = w_before + ModelParams::transition_time * ModelParams::max_w_acceleration * w_sign;
			angular_steps--;
		}
		else
		{
			w_temp = 0.5 * (w_before + w_after);
			w_step = w_after;
		}
		// transition for the second phase 0.2s
		if (fabs(w_temp) < 0.1)
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_temp * cos(yaw) * second_phase;
			wheelchair_state.wheelchair.agent_pose.position.y += v_temp * sin(yaw) * second_phase;
		}
		else
		{
			wheelchair_state.wheelchair.agent_pose.position.x += v_temp / w_temp * (sin(yaw + w_temp * second_phase) - sin(yaw));
			wheelchair_state.wheelchair.agent_pose.position.y += v_temp / w_temp * (cos(yaw) - cos(yaw + w_temp * second_phase));
			yaw += w_temp * second_phase;
		}
		// do the collision check
		// cout << "4 x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
		// cout << "4 v = " << v_temp << ", w = " << w_temp << endl;
		penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

		if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		
		// after the first transition step, finish the rest steps without response time
		for (int i = 0; i < transition_steps - 1; ++i)
		{
			if (linear_steps > 1)
			{
				v_temp = 0.5 * (2 * v_step + ModelParams::transition_time * ModelParams::max_v_acceleration * v_sign);
				v_step = v_step + ModelParams::transition_time * ModelParams::max_v_acceleration * v_sign;
				linear_steps--;
			}
			else
			{
				v_temp = 0.5 * (v_step + v_after);
				v_step = v_after;
			}
			if (angular_steps > 1)
			{
				w_temp = 0.5 * (2 * w_step + ModelParams::transition_time * ModelParams::max_w_acceleration * w_sign);
				w_step = w_step + ModelParams::transition_time * ModelParams::max_w_acceleration * w_sign;
				angular_steps--;
			}
			else
			{
				w_temp = 0.5 * (w_temp + w_after);
				w_step = w_after;
			}
			// transition
			if (fabs(w_temp) < 0.1)
			{
				wheelchair_state.wheelchair.agent_pose.position.x += v_temp * cos(yaw) * second_phase;
				wheelchair_state.wheelchair.agent_pose.position.y += v_temp * sin(yaw) * second_phase;
			}
			else
			{
				wheelchair_state.wheelchair.agent_pose.position.x += v_temp / w_temp * (sin(yaw + w_temp * second_phase) - sin(yaw));
				wheelchair_state.wheelchair.agent_pose.position.y += v_temp / w_temp * (cos(yaw) - cos(yaw + w_temp * second_phase));
				yaw += w_temp * second_phase;
			}
			// do the collision check
			// cout << "5 x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;
			// cout << "5 v = " << v_temp << ", w = " << w_temp << endl;
			penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

			if (penalty_idx > highest_penalty)
			highest_penalty = penalty_idx;
		}
	}

	// finish 2 phases
	// wheelchair_state.collision_idx = highest_penalty;
	
	// for (int i = 0; i < loop_times; i ++)
	// {
	// 	penalty_idx = CollisionCheck(wheelchair_state.wheelchair);
	// 	if (penalty_idx == 1)
	// 	{
	// 		return penalty_idx;	// A severe collision happens, transition fails, the task is terminated.
	// 	}
	// 	if (wheelchair_state.wheelchair.agent_velocity.angular.z == 0)
	// 	{
	// 		wheelchair_state.wheelchair.agent_pose.position.x += wheelchair_state.wheelchair.agent_velocity.linear.x * cos(yaw) * step_time;
	// 		wheelchair_state.wheelchair.agent_pose.position.y += wheelchair_state.wheelchair.agent_velocity.linear.x * sin(yaw) * step_time;
	// 	}
	// 	else
	// 	{
	// 		wheelchair_state.wheelchair.agent_pose.position.x += wheelchair_state.wheelchair.agent_velocity.linear.x / wheelchair_state.wheelchair.agent_velocity.angular.z * 
	// 			(sin(yaw + wheelchair_state.wheelchair.agent_velocity.angular.z * step_time) - sin(yaw));
	// 		wheelchair_state.wheelchair.agent_pose.position.y += wheelchair_state.wheelchair.agent_velocity.linear.x / wheelchair_state.wheelchair.agent_velocity.angular.z * 
	// 			(cos(yaw) - cos(yaw + wheelchair_state.wheelchair.agent_velocity.angular.z * step_time));
	// 		yaw += wheelchair_state.wheelchair.agent_velocity.angular.z * step_time;
	// 	}
	// 	accumulative_penalty += penalty_idx;
	// }
	// penalty_idx = accumulative_penalty / static_cast<float>(loop_times);

	wheelchair_state.wheelchair.agent_velocity.linear.x = v_after;
	wheelchair_state.wheelchair.agent_velocity.angular.z = w_after;
	
	// Update quaternion message in state
	tf2::Quaternion wheelchair_quat;
	wheelchair_quat.setRPY(0, 0, yaw);
	tf2::convert(wheelchair_quat, wheelchair_state.wheelchair.agent_pose.orientation);

	/*	Update joystick input, since joystick input direction keeps unchanged during the search,
		when the wheelchair rotates for an angle A, the joystick direction rotates for -A w.r.t. the wheelchair frame */

	if (initial_joystick_length < 0.1)	// no joystick input
	{
		wheelchair_state.joystick_x = 0;
		wheelchair_state.joystick_y = 0;
	}
	else
	{
		tf2::Vector3 previous_joystick(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
		wheelchair_quat.setRPY(0, 0, -yaw);
		tf2::Vector3 joystick_heading(0, 0, 0);
		joystick_heading.setX(initial_joy_x * ModelParams::waypoint_dist - wheelchair_state.wheelchair.agent_pose.position.x);
		joystick_heading.setY(initial_joy_y * ModelParams::waypoint_dist - wheelchair_state.wheelchair.agent_pose.position.y);
		float joystick_length = joystick_heading.length();

		if (joystick_length > 0)
		{
			joystick_heading = tf2::quatRotate(wheelchair_quat, joystick_heading);

			// joystick_heading.setX(joystick_heading.getX() / ModelParams::waypoint_dist);
			// joystick_heading.setY(joystick_heading.getY() / ModelParams::waypoint_dist);			
			float angle_diff = previous_joystick.angle(joystick_heading);

			if (angle_diff >= 0.5 * M_PI)    // dummy goal exceeded, only keep the direction now
			{
				wheelchair_quat.setRPY(0, 0, -(yaw - wheelchair_state.wheelchair.agent_pose_angle));
				joystick_heading = tf2::quatRotate(wheelchair_quat, previous_joystick);				
			}
			else
			{
				// scale joystick input to the original length
				joystick_heading.setX(joystick_heading.getX() * initial_joystick_length / joystick_length);
				joystick_heading.setY(joystick_heading.getY() * initial_joystick_length / joystick_length);
			}
		}
		else	// dummy goal reached, only keep the direction now
		{
			wheelchair_quat.setRPY(0, 0, -(yaw - wheelchair_state.wheelchair.agent_pose_angle));
			joystick_heading = tf2::quatRotate(wheelchair_quat, previous_joystick);
		}

		wheelchair_state.joystick_x = round(joystick_heading.getX() * 100) / 100;
		wheelchair_state.joystick_y = round(joystick_heading.getY() * 100) / 100;		
	}

	// Update agent_pose_angle in state
	wheelchair_state.wheelchair.agent_pose_angle = yaw;

	// compute the reward

	if (highest_penalty == 1) // A severe collision happens
	{
		// collision penalty plus an extra penalty based on the current linear velocity
		// reward = ModelParams::collision_penalty * (1 + fabs(wheelchair_state.wheelchair.agent_velocity.linear.x));
		// return true;	// Only terminate the task when a severe collision occurs
		// basic penalty for mild collision plus an extra penalty based on the current linear velocity
		if (fabs(wheelchair_state.wheelchair.agent_velocity.linear.x) > 0.25)
		{
			reward = ModelParams::collision_penalty * (1 + fabs(wheelchair_state.wheelchair.agent_velocity.linear.x));
		}
		else
		{
			reward = ModelParams::collision_penalty;
			// reward += (ModelParams::inflation_high_penalty - ModelParams::inflation_basic_penalty) * penalty_idx + ModelParams::inflation_basic_penalty;
			// reward += (ModelParams::inflation_max_penalty - ModelParams::inflation_basic_penalty) * fabs(wheelchair_status.agent_velocity.linear.x)
			// 	+ ModelParams::inflation_basic_penalty;	
		}
	}
	else if (highest_penalty != 0)	// Transition succeeds with mild collision
	{
		// basic penalty for mild collision plus an extra penalty based on the current linear velocity
		if (fabs(wheelchair_state.wheelchair.agent_velocity.linear.x) > 0.25)
		{
			reward = 0.5 * ModelParams::inflation_max_penalty * (1 + highest_penalty) * fabs(wheelchair_state.wheelchair.agent_velocity.linear.x) / 0.25;
		}
		else
		{
			reward = ModelParams::inflation_basic_penalty * (1 + highest_penalty);
			// reward += (ModelParams::inflation_high_penalty - ModelParams::inflation_basic_penalty) * penalty_idx + ModelParams::inflation_basic_penalty;
			// reward += (ModelParams::inflation_max_penalty - ModelParams::inflation_basic_penalty) * fabs(wheelchair_status.agent_velocity.linear.x)
			// 	+ ModelParams::inflation_basic_penalty;	
		}
	}
	
	return reward;			// return the reward for transition
}

bool WheelchairDSPOMDP::TransitParticles(WheelchairState& wheelchair_state, tf2::Quaternion& map_quat, float v_before, float v_after, float w_before, float w_after) const
{
	// check if it requires more than 1 step to finish the velocity change
	// if (fabs(v_after - v_before) > ModelParams::transition_time * ModelParams::max_v_acceleration)
	// {
	// 	if (v_after > v_before)
	// 	{
	// 		v_after = v_before + ModelParams::transition_time * ModelParams::max_v_acceleration;
	// 	}
	// 	else
	// 	{
	// 		v_after = v_before - ModelParams::transition_time * ModelParams::max_v_acceleration;
	// 	}
	// }
	// if (fabs(w_after - w_before) > ModelParams::transition_time * ModelParams::max_w_acceleration)
	// {
	// 	if (w_after > w_before)
	// 	{
	// 		w_after = w_before + ModelParams::transition_time * ModelParams::max_w_acceleration;
	// 	}
	// 	else
	// 	{
	// 		w_after = w_before - ModelParams::transition_time * ModelParams::max_w_acceleration;
	// 	}
	// }
	bool particle_moved = false;
	if (v_before != 0 || v_after != 0)
	{
		GenerateNewPath(wheelchair_state.wheelchair, wheelchair_state.path_traversed);
		particle_moved = true;
	}

	// response time is 0.2s, total transition time is 0.3s, so use v,w before for the first half of response time, namely 0.1s, and v,w after for the rest 0.2s
	float yaw = wheelchair_state.wheelchair.agent_pose_angle;
	float penalty_idx = 0, highest_penalty = 0;
	float first_phase = 0.5 * ModelParams::repsonse_time;
	float second_phase = ModelParams::planning_time - first_phase;

	float v_temp = 0, w_temp = 0;

	// the first phase, namely response phase, 0.1s
	if (fabs(w_before) < 0.1)
	{
		wheelchair_state.wheelchair.agent_pose.position.x += v_before * cos(yaw) * first_phase;
		wheelchair_state.wheelchair.agent_pose.position.y += v_before * sin(yaw) * first_phase;
	}
	else
	{
		wheelchair_state.wheelchair.agent_pose.position.x += v_before / w_before * (sin(yaw + w_before * first_phase) - sin(yaw));
		wheelchair_state.wheelchair.agent_pose.position.y += v_before / w_before * (cos(yaw) - cos(yaw + w_before * first_phase));
		yaw += w_before * first_phase;
	}
	// do the collision check
	penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

	if (penalty_idx > highest_penalty)
		highest_penalty = penalty_idx;

	// the second phase, 0.2s
	v_temp = 0.5 * (v_before + v_after);
	w_temp = 0.5 * (w_before + w_after);

	if (fabs(w_temp) < 0.1)
	{
		wheelchair_state.wheelchair.agent_pose.position.x += v_temp * cos(yaw) * second_phase;
		wheelchair_state.wheelchair.agent_pose.position.y += v_temp * sin(yaw) * second_phase;
	}
	else
	{
		wheelchair_state.wheelchair.agent_pose.position.x += v_temp / w_temp * (sin(yaw + w_temp * second_phase) - sin(yaw));
		wheelchair_state.wheelchair.agent_pose.position.y += v_temp / w_temp * (cos(yaw) - cos(yaw + w_temp * second_phase));
		yaw += w_temp * second_phase;
	}
	// do the collision check
	penalty_idx = CollisionCheck(wheelchair_state.wheelchair, map_quat);

	if (penalty_idx > highest_penalty)
		highest_penalty = penalty_idx;

	wheelchair_state.wheelchair.agent_velocity.linear.x = v_after;
	wheelchair_state.wheelchair.agent_velocity.angular.z = w_after;

	// cout << "Original path, size = "<< intermediate_goal_list.paths[0].poses.size() << endl;
	// for (int i = 0; i < intermediate_goal_list.paths[0].poses.size(); ++i)
	// {
	// 	cout << intermediate_goal_list.paths[0].poses[i].pose.position.x << " "
	// 		<< intermediate_goal_list.paths[0].poses[i].pose.position.y << endl;
	// }
	// cout << "Before contraction, wheelchair path size = "<< wheelchair_state.path2waypoint.paths[0].poses.size() << endl;
	
	// for (int i = 0; i < wheelchair_state.path_traversed.poses.size(); ++i)
	// {
	// 	cout << wheelchair_state.path_traversed.poses[i].pose.position.x << " "
	// 		<< wheelchair_state.path_traversed.poses[i].pose.position.y << endl;
	// }
	// for (int i = 0; i < wheelchair_state.path2waypoint.paths[0].poses.size(); ++i)
	// {
	// 	cout << wheelchair_state.path2waypoint.paths[0].poses[i].pose.position.x << " "
	// 		<< wheelchair_state.path2waypoint.paths[0].poses[i].pose.position.y << endl;
	// }

	if (particle_moved)
	{
		for (int i = 0; i < goal_positions.size(); ++i)
		{
			nav_msgs::Path temp_path_traversed = wheelchair_state.path_traversed;
			ContractAndInterpolatePath(wheelchair_state.wheelchair, temp_path_traversed, wheelchair_state.path2waypoint.paths[i], map_quat);
		}
		wheelchair_state.path_traversed.poses.clear();
	}
	// cout << "After contraction..." << endl;
	// for (int i = 0; i < wheelchair_state.path2waypoint.paths.size(); ++i)
	// {
	// 	cout << "Path " << i + 1  << "..." << endl;
	// 	for (int j = 0; j < wheelchair_state.path2waypoint.paths[i].poses.size(); ++j)
	// 	{
	// 		cout << wheelchair_state.path2waypoint.paths[i].poses[j].pose.position.x << " "
	// 			<< wheelchair_state.path2waypoint.paths[i].poses[j].pose.position.y << endl;
	// 	}
	// }
	
	// Update quaternion message in state
	tf2::Quaternion wheelchair_quat;
	wheelchair_quat.setRPY(0, 0, yaw);
	tf2::convert(wheelchair_quat, wheelchair_state.wheelchair.agent_pose.orientation);

	// updated_heading.setValue(1, 0, 0);
	// updated_heading = tf2::quatRotate(wheelchair_quat, updated_heading);

	/*	Update joystick input, since joystick input direction keeps unchanged during the search,
		when the wheelchair rotates for an angle A, the joystick direction rotates for -A w.r.t. the wheelchair frame */
	// cout << "Before rotation in Update, joystick x = " << external_joy_x << ", y = " << external_joy_y << endl;
	// wheelchair_quat.setRPY(0, 0, -yaw);
	tf2::Vector3 joystick_heading(wheelchair_state.joystick_x, wheelchair_state.joystick_y, 0);
	// if (fabs(external_joy_x) > 0.1)
	// {
	// 	joystick_heading.setX(external_joy_x * ModelParams::waypoint_dist - wheelchair_state.wheelchair.agent_pose.position.x);
	// 	joystick_heading.setY(external_joy_y * ModelParams::waypoint_dist - wheelchair_state.wheelchair.agent_pose.position.y);
	// 	// cout << "Before rotation in Update, joystick x = " << joystick_heading.getX() << ", y = " << joystick_heading.getY() << endl;
	// 	joystick_heading = tf2::quatRotate(wheelchair_quat, joystick_heading);

	// 	// cout << "Rotation angle in Update = " << yaw * 180 / M_PI << endl;
	// 	// cout << "After rotation in Update, joystick x = " << joystick_heading.getX()  << ", y = " << joystick_heading.getY() << endl;
	// 	joystick_heading.setX(joystick_heading.getX() / ModelParams::waypoint_dist);
	// 	joystick_heading.setY(joystick_heading.getY() / ModelParams::waypoint_dist);

	// 	joystick_heading.setX(joystick_heading.getX() / joystick_heading.getX() * fabs(external_joy_x));
	// 	joystick_heading.setY(joystick_heading.getY() / joystick_heading.getX() * fabs(external_joy_x));

	// 	// scale joystick input down if it exceeds the circle area
	// 	// float joystick_length = joystick_heading.length();
	// 	// if (joystick_length > 1)
	// 	// {
	// 	// 	wheelchair_state.joystick_x = joystick_heading.getX() / joystick_length;
	// 	// 	wheelchair_state.joystick_y = joystick_heading.getY() / joystick_length;
	// 	// }
	// }
	// else
	// {
	// 	joystick_heading.setX(external_joy_x);
	// 	joystick_heading.setY(external_joy_y);
	// }
	
	
	// wheelchair_state.joystick_x = round(joystick_heading.getX() * 100) / 100;
	// wheelchair_state.joystick_y = round(joystick_heading.getY() * 100) / 100;

	initial_joy_x = wheelchair_state.joystick_x;
	initial_joy_y = wheelchair_state.joystick_y;

	initial_joystick_length = powf(initial_joy_x, 2) + powf(initial_joy_y, 2);
	initial_joystick_length = sqrt(initial_joystick_length);
	// updated_position_x = wheelchair_state.wheelchair.agent_pose.position.x;
	// updated_position_y = wheelchair_state.wheelchair.agent_pose.position.y;
	cout << "After step particles once in Update, joystick x = " << wheelchair_state.joystick_x << ", y = " << wheelchair_state.joystick_y << endl;
	cout << "After step particles once in Update, velocity v = " << wheelchair_state.wheelchair.agent_velocity.linear.x << ", w = " << wheelchair_state.wheelchair.agent_velocity.angular.z << endl;
	cout << "After step particles once in Update, position x = " << wheelchair_state.wheelchair.agent_pose.position.x << ", y = " << wheelchair_state.wheelchair.agent_pose.position.y << endl;

	float angle_diff = joystick_heading.angle(tf2::Vector3(1, 0, 0));
	angle_diff = (joystick_heading.getY() >= 0)? angle_diff : 2 * M_PI - angle_diff;
	int direction_index = round(angle_diff * 8 / M_PI);
	direction_index = direction_index == 16 ? 0 : direction_index;
	float lower_range = direction_index * 22.5 - 11.25;
	float upper_range = direction_index * 22.5 + 11.25;	
	cout << "After step particles once in Update, joystick input is " << direction_index << ", range is " << lower_range << " ~ " << upper_range << endl;

	cout << "After step particles once in Update, pure DWA v = " << pure_dwa_v << ", w = " << pure_dwa_w << endl;
	cout << "After step particles once in Update, belief DWA v = " << belief_dwa_v << ", w = " << belief_dwa_w << endl;
	// Update agent_pose_angle in state
	wheelchair_state.wheelchair.agent_pose_angle = yaw;

	if (highest_penalty < 1)
	{
		return false;
	}
	else
	{
		return true;
	}
}

int WheelchairDSPOMDP::RecedingPenalty(WheelchairState& wheelchair_state, tf2::Vector3& agent_heading) const
{
	int receding_penalty = 0, penalty_scale = 0, penalty_norm = 0;
	double angle_diff = CalAngleDiff(wheelchair_state.wheelchair, agent_heading, wheelchair_state.path_idx, ModelParams::num_points_direction);
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

float WheelchairDSPOMDP::FollowingVel(tf2::Vector3 joystick_heading, float v_before, float w_before, float& v_follow, float& w_follow, float v_max, float time_span) const
{
	// follow the user action
	float v_mapped = joystick_heading.getX();
	float w_mapped = joystick_heading.getY();
	float v_upper = 0, v_lower = 0, w_upper = 0, w_lower = 0, v_scale = 0, w_scale = 0;
	v_upper = v_before + ModelParams::max_v_acceleration * time_span * ModelParams::step_scale;

	v_lower = v_before - ModelParams::max_v_acceleration * time_span * ModelParams::step_scale;

	w_upper = w_before + ModelParams::max_w_acceleration * time_span * ModelParams::step_scale;

	w_lower = w_before - ModelParams::max_w_acceleration * time_span * ModelParams::step_scale;

	w_upper = std::min(w_upper, ModelParams::max_angular_speed);
	w_lower = std::max(w_lower, - ModelParams::max_angular_speed);

	if (fabs(v_mapped) < 0.01)	// no linear input
	{
		v_mapped = 0.01;
		joystick_heading.setX(0.01);
	}
	if (v_mapped > 0)	// joystick forward
	{
		if (v_before >= 0)	// moving forward
		{
			v_lower = std::max(v_lower, float(0.0));
			v_upper = std::min(v_upper, std::max(v_max, v_lower));
		}
		else	// moving backward (opposite)
		{
			// stop the wheelchair first
			v_follow = v_upper >= 0 ? 0 : v_upper;
			if (w_upper * w_lower <= 0)	// w range covers 0
			{
				w_follow = 0;
			}
			else	// w range doesn't cover 0
			{
				w_follow = w_upper > 0 ? w_lower : w_upper;
			}
			return 1;
		}
	}
	else	// joystick backward
	{
		if (v_before <= 0)	// moving backward
		{
			v_upper = std::min(v_upper, float(0.0));
			v_lower = std::max(v_lower, std::min(- v_max, v_upper));
		}
		else	// moving forward (opposite)
		{
			// stop the wheelchair first
			v_follow = v_lower <= 0 ? 0 : v_lower;
			if (w_upper * w_lower <= 0)	// w range covers 0
			{
				w_follow = 0;
			}
			else	// w range doesn't cover 0
			{
				w_follow = w_upper > 0 ? w_lower : w_upper;
			}
			return 1;
		}		
	}

	// if (print_out)
	// {
		// cout << "v_upper = " << v_upper << ", v_lower = " << v_lower << endl;
		// cout << "w_upper = " << w_upper << ", w_lower = " << w_lower << endl;
	// }

	if (fabs(v_upper) < 0.1 && fabs(v_lower) < 0.1)	// pure turning
	{
		v_follow = 0;
		if (w_mapped > w_upper)	// w is greater than the DW
		{
			w_follow = w_upper;
		}
		else if (w_mapped < w_lower)	// w is smaller than the DW
		{
			w_follow = w_lower;
		}
		else	// w is inside the DW
		{
			w_follow = w_mapped;
		}
	}
	else	// not turning in place
	{
		tf2::Vector3 upper_left(v_upper, w_upper, 0), upper_right(v_upper, w_lower, 0), lower_left(v_lower, w_upper, 0), lower_right(v_lower, w_lower, 0);

		if (fabs(v_upper) < 0.01 && fabs(w_upper) < 0.01)
		{
			if (v_mapped > 0)
			{
				upper_left.setX(0.01);
			}
			else
			{
				upper_left.setX(-0.01);
			}
			upper_left.setY(0.0);
		}
		if (fabs(v_upper) < 0.01 && fabs(w_lower) < 0.01)
		{
			if (v_mapped > 0)
			{
				upper_right.setX(0.01);
			}
			else
			{
				upper_right.setX(-0.01);
			}
			upper_right.setY(0.0);
		}
		if (fabs(v_lower) < 0.01 && fabs(w_upper) < 0.01)
		{
			if (v_mapped > 0)
			{
				lower_left.setX(0.01);
			}
			else
			{
				lower_left.setX(-0.01);
			}
			lower_left.setY(0.0);
		}
		if (fabs(v_lower) < 0.01 && fabs(w_lower) < 0.01)
		{
			if (v_mapped > 0)
			{
				lower_right.setX(0.01);
			}
			else
			{
				lower_right.setX(-0.01);
			}
			lower_right.setY(0.0);
		}

		float upper_left_angle = upper_left.angle(tf2::Vector3(1, 0, 0));
		upper_left_angle = upper_left.getY() >= 0 ? upper_left_angle : - upper_left_angle;

		float upper_right_angle = upper_right.angle(tf2::Vector3(1, 0, 0));
		upper_right_angle = upper_right.getY() >= 0 ? upper_right_angle : - upper_right_angle;

		float lower_left_angle = lower_left.angle(tf2::Vector3(1, 0, 0));
		lower_left_angle = lower_left.getY() >= 0 ? lower_left_angle : - lower_left_angle;

		float lower_right_angle = lower_right.angle(tf2::Vector3(1, 0, 0));
		lower_right_angle = lower_right.getY() >= 0 ? lower_right_angle : - lower_right_angle;
		
		float joystick_angle = joystick_heading.angle(tf2::Vector3(1, 0, 0));
		joystick_angle = joystick_heading.getY() >= 0 ? joystick_angle : - joystick_angle;

		// if (print_out)
		// {
			// cout << "v_mapped = " << v_mapped << ", w_mapped = " << w_mapped << endl;
			// cout << "upper_left_angle = " << upper_left_angle << ", upper_right_angle = " << upper_right_angle << endl;
			// cout << "lower_left_angle = " << lower_left_angle << ", lower_right_angle = " << lower_right_angle << endl;
			// cout << "joystick_angle = " << joystick_angle << endl;
		// }
		if (std::isnan(joystick_angle))
		{
			cout << "Nan joystick angle" << endl;
		}

		if (v_mapped > 0)	// moving forward
		{
			if (w_lower > 0)	// DW on the left side
			{
				if (joystick_angle > lower_left_angle)	// joystick direction outside the DW and the lower left corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_upper;
				}
				else if (joystick_angle < upper_right_angle)	// joystick direction outside the DW and the upper right corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_lower;
				}
				else	// joystick direction inside the DW
				{
					if (upper_left_angle >= lower_right_angle)	// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					{
						if (joystick_angle >= upper_left_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle >= lower_right_angle)	// mid up-down trapezoid
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
					else	// partition the DW into 3 parts, left and right triangles, mid left-right trapezoid
					{
						if (joystick_angle >= lower_right_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle >= upper_left_angle)	// mid left-right trapezoid
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
				}
			}
			else if (w_upper < 0)	// DW on the right side
			{
				if (joystick_angle > upper_left_angle)	// joystick direction outside the DW and the upper left corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_upper;
				}
				else if (joystick_angle < lower_right_angle)	// joystick direction outside the DW and the lower right corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_lower;
				}
				else	// joystick direction inside the DW
				{
					if (lower_left_angle >= upper_right_angle)	// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					{
						if (joystick_angle >= lower_left_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle >= upper_right_angle)	// mid up-down trapezoid
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
					else	// partition the DW into 3 parts, left and right triangles, mid left-right trapezoid
					{
						if (joystick_angle >= upper_right_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle >= lower_left_angle)	// mid left-right trapezoid
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
				}
			}
			else	// DW covers v axis
			{
				if (joystick_angle > lower_left_angle)	// joystick direction outside the DW and the lower left corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_upper;
				}
				else if (joystick_angle < lower_right_angle)	// joystick direction outside the DW and the lower right corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_lower;
				}
				else	// joystick direction inside the DW
				{
					// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					if (joystick_angle >= upper_left_angle)	// left triangle
					{
						if (w_mapped > w_upper)
						{
							w_follow = w_upper;
							v_follow = v_mapped * w_upper / w_mapped;
						}
						else if (v_mapped < v_lower)
						{
							v_follow = v_lower;
							w_follow = w_mapped * v_lower / v_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
					else if (joystick_angle >= upper_right_angle)	// mid up-down trapezoid
					{
						if (v_mapped > v_upper)
						{
							v_follow = v_upper;
							w_follow = w_mapped * v_upper / v_mapped;
						}
						else if (v_mapped < v_lower)
						{
							v_follow = v_lower;
							w_follow = w_mapped * v_lower / v_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
					else	// right triangle
					{
						if (v_mapped < v_lower)
						{
							v_follow = v_lower;
							w_follow = w_mapped * v_lower / v_mapped;
						}
						else if (w_mapped < w_lower)
						{
							w_follow = w_lower;
							v_follow = v_mapped * w_lower / w_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
				}
			}
		}
		else	// moving backward
		{
			if (w_lower > 0)	// DW on the left side
			{
				if (joystick_angle < upper_left_angle)	// joystick direction outside the DW and the upper left corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_upper;
				}
				else if (joystick_angle > lower_right_angle)	// joystick direction outside the DW and the lower right corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_lower;
				}
				else	// joystick direction inside the DW
				{
					if (lower_left_angle >= upper_right_angle)	// partition the DW into 3 parts, left and right triangles, mid left-right trapezoid
					{
						if (joystick_angle <= upper_right_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle <= lower_left_angle)	// mid left-right trapezoid
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
					else	// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					{
						if (joystick_angle <= lower_left_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle <= upper_right_angle)	// mid up-down trapezoid
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
				}
			}
			else if (w_upper < 0)	// DW on the right side
			{
				if (joystick_angle > upper_right_angle)	// joystick direction outside the DW and the upper right corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_lower;
				}
				else if (joystick_angle < lower_left_angle)	// joystick direction outside the DW and the lower left corner is the nearest
				{
					v_follow = v_lower;
					w_follow = w_upper;
				}
				else	// joystick direction inside the DW
				{
					if (upper_left_angle >= lower_right_angle)	// partition the DW into 3 parts, left and right triangles, mid left-right trapezoid
					{
						if (joystick_angle <= lower_right_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle <= upper_left_angle)	// mid left-right trapezoid
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
					else	// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					{
						if (joystick_angle <= upper_left_angle)	// left triangle
						{
							if (w_mapped > w_upper)
							{
								w_follow = w_upper;
								v_follow = v_mapped * w_upper / w_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else if (joystick_angle <= lower_right_angle)	// mid up-down trapezoid
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (v_mapped < v_lower)
							{
								v_follow = v_lower;
								w_follow = w_mapped * v_lower / v_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
						else	// right triangle
						{
							if (v_mapped > v_upper)
							{
								v_follow = v_upper;
								w_follow = w_mapped * v_upper / v_mapped;
							}
							else if (w_mapped < w_lower)
							{
								w_follow = w_lower;
								v_follow = v_mapped * w_lower / w_mapped;
							}
							else
							{
								v_follow = v_mapped;
								w_follow = w_mapped;
							}
						}
					}
				}
			}
			else	// DW covers v axis
			{
				upper_left_angle = upper_left_angle >= 0 ? upper_left_angle : upper_left_angle + 2 * M_PI;
				upper_right_angle = upper_right_angle >= 0 ? upper_right_angle : upper_right_angle + 2 * M_PI;
				lower_left_angle = lower_left_angle >= 0 ? lower_left_angle : lower_left_angle + 2 * M_PI;
				lower_right_angle = lower_right_angle >= 0 ? lower_right_angle : lower_right_angle + 2 * M_PI;
				joystick_angle = joystick_angle >= 0 ? joystick_angle : joystick_angle + 2 * M_PI;

				if (joystick_angle < upper_left_angle)	// joystick direction outside the DW and the upper left corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_upper;
				}
				else if (joystick_angle > upper_right_angle)	// joystick direction outside the DW and the upper right corner is the nearest
				{
					v_follow = v_upper;
					w_follow = w_lower;
				}
				else	// joystick direction inside the DW
				{
					// partition the DW into 3 parts, left and right triangles, mid up-down trapezoid
					if (joystick_angle <= lower_left_angle)	// left triangle
					{
						if (w_mapped > w_upper)
						{
							w_follow = w_upper;
							v_follow = v_mapped * w_upper / w_mapped;
						}
						else if (v_mapped > v_upper)
						{
							v_follow = v_upper;
							w_follow = w_mapped * v_upper / v_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
					else if (joystick_angle <= lower_right_angle)	// mid up-down trapezoid
					{
						if (v_mapped > v_upper)
						{
							v_follow = v_upper;
							w_follow = w_mapped * v_upper / v_mapped;
						}
						else if (v_mapped < v_lower)
						{
							v_follow = v_lower;
							w_follow = w_mapped * v_lower / v_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
					else	// right triangle
					{
						if (v_mapped > v_upper)
						{
							v_follow = v_upper;
							w_follow = w_mapped * v_upper / v_mapped;
						}
						else if (w_mapped < w_lower)
						{
							w_follow = w_lower;
							v_follow = v_mapped * w_lower / w_mapped;
						}
						else
						{
							v_follow = v_mapped;
							w_follow = w_mapped;
						}
					}
				}
			}
		}
	}

	// if (v_mapped > v_upper)	// mapped v is greater than the DW
	// {
	// 	// scale down to v_upper
	// 	v_scale = v_upper;
	// 	if (fabs(v_scale) < 0.01)
	// 	{
	// 		w_scale = w_mapped;			
	// 	}
	// 	else
	// 	{
	// 		w_scale = w_mapped * v_scale / v_mapped;
	// 	}
	// 	if (w_scale > w_upper)	// scaled w is greater than the DW
	// 	{
	// 		v_follow = v_upper;
	// 		w_follow = w_upper;
	// 	}
	// 	else if (w_scale < w_lower)	// scaled w is smaller than the DW
	// 	{
	// 		v_follow = v_upper;
	// 		w_follow = w_lower;
	// 	}
	// 	else	// scaled w is inside the DW
	// 	{
	// 		v_follow = v_scale;
	// 		w_follow = w_scale;
	// 	}
	// }
	// else if (v_mapped < v_lower)	// mapped v is smaller than the DW
	// {
	// 	// scale up to v_lower
	// 	v_scale = v_lower;
	// 	w_scale = w_mapped * v_scale / v_mapped;
	// 	if (w_scale > w_upper)	// scaled w is greater than the DW
	// 	{
	// 		v_follow = v_lower;
	// 		w_follow = w_upper;
	// 	}
	// 	else if (w_scale < w_lower)	// scaled w is smaller than the DW
	// 	{
	// 		v_follow = v_lower;
	// 		w_follow = w_lower;
	// 	}
	// 	else	// scaled w is inside the DW
	// 	{
	// 		v_follow = v_scale;
	// 		w_follow = w_scale;
	// 	}
	// }
	// else	// mapped v is inside the DW
	// {
	// 	if (w_mapped > w_upper)	// mapped w is greater than the DW
	// 	{
	// 		v_follow = v_mapped;
	// 		w_follow = w_upper;
	// 	}
	// 	else if (w_mapped < w_lower)	// mapped w is smaller than the DW
	// 	{
	// 		v_follow = v_mapped;
	// 		w_follow = w_lower;
	// 	}
	// 	else	// scaled w is inside the DW
	// 	{
	// 		v_follow = v_mapped;
	// 		w_follow = w_mapped;
	// 	}
	// }

	float follow_cost = 0, temp_cost = 0;
	tf2::Vector3 follow_heading(v_follow, w_follow, 0);
	float joystick_length = joystick_heading.length();
	float follow_length = follow_heading.length();
	if (joystick_length < 0.1)
	{
		follow_cost = follow_length < 0.1 ? 0 : 1;
	}
	else
	{
		if (follow_length < 0.1)
		{
			follow_cost = 1;
		}
		else
		{
			float angle_diff = joystick_heading.angle(follow_heading) * 2 / M_PI;
			float vel_diff = fabs(joystick_length - follow_length) / joystick_length;
			temp_cost = angle_diff > 1 ? 1 : angle_diff;

			temp_cost = (1 - temp_cost) * ModelParams::weight_velocity;
			follow_cost = temp_cost * vel_diff + (1 - temp_cost) * angle_diff;
		}
	}
	// total_cost = powf(total_cost, 0.5);
	return follow_cost;
}

float WheelchairDSPOMDP::FollowUserReward(tf2::Vector3& follow_heading, float& v_after, float& w_after, float& follow_cost) const
{
	float follow_user_reward = (1 - follow_cost) * ModelParams::user_following_reward;
	float weight_vel = 0;
	if (fabs(v_after - follow_heading.getX()) < 0.1 && fabs(w_after - follow_heading.getY()) < 0.1)
	{
		return follow_user_reward;
	}
	else
	{
		tf2::Vector3 velocity_heading(v_after, w_after, 0);

		float follow_length = follow_heading.length();
		float velocity_length = velocity_heading.length();
		float heading_diff = 0, vel_diff = 0;
		if (follow_length < 0.1)	// (v_follow, w_follow) is 0, following user is to stop
		{
			heading_diff = velocity_length < 0.1 ? 0 : 1;	// if (v_after, w_after) is also 0, then no cost, otherwise highest cost
			vel_diff = velocity_length;	// cost equals the length of (v_after, w_after)
		}
		else	// (v_follow, w_follow) is non-zero, following user is to move
		{
			// if (v_after, w_after) is 0, then highest cost, otherwise based on angle discrepancy
			heading_diff = velocity_length < 0.1 ? 1 : follow_heading.angle(velocity_heading) * 2 / M_PI;
			// cost equals the length difference between (v_after, w_after) and (v_follow, w_follow) 
			vel_diff = fabs(follow_length - velocity_length) / follow_length;
		}
		// (v_combined == 0 ? 0 : fabs(dynamic_window.at<cv::Vec2f>(i, j)[0] - v_combined) / fabs(v_combined));

		// heading_diff = powf(heading_diff, 0.25);

		heading_diff = heading_diff > 1 ? 1 : heading_diff;
		weight_vel = (1 - heading_diff) * ModelParams::weight_velocity;
		
		float total_cost = (1 - weight_vel) * heading_diff + weight_vel * vel_diff;
		total_cost = powf(total_cost, 0.5);

		// float reward_idx = 1 - total_cost;

		return follow_user_reward - ModelParams::user_following_reward * total_cost - 2 * ModelParams::inter_goal_reward;

		// return (0.5 - total_cost) * 2 * ModelParams::user_following_reward;
	}
}

float WheelchairDSPOMDP::M2G_FollowUserReward(tf2::Vector3& joystick_heading, tf2::Vector3& final_heading) const
{
	float joystick_length = joystick_heading.length();
	float final_length = final_heading.length();

	float total_cost = 0, angle_diff = 0, vel_diff = 0, temp_cost = 0;
	
	if (joystick_length < 0.1)	// no joystick input
	{
		total_cost = final_length < 0.1 ? 0 : 1;
	}
	else
	{
		if (final_length < 0.1)
		{
			total_cost = 1;
		}
		else
		{
			angle_diff = joystick_heading.angle(final_heading) * 2 / M_PI;
			temp_cost = angle_diff > 1 ? 1 : angle_diff;


			vel_diff = fabs(joystick_length - final_length) / joystick_length;

			temp_cost = (1 - temp_cost) * ModelParams::weight_velocity;
			total_cost = temp_cost * vel_diff + (1 - temp_cost) * angle_diff;
		}
	}

	return (1 - total_cost) * ModelParams::user_following_reward;
}

double WheelchairDSPOMDP::CalAngleDiff(WheelchairStruct &wheelchair_status, tf2::Vector3& agent_heading, int &goal_idx, int point_idx) const
{
	tf2::Vector3 agent2goal(0, 0, 0);
	double angle_difference;
	// Obtain the angle between wheelchair heading and the vector pointing from wheelchair position to a certain point along the intermediate goal list
	if (point_idx == -1 || point_idx >= intermediate_goal_list.paths[goal_idx].poses.size())
	{
		agent2goal.setX(goal_positions[goal_idx].pose.position.x - wheelchair_status.agent_pose.position.x);
		agent2goal.setY(goal_positions[goal_idx].pose.position.y - wheelchair_status.agent_pose.position.y);
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
	return 3 * V_next + C_action; //+ V_current;
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

void WheelchairDSPOMDP::clipBelief(std::vector<float>& belief_vector) const
{
	for (int i = 0; i < belief_vector.size(); i++)
	{
		if (belief_vector[i] < ModelParams::lower_bound)
		{
			belief_vector[i] = ModelParams::lower_bound;
			continue;
		}
		else if (belief_vector[i] > ModelParams::upper_bound)
		{
			belief_vector[i] = ModelParams::upper_bound;
		}
	}
}

void WheelchairDSPOMDP::normalize(std::vector<float>& belief_vector) const
{
	float sum_value = 0;
	for (int i = 0; i < belief_vector.size(); i++)
	{
		sum_value += belief_vector[i];
	}
	for (int i = 0; i < belief_vector.size(); i++)
	{
		belief_vector[i] /= sum_value;
	}
}

int WheelchairDSPOMDP::TurningSteps(float& angle2turn, float& current_w, float& angular_vel) const
{
	// case 1: turn left, angular+
	// case -1: turn right, angular-
	// case 2: keep
	float step_size = ModelParams::transition_time * ModelParams::max_w_acceleration;
	float time2turn = 0;
	// max sector: accelerate to max speed and decelerate to 0
	int max_sector_steps = ceil(ModelParams::max_angular_speed / step_size);
	float acc_maxsector = 0, dec_maxsector = 0, maxsector = 0;

	float max_step_size = step_size * (max_sector_steps - 1);
	float rounding_diff = ModelParams::max_angular_speed - max_step_size;
	// first triangle, accelerate from 0 to max_step_size
	acc_maxsector += 0.5 * ModelParams::transition_time * (max_sector_steps - 1) * max_step_size;
	// second trapezoid, accelerate from max_step_size to max_angular_speed
	acc_maxsector += 0.5 * ModelParams::transition_time * (max_step_size + ModelParams::max_angular_speed);
	// third trapezoid, decelerate from max_angular_speed to rounding_diff
	dec_maxsector += 0.5 * ModelParams::transition_time * (max_sector_steps - 1) * (rounding_diff + ModelParams::max_angular_speed);
	// last triangle, decelerate from rounding_diff to 0
	dec_maxsector += 0.5 * ModelParams::transition_time * rounding_diff;

	maxsector = acc_maxsector + dec_maxsector;
	// float maxsector = ModelParams::max_angular_speed * ModelParams::max_angular_speed / ModelParams::max_w_acceleration;
	// the wheelchair is not turning currently
	if (fabs(current_w) < 0.1)
	{
		int turning_sign = angle2turn >= 0 ? 1 : -1;
		// check if maxsector can cover the angle2turn

		// angle2turn can be finished inside max sector
		if (maxsector >= fabs(angle2turn))
		{
			time2turn = 2 * sqrt(fabs(angle2turn) / ModelParams::max_w_acceleration);
			if (time2turn > 2 * ModelParams::transition_time)
			{
				angular_vel = step_size;
			}
			else
			{
				angular_vel = fabs(angle2turn) / ModelParams::transition_time;
				time2turn = 2 * ModelParams::transition_time;
			}
		}
		// accelerate to max, keep and decelerate to 0
		else
		{
			time2turn = 2 * (ModelParams::max_angular_speed / ModelParams::max_w_acceleration) + (fabs(angle2turn) - maxsector) / ModelParams::max_angular_speed;
			angular_vel = step_size;
		}
		angular_vel = turning_sign * angular_vel;
	}
	// the wheelchair is turning now
	else
	{
		// the wheelchair is rotating towards the path
		if (current_w * angle2turn > 0)
		{
			// first check when exerting max deceleration, how many radians can be finished before stop

			max_sector_steps = floor(fabs(current_w) / step_size);
			rounding_diff = fabs(current_w) - step_size * max_sector_steps;
			float sector2stop = 0.5 * ModelParams::transition_time * max_sector_steps * (fabs(current_w) + rounding_diff);
			sector2stop += 0.5 * ModelParams::transition_time * rounding_diff;
			// float sector2stop = 0.5 * current_w * current_w / ModelParams::max_w_acceleration;

			// the wheelchair cannot stop when facing the path
			if (fabs(sector2stop) > fabs(angle2turn))
			{
				// add the time to stop to the total time first
				time2turn += fabs(current_w)/ ModelParams::max_w_acceleration;
				if (time2turn > ModelParams::transition_time)
				{
					angular_vel = current_w > 0 ? current_w - step_size : current_w + step_size;
				}
				else
				{
					angular_vel = -current_w;
					time2turn = ModelParams::transition_time;
				}
				// then it becomes the first situation, wheelchair not moving, the rest angle to turn is sector2stop - angle2turn

				if (maxsector >= fabs(sector2stop) - fabs(angle2turn))
				{
					time2turn += 2 * sqrt((fabs(sector2stop) - fabs(angle2turn)) / ModelParams::max_w_acceleration);
				}
				// accelerate to max, keep and decelerate to 0
				else
				{
					time2turn += 2 * (ModelParams::max_angular_speed / ModelParams::max_w_acceleration) + (fabs(sector2stop) - fabs(angle2turn) - maxsector) / ModelParams::max_angular_speed;
				}
			}
			// the wheelchair can stop when facing the path
			else
			{
				// the fastest action sequence is to accelerate to max first, keep, and decelerate to 0 then

				// check the max sector
				max_sector_steps = ceil((ModelParams::max_angular_speed - fabs(current_w)) / step_size);
				max_step_size = fabs(current_w) + step_size * (max_sector_steps - 1);
				float time2max = max_sector_steps * ModelParams::transition_time;
				// float time2max = (ModelParams::max_angular_speed - fabs(current_w)) / ModelParams::max_w_acceleration;
				float current_maxsector = 0.5 * (fabs(current_w) + max_step_size) * ModelParams::transition_time * (max_sector_steps - 1);
				current_maxsector += 0.5 * (max_step_size + ModelParams::max_angular_speed) * ModelParams::transition_time;
				current_maxsector += dec_maxsector;

				// accelerate and decelerate
				if (current_maxsector > fabs(angle2turn))
				{
					// solve the Quadratic Equation, where acceleration time t is the unknown value
					// 2*(a*t)^2 + 4*w*a*t + w^2 - 2*a*S = 0, a: acceleration, w: current_w, S: the angle to turn
					// A = 2*a^2, B = 4*w*a, C = w^2 - 2*a*S, the solutions: (-B ± sqrt(B^2- 4*A*C)) / (2*A)
					float A = 2 * ModelParams::max_w_acceleration * ModelParams::max_w_acceleration;
					float B = 4 * fabs(current_w) * ModelParams::max_w_acceleration;
					float C = current_w * current_w - 2 * ModelParams::max_w_acceleration * fabs(angle2turn);
					float time1 = (-B + sqrt(B*B- 4*A*C)) / (2*A), time2 = (-B - sqrt(B*B- 4*A*C)) / (2*A);
					float acc_time = 0, max_speed = 0;
					if (time2 <= 0)
					{
						acc_time = time1;
					}
					else
					{
						acc_time = time2;
					}
					max_speed = fabs(current_w) + ModelParams::max_w_acceleration * acc_time;

					if (acc_time > ModelParams::transition_time)
					{
						angular_vel = current_w > 0 ? current_w + step_size : current_w - step_size;
					}
					else if (acc_time <= 0.5 * ModelParams::transition_time)
					{
						angular_vel = current_w;
					}
					else
					{
						angular_vel = current_w > 0 ? max_speed : -max_speed;
						acc_time = ModelParams::transition_time;
					}
					time2turn += acc_time;
					time2turn += max_speed / ModelParams::max_w_acceleration;
				}
				// accelerate to max, keep and decelerate
				else
				{
					if (time2max > ModelParams::transition_time)
					{
						angular_vel = current_w > 0 ? current_w + step_size : current_w - step_size;
					}
					// else if (time2max <= 0.5 * ModelParams::transition_time)
					// {
					// 	angular_vel = current_w;
					// }
					else
					{
						angular_vel = current_w > 0 ? ModelParams::max_angular_speed : -ModelParams::max_angular_speed;
						time2max = ModelParams::transition_time;
					}
					// first compute the time to keep max w
					time2turn += (fabs(angle2turn) - current_maxsector) / ModelParams::max_angular_speed;
					// then add the acceleration time and deceleration time
					time2turn += time2max + ModelParams::max_angular_speed / ModelParams::max_w_acceleration;
				}
			}
		}
		// the wheelchair is rotating away from the path
		else
		{
			// stop the wheelchair first by exerting max deceleration
			max_sector_steps = floor(fabs(current_w) / step_size);
			rounding_diff = fabs(current_w) - step_size * max_sector_steps;
			float sector2stop = 0.5 * ModelParams::transition_time * max_sector_steps * (fabs(current_w) + rounding_diff);
			sector2stop += 0.5 * ModelParams::transition_time * rounding_diff;
			
			float stop_turning_time = (max_sector_steps + 1) * ModelParams::transition_time;
			if (stop_turning_time > ModelParams::transition_time)
			{
				angular_vel = current_w > 0 ? current_w - step_size : current_w + step_size;
			}
			// else if (stop_turning_time <= 0.5 * ModelParams::transition_time)
			// {
			// 	angular_vel = 0;
			// }
			else
			{
				angular_vel = 0;
				stop_turning_time = ModelParams::transition_time;
			}
			time2turn += stop_turning_time;
			// radians that can be finished before stop
			sector2stop = 0.5 * current_w * current_w / ModelParams::max_w_acceleration;
			// then it becomes the first situation, wheelchair not moving, the rest angle to turn is sector2stop + angle2turn
			// but if the sum is greater than PI, namely 180 degrees, the angle to turn is just 2*PI - sum
			float sum_angle2turn = fabs(sector2stop) + fabs(angle2turn);
			if (fabs(sector2stop) + fabs(angle2turn) > M_PI)
				sum_angle2turn = 2 * M_PI - sum_angle2turn;

			if (maxsector >= fabs(sum_angle2turn))
			{
				time2turn += 2 * sqrt((fabs(sum_angle2turn)) / ModelParams::max_w_acceleration);
			}
			// accelerate to max, keep and decelerate to 0
			else
			{
				time2turn += 2 * (ModelParams::max_angular_speed / ModelParams::max_w_acceleration) + (fabs(sum_angle2turn) - maxsector) / ModelParams::max_angular_speed;
			}
		}
	}

	return ceil(time2turn / ModelParams::transition_time);
}

void WheelchairDSPOMDP::ContractAndInterpolatePath(WheelchairStruct &wheelchair_status, nav_msgs::Path &path_traversed, nav_msgs::Path &original_path, tf2::Quaternion& map_quat) const
{
	// contract the path and check collision

	// current_position is the current position of the wheelchair, check_point will sweep along the path
	geometry_msgs::Point &current_position = wheelchair_status.agent_pose.position;
	geometry_msgs::Point check_point;
	
	float dist2checkpoint = 0;
	// the index along the path whose point is the last collision free point
	int collision_free_index = 1000;
	// the final new path and interpolated path segement, the temp path is used for storing a temporary path
	std::vector<geometry_msgs::PoseStamped> new_path, interpolated_path, temp_path;
	new_path.clear();
	interpolated_path.clear();
	temp_path.clear();
	// the pose to add to the interpolated path
	geometry_msgs::PoseStamped pose2add;
	pose2add.header = original_path.header;
	pose2add.pose.orientation.w = 1;
	pose2add.pose.orientation.x = 0;
	pose2add.pose.orientation.y = 0;
	pose2add.pose.orientation.z = 0;

	int max_i = original_path.poses.size();
	// cout << "path size " << original_path.poses.size() + path_traversed.poses.size() << endl;
	for (int i = path_traversed.poses.size(); i > - max_i; --i)
	{
		// link the current position to the rest points to check collision, and do the collision check every 0.05m, namely the resolution of costmap
		if (i > 0)
		{
			check_point = path_traversed.poses[i - 1].pose.position;
		}
		else
		{
			check_point = original_path.poses[- i].pose.position;
		}
		dist2checkpoint = sqrt(powf(current_position.x - check_point.x, 2) + powf(current_position.y - check_point.y, 2));

		// cout << "dist2checkpoint " << i << " = " << dist2checkpoint << endl;

		// cout << "map_resolution " << map_resolution << endl;

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

			geometry_msgs::Point moving_point = current_position;

			for (int j = 1; j < ceil(num_steps); ++j)
			{
				moving_point.x += increment_x;
				moving_point.y += increment_y;

				// cout << "j: " << j << endl;

				// cout << "x_check in local frame: " << moving_point.x << endl;
				// cout << "y_check in local frame: " << moving_point.y << endl;

				// add a point to the path every 0.2m, namely every 4 steps since the step size is 0.05m

				if (j % 4 == 0)
				{
					pose2add.pose.position = moving_point;
					temp_path.push_back(pose2add);
				}

				tf2::Vector3 check_vector(moving_point.x, moving_point.y, 0);
				check_vector = tf2::quatRotate(map_quat, check_vector);

				float x_check = check_vector.getX();
				float y_check = check_vector.getY();

				int col_check = x_check >= 0 ? ceil(x_check / map_resolution) : floor(x_check / map_resolution);
				int row_check = y_check >= 0 ? ceil(y_check / map_resolution) : floor(y_check / map_resolution);

				// cout << "x_center in costmap frame: " << x_center << endl;
				// cout << "y_center in costmap frame: " << y_center << endl;

				// cout << "col_check in costmap frame: " << col_check << endl;
				// cout << "row_check in costmap frame: " << row_check << endl;

				// cout << "x_check: " << x_center + col_check << endl;
				// cout << "y_check: " << y_center + row_check << endl;

				int check_pixel = local_costmap.at<uint8_t>(y_center + row_check, x_center + col_check);

				// cout << "check_pixel: " << check_pixel << endl;

				if (check_pixel <= ModelParams::pixel_path)	// no collision
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
			temp_path.clear();
			break;
		}
		else	// no collision along this check_point, move to the next check_point and store the current path
		{
			interpolated_path.clear();
			interpolated_path = temp_path;
			temp_path.clear();
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
		collision_free_index = collision_free_index == path_traversed.poses.size() + 1 ? path_traversed.poses.size() : collision_free_index;
		new_path.resize(interpolated_path.size());
		std::copy(interpolated_path.begin(), interpolated_path.end(), new_path.begin());

		if (collision_free_index > 0)	// the path_traversed is not finished after the contraction
		{
			// copy the rest path_traversed into new path in a reverse order
			for (int j = collision_free_index; j > 0; --j)
			{
				new_path.push_back(path_traversed.poses[j - 1]);
			}
			// copy the whole original path
			std::copy(original_path.poses.begin(), original_path.poses.end(), std::back_inserter(new_path));
		}
		else	// the path_traversed is finished after the contraction
		{
			// copy the rest segment from the original path with points before collision_free_index removed
			std::copy(original_path.poses.begin() - collision_free_index, original_path.poses.end(), std::back_inserter(new_path));
		}		
		// cout << "original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << new_path.size() << endl;
		original_path.poses.clear();
		original_path.poses = new_path;
		// cout << "after clearing, original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << new_path.size() << endl;
	}
	else	// no collision detected along the whole path, the whole original path is discarded
	{
		// cout << "no collision detected along the whole path" << endl;
		// cout << "original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << interpolated_path.size() << endl;
		original_path.poses.clear();
		original_path.poses = interpolated_path;
		pose2add.pose.position = check_point;
		original_path.poses.push_back(pose2add);
		// cout << "after clearing, original path size " << original_path.poses.size() << endl;
		// cout << "new path size " << interpolated_path.size() << endl;
	}
	// add orientation to new path
	tf2::Vector3 point_direction;
	float yaw = 0;
	tf2::Quaternion point_quat;
	for (int i = 0; i < original_path.poses.size() - 1; ++i)
	{
		point_direction.setValue(original_path.poses[i + 1].pose.position.x - original_path.poses[i].pose.position.x,
            original_path.poses[i + 1].pose.position.y - original_path.poses[i].pose.position.y, 0);
		yaw = point_direction.angle(tf2::Vector3(1, 0, 0));
		yaw = point_direction.getY() >= 0 ? yaw : - yaw;
		point_quat.setRPY(0, 0, yaw);
        tf2::convert(point_quat, original_path.poses[i].pose.orientation);
	}
	original_path.poses[original_path.poses.size() - 1].pose.orientation = original_path.poses[original_path.poses.size() - 2].pose.orientation;
	
	new_path.clear();
	interpolated_path.clear();
	path_traversed.poses.clear();
}

void WheelchairDSPOMDP::GenerateNewPath(WheelchairStruct &wheelchair_status, nav_msgs::Path& current_path) const
{
	
	geometry_msgs::PoseStamped new_point;
	new_point.header = current_path.header;
	new_point.pose = wheelchair_status.agent_pose;
	current_path.poses.push_back(new_point);
	// std::vector<geometry_msgs::PoseStamped> new_path;
	// geometry_msgs::PoseStamped new_point;
	// new_point.header = current_path.header;
	// new_point.pose = wheelchair_status.agent_pose;
	// // simply add the new position to the path
	// new_path.push_back(new_point);
	// std::copy(current_path.poses.begin(), current_path.poses.end(), std::back_inserter(new_path));
	// current_path.poses.clear();
	// current_path.poses = new_path;
	// new_path.clear();
	// // // first check if the distance between the new position and the first point of the path is greater than the threshold, namely 0.2
	// // float dist2origin_sq = powf(wheelchair_status.agent_pose.position.x - current_path.poses[0].pose.position.x, 2)
	// // 	+ powf(wheelchair_status.agent_pose.position.y - current_path.poses[0].pose.position.y, 2);
	// // if (dist2origin_sq <= 0.04)	// distance is smaller than 0.2
	// // {
	// // 	// simply add the new position to the path
	// // 	new_path.poses.push_back(new_point);
	// // 	std::copy(current_path.poses.begin(), current_path.poses.end(), std::back_inserter(new_path.poses));
	// // }
	// // else	// distance is greater than 0.2, interpolation is required
	// // {
	// // }
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
		// out << "Joy: " << wheelchair_obs.joystick_obs <<
		// "; pos: x " << round(wheelchair_obs.wheelchair_pose.position.x * 100) / 100 <<
		// ", y " << round(wheelchair_obs.wheelchair_pose.position.y * 100) / 100 <<
		// "; ori: x " << round(wheelchair_obs.wheelchair_pose.orientation.x *100) / 100 << 
		// ", y " << round(wheelchair_obs.wheelchair_pose.orientation.y *100) / 100 <<
		// ", z " << round(wheelchair_obs.wheelchair_pose.orientation.z *100) / 100 <<
		// ", w " << round(wheelchair_obs.wheelchair_pose.orientation.w *100) / 100 <<
		// "; vel: v " << round(wheelchair_obs.wheelchair_twist.linear.x * 100) / 100 <<
		// ", w " << round(wheelchair_obs.wheelchair_twist.angular.z * 100) / 100 << endl;
		uint64_t obs_int;
		int pos_x = round((wheelchair_obs.wheelchair_pose.position.x + 5) * 100);
		int pos_y = round((wheelchair_obs.wheelchair_pose.position.y + 5) * 100);
		int theta = round((wheelchair_obs.agent_pose_angle + M_PI) * 100);
		int vel_v = round((wheelchair_obs.wheelchair_twist.linear.x + 2) * 10);
		int vel_w = round((wheelchair_obs.wheelchair_twist.angular.z + 2) * 10);
		obs_int = vel_w;
		obs_int = obs_int * 100 + vel_v;
		obs_int = obs_int * 1000 + theta;
		obs_int = obs_int * 1000 + pos_y;
		obs_int = obs_int * 1000 + pos_x;
		obs_int = obs_int * 100 + wheelchair_obs.joystick_obs;
		// cout << "joystick: " << wheelchair_obs.joystick_obs << ", pos_x " << pos_x << ", pos_y " << pos_y << ", theta " << theta << ", vel_v " << vel_v << ", vel_w " << vel_w << endl;
		// cout << "PrintObs: " << obs_int << endl;

		wheelchair_obs.SetIntObs(obs_int);

	}
}

} // namespace despot