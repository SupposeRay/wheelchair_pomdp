#include <despot/planner.h>
#include "wheelchair_pomdp/wheelchair_model.h"
#include "wheelchair_pomdp/wheelchair_gazebo.h"
#include <fstream>

using namespace despot;

class WheelchairPlanner: public Planner
{
public:
  WheelchairPlanner()
  {}

  DSPOMDP* InitializeModel(option::Option* options)
  {
    std::cout << "Initializing POMDP model..." << std::endl;
    DSPOMDP* model = new WheelchairDSPOMDP();
    std::cout << "Created model." << std::endl; 
    return model;
  }

  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options)
  {
      // //Create a custom world as defined and implemented by the user
      // LaserTagWorld* world = new LaserTagWorld();
      // //Establish connection with external system
      // world->Connect();
      // //Initialize the state of the external system
      // world->Initialize();
      // static_cast<LaserTag*>(model)->NoiseSigma(LaserTagWorld::noise_sigma_);
      // //Inform despot the type of world
      // world_type = "simulator";
      // return world;

      std::cout << "Initializing POMDP world..." << std::endl;
      // Create a custom world defined and implemented by user
      GazeboWheelchairWorld* world = new GazeboWheelchairWorld((WheelchairDSPOMDP*) model);
      
      std::cout << "Created world." << std::endl;  // segfaults without this line
      ros::Time::init();
      ros::Duration(1.5).sleep();

      
      world->Connect();

      std::cout << "Connection established." << std::endl;
      world->Initialize();
      //Lidar points are updated after Connect() function is called
      if (Globals::config.useGPU)
		        model->InitGPUModel();
      // Inform despot the type of world
      world_type = "simulator";
      return world;

      // return InitializePOMDPWorld(world_type, model, options);
  }

  void InitializeDefaultParameters()
  {
    // DESPOT parameters setting

    Globals::config.num_scenarios = ModelParams::num_particles;
    Globals::config.sim_len = ModelParams::planning_step;
    Globals::config.max_policy_sim_len = ModelParams::max_default_policy_len;
    Globals::config.time_per_move = ModelParams::planning_time;
    Globals::config.discount = ModelParams::reward_discount;
    Globals::config.search_depth = ModelParams::search_depth;
    //Globals::config.search_depth = 30;
    Globals::config.useGPU = false;
    if (ModelParams::enable_multithreading)
    {
      Globals::config.use_multi_thread_ = true;
      Globals::config.NUM_THREADS = ModelParams::num_thread;
    }
    logging::level(ModelParams::verbosity_lvl);

    // Globals::config.pruning_constant = 0.01;
    Globals::config.exploration_mode=UCT;
	  Globals::config.exploration_constant=0.3;
	  Globals::config.exploration_constant_o = 1.0;

	  Globals::config.silence=false;
	  Obs_type=OBS_INT_ARRAY;
	  DESPOT::num_Obs_element_in_GPU= ModelParams::num_int_observations;
    // cout << "Complete loading parameters." << endl;
    // nh_param.shutdown();
    // nh_param.~NodeHandle();
  }

  std::string ChooseSolver()
  {
	  return "DESPOT";
  }

  /*Customize your planning pipeline by overloading the following function if necessary*/
  void PlanningLoop(Solver*& solver, World* world, Logger* logger)
  {
    GazeboWheelchairWorld *wheelchair_world = static_cast<GazeboWheelchairWorld*>(world);
    bool terminal, print_flag = true;
    OBS_TYPE obs;
    
    if (wheelchair_world->odom_receive && wheelchair_world->joystick_receive && wheelchair_world->path_receive)
    {
      if (Globals::config.sim_len > 0)
      {
        for (int i = 0; i < Globals::config.sim_len;)
        {
          if (!wheelchair_world->zero_joystick_input)
          {
            terminal = RunStep(solver, world, logger);
            print_flag = true;
            if (terminal)
              break;
          }
          else
          {
            if (print_flag)
            {
              terminal = wheelchair_world->ExecuteAction(-1, obs);
              cout << endl << "No joystick input, the system has been paused..." << endl << endl;
              print_flag = false;
            }
            continue;
          }
          if (i == Globals::config.sim_len - 1)
          {
            cout << "Finished the max number of steps, stopping the wheelchair..." << endl;
            wheelchair_world->ExecuteAction(-1, obs);
            sleep(1);
          }
          i++;
        }
      }
      else
      {
        while(true)
        {
          if (!wheelchair_world->zero_joystick_input)
          {
            terminal = RunStep(solver, world, logger);
            print_flag = true;
            if (terminal)
              break;
          }
          else
          {
            terminal = wheelchair_world->ExecuteAction(-1, obs);
            if (print_flag)
            {
              cout << endl << "No joystick input, the system has been paused..." << endl << endl;
              print_flag = false;
            }
            continue;
          }
        }
      }
    }
    else
    {
      cout << endl << "Topics not all received, the task is aborted..." << endl << endl;
    }
  }

  /*Customize the inner step of the planning pipeline by overloading the following function if necessary*/
  bool RunStep(Solver* solver, World* world, Logger* logger)
  {
    GazeboWheelchairWorld *wheelchair_world = static_cast<GazeboWheelchairWorld*>(world);
    bool wrong_planner = false;
    logger->CheckTargetTime();

    double step_start_t = get_time_second();

    double start_t = get_time_second();
    ACT_TYPE action = 0;
    if (wheelchair_world->planner_type == "POMDP")
    {
      action = solver->Search().action;
    }
    else if (wheelchair_world->planner_type == "pure-DWA")
    {
      action = 7;
    }
    else if (wheelchair_world->planner_type == "belief-DWA")
    {
      action = 8;
    }
    else
    {
      cout << "ERROR! WRONG PLANNER TYPE! Please check the config file!" << endl;
      wrong_planner = true;
      action = -1;      
    }
    double end_t = get_time_second();
    double search_time = (end_t - start_t);
    logi << "[RunStep] Time spent in " << typeid(*solver).name()
        << "::Search(): " << search_time << endl;

    if (search_time < ModelParams::planning_time)
    {
      double sleep_time = ModelParams::planning_time - search_time;
      sleep_time *= powf(10, 6);
      usleep(sleep_time);
    }
    OBS_TYPE obs;
    start_t = get_time_second();
    bool terminal = world->ExecuteAction(action, obs);
    end_t = get_time_second();
    double execute_time = (end_t - start_t);
    logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

    if(!terminal && wheelchair_world->planner_type == "POMDP")
	  {
      start_t = get_time_second();
      solver->BeliefUpdate(action, obs);
      end_t = get_time_second();
      double update_time = (end_t - start_t);
      logi << "[RunStep] Time spent in Update(): " << update_time << endl;
	  }
    if (wrong_planner)
    {
      terminal = true;
    }

    return logger->SummarizeStep(step_++, round_, terminal, action, obs,
        step_start_t);
  }

};

int main(int argc, char* argv[]) 
{
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  return WheelchairPlanner().RunPlanning(argc, argv);
  // return WheelchairPlanner().RunEvaluation(argc, argv);
}
