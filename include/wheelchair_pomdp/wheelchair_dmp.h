#ifndef WHEELCHAIR_DMP_H_
#define WHEELCHAIR_DMP_H_


#include <Python.h>
//#include "NumCpp.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
//#include "cs.h"
//#include "dmp.h"
//#include <dlfcn.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "numpy/ndarraytypes.h"

#include <voronoi_msgs_and_types/PathList.h>

class DMP_variables
{
private:
   
    
public:
    
    Eigen::MatrixXd dmp_w; // weight of gaussians in the force term Dimension n_dmps* n_bfs
    Eigen::Vector2d dmp_goal; // Goal coordinates, Dimension n_dmps
    

     // Should be stored in particle state and changed to EigenVector from state variables
    //Canonical system variables
    //double cs_x;
    //Eigen::Vector2d dmp_y; // Current pose, Dimension n_dmps
    //Eigen::Vector2d dmp_dy; // Current velocity, Dimension n_dmps
    //Eigen::Vector2d dmp_ddy; // Current acceleration, Dimension n_dmps
    DMP_variables(){
        dmp_goal = Eigen::Vector2d::Zero();
        //dmp_y = Eigen::Vector2d::Zero();
        //dmp_dy = Eigen::Vector2d::Zero();
        //dmp_ddy = Eigen::Vector2d::Zero();
    };
    ~DMP_variables(){}; 
    void Copy_from_PyObject(PyObject* dmp_pyobject, int n_dmps, int n_bfs);
    //void Copy_from_DMP_variable(DMP_variables* dmp_variable);

    

};


class DMP_init_variables
{
private:
     void cs_step_discrete(double& cs_x, float tau, float error_coupling);
    /* data */
public:
    PyObject* pModule ; // Stores the imported python module
    PyObject* dmp_pyobject ; //Variable used to store python dmp class object
    PyObject* imitate_path_function; //Stores imitate_path_function
    PyObject* dmp_print_function;
    int n_dmps;
    int n_bfs;
    //Canonical system variables
    double cs_ax; //a gain term on the dynamical system

    double dt; //the timestep
    Eigen::VectorXd dmp_c; // gaussian centers for the force term  Dimension n_bfs
    Eigen::VectorXd dmp_h; // gaussian variance for the force term Dimension n_bfs

    Eigen::Vector2d dmp_ay ;//gain on attractor term y dynamics , Dimension n_dmps
    Eigen::Vector2d dmp_by ;//gain on attractor term y dynamics , Dimension n_dmps
    Eigen::Vector2d dmp_y0 ; // Initial pose, Dimension n_dmps, Same i.e. 0.0 0.0 for all goals

    std::vector<DMP_variables> dmp_variables; //Used to contain dmp weights corresponding to each intermediate goals
    DMP_init_variables(){};
    DMP_init_variables(int num_goals);
    ~DMP_init_variables();

    void dmp_step( const Eigen::MatrixXd& dmp_w, const Eigen::Vector2d& dmp_goal, 
    double& cs_x, Eigen::Vector2d& dmp_y, Eigen::Vector2d& dmp_dy, Eigen::Vector2d dmp_ddy,
    float tau=1.0, float error=0.0);
    void dmp_step( int path_id, 
    double& cs_x, Eigen::Vector2d& dmp_y, Eigen::Vector2d& dmp_dy, Eigen::Vector2d dmp_ddy,
    float tau=1.0, float error=0.0);
    void Copy_from_PyObject(); 
    void update_weights(int num_goals, const voronoi_msgs_and_types::PathList& intermediate_goal_list);
};



#endif