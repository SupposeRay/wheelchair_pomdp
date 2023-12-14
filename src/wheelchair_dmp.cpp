#include "wheelchair_pomdp/wheelchair_dmp.h"


DMP_init_variables::DMP_init_variables(int num_goals)
{
    Py_Initialize();
    //PyRun_SimpleString("import sys");
    PyRun_SimpleString("import sys\n"
                     "sys.argv = ['']");
    PyRun_SimpleString("import numpy");
    PyRun_SimpleString("sys.path.append('../pydmps/examples')");
    PyRun_SimpleString("print(sys.version)");


    //Import wheelchair_dmps
    PyObject *pName;
    pName = PyUnicode_FromString("wheelchair_dmps");
    pModule = PyImport_Import(pName);
    if(pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"wheelchair_dmps\"\n");
        assert(0==1);
    }
    Py_DECREF(pName);



    //Create PyObject for printing dmp //Used for debugging
    dmp_print_function = PyObject_GetAttrString(pModule, "dmp_print");
    if (!(dmp_print_function && PyCallable_Check(dmp_print_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"dmp_print\"\n");
    }


    //Call get_dmp function
    PyObject* get_dmp_function = PyObject_GetAttrString(pModule, "get_dmp");
    if (!(get_dmp_function && PyCallable_Check(get_dmp_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_dmp\"\n");
    }
    
    //std::cout << "Reference count of get dmp function " << Py_REFCNT(get_dmp_function) << std::endl;
    //std::cout << "Reference count of get dmp function " << Py_REFCNT(get_dmp_function) << std::endl;
    dmp_pyobject = PyObject_CallObject(get_dmp_function, NULL);
    //std::cout << "Reference count of dmp " << Py_REFCNT(dmp_pyobject) << std::endl;
    Py_DECREF(get_dmp_function);
    //std::cout << "Reference count of get dmp function " << Py_REFCNT(get_dmp_function) << std::endl;
    //std::cout << "Reference count of dmp " << Py_REFCNT(dmp_pyobject) << std::endl;

    Copy_from_PyObject();

    //Create PyObject for imitate_path function
    imitate_path_function = PyObject_GetAttrString(pModule, "imitate_path");
    if (!(imitate_path_function && PyCallable_Check(imitate_path_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"imitate_path\"\n");
    }

    std::cout << "Reference count of dmp " << Py_REFCNT(dmp_pyobject) << std::endl;
    std::cout << "Reference count of dmp_imitate_function " << Py_REFCNT(imitate_path_function) << std::endl;
    
    //PyObject* pArgs;
    //pArgs = PyTuple_New(1);
    //PyTuple_SetItem(pArgs, 0, dmp_pyobject);
    //dmp_pyobject = PyObject_CallObject(dmp_print_function, pArgs);
    //Py_DECREF(pArgs);
     
    //For some reason one ref count is lost when entering upate function. 
    //So the reference count needs to be incremented to prvent segfault
    Py_INCREF(dmp_pyobject);
    for(int i = 0; i < num_goals; i++)
    {
        DMP_variables d_v = DMP_variables();
        dmp_variables.push_back(d_v);
    }

    dmp_y0 = Eigen::Vector2d::Zero();
       
}

DMP_init_variables::~DMP_init_variables()
{
    Py_DECREF(pModule);
    Py_DECREF(dmp_pyobject);
    Py_DECREF(imitate_path_function);
}


void DMP_init_variables::Copy_from_PyObject()
{
    
    PyArrayObject* dmp_array_attribute;
   
    PyObject* dmp_attribute;
    dmp_attribute = PyObject_GetAttrString(dmp_pyobject, "n_dmps");
    n_dmps = (int) PyLong_AsLong(dmp_attribute);
    Py_DECREF(dmp_attribute);

    dmp_attribute = PyObject_GetAttrString(dmp_pyobject, "n_bfs");
    n_bfs = (int) PyLong_AsLong(dmp_attribute);
    Py_DECREF(dmp_attribute);

    PyObject* dmp_pyobject_cs = PyObject_GetAttrString(dmp_pyobject, "cs");
    dmp_attribute = PyObject_GetAttrString(dmp_pyobject_cs, "ax");
    cs_ax = (double) PyFloat_AsDouble(dmp_attribute);
    Py_DECREF(dmp_attribute);

    /*dmp_attribute = PyObject_GetAttrString(dmp_pyobject_cs, "x");
    cs_x = (double) PyFloat_AsDouble(dmp_attribute);
    Py_DECREF(dmp_attribute);
    */
    Py_DECREF(dmp_pyobject_cs);

    dmp_attribute = PyObject_GetAttrString(dmp_pyobject, "dt");
    dt = (double) PyFloat_AsDouble(dmp_attribute);
    Py_DECREF(dmp_attribute);


    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "c");
    //int size_c =  PyArray_ITEMSIZE(dmp_array_attribute);
    //int type_pyobject_c = PyArray_TYPE(dmp_array_attribute);
    //std::cout << "Item_type of c is " << type_pyobject_c  << std::endl;
    //std::cout << "Item_size of c is " << size_c 
    //<< " size of float is " << sizeof(float) << " size of double is " << sizeof(double) << std::endl;
    dmp_c = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_bfs);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "h");
    dmp_h = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_bfs);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "ay");
    dmp_ay = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "by");
    dmp_by = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);
    
    /*
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "y0");
    dmp_y0 = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);


    
    PyArrayObject* dmp_array_attribute;
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "w");
    dmp_w = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned, Eigen::Stride<1,Eigen::Dynamic>>((double *) PyArray_DATA(dmp_array_attribute), n_dmps, n_bfs, Eigen::Stride<1,Eigen::Dynamic>(1,n_bfs));
    Py_DECREF(dmp_array_attribute);



    
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "goal");
    dmp_goal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "y");
    dmp_y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "dy");
    dmp_dy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribustd::cout << "Reference count of dmp " << Py_REFCNT(dmp_pyobject) << std::endl;te);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "ddy");
    dmp_ddy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);
    */
}

void DMP_init_variables::update_weights(int num_goals, const voronoi_msgs_and_types::PathList& intermediate_goal_list)
{

    std::cout << "Reference count of dmp in update weigths " << Py_REFCNT(dmp_pyobject) << std::endl;
    std::cout << "Reference count of dmp_imitate_function in update weigths " << Py_REFCNT(imitate_path_function) << std::endl;
    int path_size;
    PyObject* intermediate_goal; PyObject* pos_x_y;
    PyObject* pArgs;
    double pos_x, pos_y;
    for(int i = 0; i < num_goals; i++)
    {
        path_size = intermediate_goal_list.paths[i].poses.size();
        
        
        //Convert voronoi path into pyobject
        intermediate_goal = PyList_New(path_size+1);
        
        
        pos_x_y = PyList_New(2);
        PyList_SetItem(pos_x_y, 0, PyFloat_FromDouble(0.0));
        PyList_SetItem(pos_x_y, 1, PyFloat_FromDouble(0.0));
        PyList_SetItem(intermediate_goal, 0, pos_x_y);
        for(int j=0; j < path_size; j++)
        {
                    
            pos_x = intermediate_goal_list.paths[i].poses[j].pose.position.x;
            pos_y = intermediate_goal_list.paths[i].poses[j].pose.position.y;
            pos_x_y = PyList_New(2);
            PyList_SetItem(pos_x_y, 0, PyFloat_FromDouble(pos_x));
            PyList_SetItem(pos_x_y, 1, PyFloat_FromDouble(pos_y));
            PyList_SetItem(intermediate_goal, j+1, pos_x_y);

        }

        //Update goal value
        dmp_variables[i].dmp_goal[0] = pos_x;
        dmp_variables[i].dmp_goal[1] = pos_y;

        //Call imitate path function to compute weights
        pArgs = PyTuple_New(2);
        PyTuple_SetItem(pArgs, 0, dmp_pyobject);
        PyTuple_SetItem(pArgs, 1, intermediate_goal);

        dmp_pyobject = PyObject_CallObject(imitate_path_function, pArgs);
        Py_DECREF(pArgs);

        //Update weights
        dmp_variables[i].Copy_from_PyObject(dmp_pyobject, n_dmps, n_bfs);

    }

}

void DMP_init_variables::cs_step_discrete(double& cs_x, float tau, float error_coupling)
{
    cs_x += (-cs_ax * cs_x * error_coupling) * tau * dt;
}

void DMP_init_variables::dmp_step( int path_id, 
    double& cs_x, Eigen::Vector2d& dmp_y, Eigen::Vector2d& dmp_dy, Eigen::Vector2d dmp_ddy,
    float tau, float error)
{
   dmp_step(dmp_variables[path_id].dmp_w, dmp_variables[path_id].dmp_goal, cs_x, dmp_y, dmp_dy, dmp_ddy, tau, error);    
}
void DMP_init_variables::dmp_step( const Eigen::MatrixXd& dmp_w, const Eigen::Vector2d& dmp_goal, 
    double& cs_x, Eigen::Vector2d& dmp_y, Eigen::Vector2d& dmp_dy, Eigen::Vector2d dmp_ddy,
    float tau, float error)
{
    /*Run the DMP system for a single timestep.

    tau float: scales the timestep
                increase tau to make the system execute faster
    error float: optional system feedback
    */
    
    float error_coupling = 1.0 / (1.0 + error);

    // run canonical system
    cs_step_discrete(cs_x, tau, error_coupling);

    //Eigen::VectorXd psi_step1 = (cs_x - dmp_c.array()).array().square();
    
    //generate basis function activation
    Eigen::VectorXd psi = ((-1*dmp_h).array() * (cs_x - dmp_c.array()).array().square()).array().exp(); //np.exp(-self.h * (x - self.c) ** 2)//gen_psi(x);

    double sum_psi = psi.sum();
    /*if(cs_x > 0.985)
    {
        std::cout << "Psi_step1 =  " << psi_step1.transpose() << std::endl;
        std::cout << "Psi is " << psi.transpose() << std::endl;
    }
    else
    {
        std::cout << "cs_x = " << cs_x ;
    }*/
    //std::cout << "x is " << cs_x << " sum_psi is " << sum_psi << " ";
    for(int d = 0; d < n_dmps; d++) //d in range(self.n_dmps):
    {
        //generate the forcing term
        double f = cs_x * (dmp_goal[d] - dmp_y0[d]) * psi.dot(dmp_w.row(d)); //self.gen_front_term(x, d) * (np.dot(psi, self.w[d]))
        //std:: cout << "Bedore sum_psi f["<< d << "] is" << f << " ";
        if (fabs(sum_psi) > 1e-6)
        {
            f = f/sum_psi;
        }
        //std:: cout << "f["<< d << "] is" << f << " ";
        // DMP acceleration
        dmp_ddy[d] = (
            dmp_ay[d] * (dmp_by[d] * (dmp_goal[d] - dmp_y[d]) - dmp_dy[d]) + f
        );
        //if external_force is not None:
        //    self.ddy[d] += external_force[d]
        dmp_dy[d] += dmp_ddy[d] * tau * dt * error_coupling;
        dmp_y[d] += dmp_dy[d] * tau * dt * error_coupling;
    }
}


void DMP_variables::Copy_from_PyObject(PyObject* dmp_pyobject, int n_dmps, int n_bfs)
{

    PyArrayObject* dmp_array_attribute;
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "w");
    dmp_w = Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned, Eigen::Stride<1,Eigen::Dynamic>>((double *) PyArray_DATA(dmp_array_attribute), n_dmps, n_bfs, Eigen::Stride<1,Eigen::Dynamic>(1,n_bfs));
    Py_DECREF(dmp_array_attribute);



    /*
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "goal");
    dmp_goal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    
    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "y");
    dmp_y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "dy");
    dmp_dy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);

    dmp_array_attribute = (PyArrayObject*)PyObject_GetAttrString(dmp_pyobject, "ddy");
    dmp_ddy = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((double *) PyArray_DATA(dmp_array_attribute), n_dmps);
    Py_DECREF(dmp_array_attribute);
    */

}
