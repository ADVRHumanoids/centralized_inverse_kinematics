#ifndef centralized_inverse_kinematics_THREAD_H_
#define centralized_inverse_kinematics_THREAD_H_

#include <GYM/control_thread.hpp>
#include <problems/simple_problem.h>
#include <problems/interaction_problem.h>
#include <problems/wb_manip_problem.h>
#include <problems/walking_problem.h>
#include <ros/ros.h>
#include <yarp/os/all.h>
#include <mutex>


/**
 * @brief centralized_inverse_kinematics control thread
 * 
 **/
class centralized_inverse_kinematics_thread : public control_thread
{
private:
    /**
     * @brief robot status
     * 
     */
    yarp::sig::Vector _q, _dq, _tau;

    /**
     * @brief _dq_ref
     */
    yarp::sig::Vector _dq_ref;

    yarp::sig::Vector _q_ref;

    std::vector<iDynUtils::ft_measure> _ft_measurements;

    yarp_IMU_interface *imuIFace;


    bool _is_clik;

    std::mutex _mtx;

    unsigned int _counter;

    iDynUtils _robot_real;
    
    // LOG
    std::vector<double> log_data;
    
public:
    //boost::shared_ptr<simple_problem> ik_problem;
    //boost::shared_ptr<interaction_problem> ik_problem;
    //boost::shared_ptr<wb_manip_problem> ik_problem;
    boost::shared_ptr<walking_problem> ik_problem;

    boost::shared_ptr<OpenSoT::solvers::QPOases_sot> qp_solver;
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     centralized_inverse_kinematics_thread( std::string module_prefix, 
					    yarp::os::ResourceFinder rf, 
					    std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief centralized_inverse_kinematics control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();

    void custom_release();
    
    /**
     * @brief centralized_inverse_kinematics control thread main loop
     * 
     */
    virtual void run();
    Eigen::VectorXd ZMPcalculation();
    Eigen::Vector2d Forcecalculation();
    void StabilizerCOP();
    double deltaZMPx_ODE;   // zmp error
    double deltaZMPy_ODE;

    double deltaZMPx_old;
    double deltaZMPy_old;
    const int samples2ODE=50;
    double dZMP_bufferODE[2][50]; //derevative of filtered ZMP circular buffer
    double dZMPODE[2]={0};
    double FzODE[2];
     double deltaHip_ODE[3];
     double scaleCOP;
};

#endif
