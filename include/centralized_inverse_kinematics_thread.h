#ifndef centralized_inverse_kinematics_THREAD_H_
#define centralized_inverse_kinematics_THREAD_H_

#include <GYM/control_thread.hpp>
#include <openSoTServer.h>

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

    void custom_release();
    
public:

    openSoTServer open_sot_server;
    
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
    
    /**
     * @brief centralized_inverse_kinematics control thread main loop
     * 
     */
    virtual void run();

    
};

#endif
