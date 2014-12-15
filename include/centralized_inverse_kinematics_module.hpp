#ifndef centralized_inverse_kinematics_MODULE_HPP_
#define centralized_inverse_kinematics_MODULE_HPP_

#include <GYM/control_module.hpp>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"

/**
 * @brief centralized_inverse_kinematics module derived from control_module
 * 
 * @author 
 */
class centralized_inverse_kinematics_module : public control_module<centralized_inverse_kinematics_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    centralized_inverse_kinematics_module(   int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : control_module<centralized_inverse_kinematics_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
