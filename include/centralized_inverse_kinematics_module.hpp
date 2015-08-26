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
        std::vector<paramHelp::ParamProxyInterface *> custom_params;


        custom_params.push_back( new paramHelp::ParamProxyBasic<bool>(    "is_clik",
                                                                            IS_CLIK_ID,
                                                                            IS_CLIK_SIZE,
                                                                            paramHelp::PARAM_IN_OUT,
                                                                            NULL,
                                                                            "Enable Closed Loop Inverse Kinematics" ) );

        return custom_params;
    }

    virtual void custom_ph_param_value_changed_callback()
    {
        // get param helper
        std::shared_ptr< paramHelp::ParamHelperServer > ph = get_param_helper();
        // register all the callbacks
        ph->registerParamValueChangedCallback( IS_CLIK_ID, this );
    }

    virtual void custom_parameterUpdated(const paramHelp::ParamProxyInterface *pd)
    {
        centralized_inverse_kinematics_thread* thread = get_thread();
        if(pd->id == IS_CLIK_ID)
        {
            if( thread )
            {
                ROS_INFO("IS_CLIK WAS CHANGED!");
            }
        }
    }
    
    
};

#endif
