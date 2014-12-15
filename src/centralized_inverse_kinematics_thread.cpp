#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"

centralized_inverse_kinematics_thread::centralized_inverse_kinematics_thread(   std::string module_prefix, 
										yarp::os::ResourceFinder rf, 
										std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph )
{
    // TODO: skeleton constructor
}

bool centralized_inverse_kinematics_thread::custom_init()
{
    // TODO: skeleton function   
    return true;
}

void centralized_inverse_kinematics_thread::run()
{   
    // TODO: skeleton function
}    
