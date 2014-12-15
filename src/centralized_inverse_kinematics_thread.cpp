#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"

centralized_inverse_kinematics_thread::centralized_inverse_kinematics_thread(   std::string module_prefix, 
										yarp::os::ResourceFinder rf, 
										std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    _q( model.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _model_com(get_robot_name(),
	       get_urdf_path(),
	       get_srdf_path() )
    
{
    // setting floating base under left foot
    _model_com.iDyn3_model.setFloatingBaseLink(_model_com.left_leg.index);
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
