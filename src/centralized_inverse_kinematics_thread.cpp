#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"
#include <ros/ros.h>

centralized_inverse_kinematics_thread::centralized_inverse_kinematics_thread(   std::string module_prefix, 
										yarp::os::ResourceFinder rf, 
										std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    _q( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _dq( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _tau( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _dq_ref( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    open_sot_server()
    
{
    // setting floating base under left foot
    robot.idynutils.updateiDyn3Model(_q, true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);
}

bool centralized_inverse_kinematics_thread::custom_init()
{
    robot.sense(_q, _dq, _tau);
    robot.idynutils.updateiDyn3Model(_q, true);

    open_sot_server.create_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

    if(open_sot_server.solve(_dq_ref))
    {
        robot.setPositionDirectMode();
        yarp::sig::Vector q_ref = _q;
        q_ref += _dq_ref;
        robot.move(q_ref);
        return true;
    }
    else
        return false;
}

void centralized_inverse_kinematics_thread::custom_release()
{
    ROS_INFO("Resetting Problem");
    open_sot_server.reset_problem();
    ROS_INFO("Problem reset!");
}

void centralized_inverse_kinematics_thread::run()
{   
    /** Sense **/
    //if is_click
    //  robot.sense(_q)
    //else
    _q += _dq_ref;

    /** Update Models **/
    robot.idynutils.updateiDyn3Model(_q,true);

    /** Update OpenSoTServer **/
    open_sot_server.update(_q);

    if(open_sot_server.solve(_dq_ref))
    {
        yarp::sig::Vector q_ref = _q;
        q_ref += _dq_ref;
        robot.move(q_ref);
    }
    else
    {
        _dq_ref = 0.0;
        ROS_ERROR("ERROR solving stack of tasks!");
    }
}



