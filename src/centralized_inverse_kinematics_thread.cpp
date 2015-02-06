#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <OpenSoT/tasks/velocity/Interaction.h>

using namespace OpenSoT::tasks::velocity;

centralized_inverse_kinematics_thread::centralized_inverse_kinematics_thread(   std::string module_prefix, 
										yarp::os::ResourceFinder rf, 
										std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    _q( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _dq( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _tau( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    _dq_ref( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    ik_problem(),
    _is_phantom(true),
    _is_clik(false),
    _n()
    
{
    // setting floating base under left foot
    robot.idynutils.updateiDyn3Model(_q, true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);

    std::string topic_name = module_prefix + "/" + robot.idynutils.getRobotName() + "/joint_command";
    _joint_command_publisher = _n.advertise<sensor_msgs::JointState>(topic_name, 1);
}

bool centralized_inverse_kinematics_thread::custom_init()
{    
    // param helper link param for all the chains and the max_vel param
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    ph->linkParam( IS_PHANTOM_ID, &_is_phantom );
    ph->linkParam( IS_CLIK_ID, &_is_clik );

    robot.sense(_q, _dq, _tau);
    robot.idynutils.updateiDyn3Model(_q, true);

    RobotUtils::ftReadings ft_readings = robot.senseftSensors();
    RobotUtils::ftPtrMap ft_sensors = robot.getftSensors();
    for(RobotUtils::ftReadings::iterator it = ft_readings.begin(); it != ft_readings.end(); it++)
    {
        moveit::core::LinkModel* ft_link = robot.idynutils.moveit_robot_model->getLinkModel(
                    ft_sensors[it->first]->getReferenceFrame());
        std::string ft_joint_name = ft_link->getParentJointModel()->getName();
        int ft_index = robot.idynutils.iDyn3_model.getFTSensorIndex(ft_joint_name);

        robot.idynutils.iDyn3_model.setSensorMeasurement(ft_index, -1.0*it->second);
    }

    //ik_problem = boost::shared_ptr<simple_problem>(new simple_problem());
    //ik_problem = boost::shared_ptr<interaction_problem>(new interaction_problem());
    ik_problem = boost::shared_ptr<wb_manip_problem>(new wb_manip_problem());

    boost::shared_ptr<general_ik_problem::ik_problem> problem =
            ik_problem->create_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

//    yarp::sig::Vector desired_wrench(6, 0.0);
//    desired_wrench = ik_problem->taskRWrist->getReferenceWrench();
//    desired_wrench[0] = 25.0; desired_wrench[1] = 5.0; desired_wrench[2] = 5.0;
//    ik_problem->taskRWrist->setReferenceWrench(desired_wrench);

    try{ qp_solver = OpenSoT::solvers::QPOases_sot::Ptr(new OpenSoT::solvers::QPOases_sot(
                                                                     problem->stack_of_tasks,
                                                                     problem->bounds,
                                                                     problem->damped_least_square_eps));}
    catch (const char* s){
        ROS_ERROR(s);
        ROS_ERROR("The Module will be stopped.");
        custom_release();
        return false;}

    if(qp_solver->solve(_dq_ref))
    {
        robot.setPositionDirectMode();
        yarp::sig::Vector q_ref = _q;
        q_ref += _dq_ref;
        if(_is_phantom){
            ros::spinOnce();

            sensor_msgs::JointState joint_msg;
            for(unsigned int i = 0; i < q_ref.size(); ++i){
                joint_msg.position.push_back(q_ref[i]);
                joint_msg.effort.push_back(0.0);
                joint_msg.velocity.push_back(_dq_ref[i]/(get_thread_period()*0.001));
                joint_msg.name.push_back(robot.getJointNames()[i]);}
            joint_msg.header.stamp = ros::Time::now();
            _joint_command_publisher.publish(joint_msg);
        }
        else
            robot.move(q_ref);

        ROS_INFO("SoT is succesfully intialized!");

        if(_is_phantom)
            ROS_WARN("SOLVER is running in PHANTOM MODE");
        else
            ROS_INFO("SOLVER is running in NORMAL MODE");

        return true;
    }
    else
        ROS_ERROR("ERRORS occurred during SoT initialization!");
        return false;
}

void centralized_inverse_kinematics_thread::custom_release()
{
    std::unique_lock<std::mutex>lck(_mtx);

    ROS_INFO("Resetting Solver");
    qp_solver.reset();
    ROS_INFO("Solver reset!");
    ROS_INFO("Resetting Problem");
    ik_problem.reset();
    ROS_INFO("Problem reset!");
    _dq_ref = 0.0;
}

void centralized_inverse_kinematics_thread::run()
{
    bool tmp_is_phantom = _is_phantom;

    if(ik_problem && qp_solver)
    {
        /** Sense **/
        RobotUtils::ftReadings ft_readings = robot.senseftSensors();
        RobotUtils::ftPtrMap ft_sensors = robot.getftSensors();
        if(_is_clik)
            robot.sense(_q, _dq, _tau);
        else
            _q += _dq_ref;

        /** Update Models **/
        robot.idynutils.updateiDyn3Model(_q,true);

        for(RobotUtils::ftReadings::iterator it = ft_readings.begin(); it != ft_readings.end(); it++)
        {
            moveit::core::LinkModel* ft_link = robot.idynutils.moveit_robot_model->getLinkModel(
                        ft_sensors[it->first]->getReferenceFrame());
            std::string ft_joint_name = ft_link->getParentJointModel()->getName();
            int ft_index = robot.idynutils.iDyn3_model.getFTSensorIndex(ft_joint_name);

            robot.idynutils.iDyn3_model.setSensorMeasurement(ft_index, -1.0*it->second);
        }

        /** Update OpenSoTServer **/
        ik_problem->update(_q);

//        std::cout<<"Actual Wrench in "<<ik_problem->taskRWrist->getBaseLink();
//        std::cout<<" ["<<ik_problem->taskRWrist->getActualWrench().toString()<<"]"<<std::endl;
//        ik_problem->recordWrench();

        if(qp_solver->solve(_dq_ref))
        {
            yarp::sig::Vector q_ref = _q;
            q_ref += _dq_ref;
            if(tmp_is_phantom){
                ros::spinOnce();

                sensor_msgs::JointState joint_msg;
                for(unsigned int i = 0; i < q_ref.size(); ++i){
                    joint_msg.position.push_back(q_ref[i]);
                    joint_msg.effort.push_back(0.0);
                    joint_msg.velocity.push_back(_dq_ref[i]/(get_thread_period()*0.001));
                    joint_msg.name.push_back(robot.getJointNames()[i]);}
                joint_msg.header.stamp = ros::Time::now();
                _joint_command_publisher.publish(joint_msg);
            }
            else
                robot.move(q_ref);
        }
        else{
            _dq_ref = 0.0;
            ROS_ERROR("ERROR solving stack of tasks!");
        }
    }
}



