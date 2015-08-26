#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"
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
    _q_ref( robot.idynutils.iDyn3_model.getNrOfDOFs(), 0.0 ),
    ik_problem(),
    _is_clik(false),
    _counter(0)
    
{
    // setting floating base under left foot
    robot.idynutils.updateiDyn3Model(_q, true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);
}

bool centralized_inverse_kinematics_thread::custom_init()
{    
    // param helper link param for all the chains and the max_vel param
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    ph->linkParam( IS_CLIK_ID, &_is_clik );

    robot.sense(_q, _dq, _tau);
    _q_ref = _q;
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
    //ik_problem = boost::shared_ptr<wb_manip_problem>(new wb_manip_problem());
    ik_problem = boost::shared_ptr<walking_problem>(new walking_problem(robot.idynutils));

    std::string saveDataPath = GetEnv("ROBOTOLOGY_ROOT") + "/build/robots/walking/data/";
    yarp::sig::Matrix massMat;
    robot.idynutils.iDyn3_model.getFloatingBaseMassMatrix(massMat);
    std::string path_to_config = GetEnv("ROBOTOLOGY_ROOT") + "/robots/walking/app/conf/inputs";
    ik_problem->pattern_generator.reset(
                new Clocomotor(robot.getNumberOfJoints(), 0.005, 0.005, massMat(0,0),
                               get_robot_name(),get_urdf_path(), get_srdf_path(),
                               saveDataPath, path_to_config));
    ik_problem->walkingPatternGeneration(1.5, 10, 0.28, 0.05);

    boost::shared_ptr<general_ik_problem::ik_problem> problem =
            ik_problem->homing_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

    try{ qp_solver.reset(new OpenSoT::solvers::QPOases_sot(problem->stack_of_tasks,
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
        _q_ref += _dq_ref;
        robot.move(_q_ref);


        ROS_INFO("SoT is succesfully intialized!");


        if(_is_clik)
            ROS_INFO("SOLVER is running in CLIK MODE");

        ROS_INFO("PRESS A KEY TO START MODULE");
        cin.get();

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
    double tic = yarp::os::Time::now();

    if(ik_problem && qp_solver)
    {
        /** Sense **/
        RobotUtils::ftReadings ft_readings = robot.senseftSensors();
        RobotUtils::ftPtrMap ft_sensors = robot.getftSensors();

        yarp::sig::Vector q_measured(_q.size(), 0.0);
        robot.sense(q_measured, _dq, _tau);
        if(_is_clik)
            _q = q_measured;
        else
            _q += _dq_ref;

        /** Update Models **/
        robot.idynutils.updateiDyn3Model(_q,true);


        if(ik_problem->reset_solver && ik_problem->homing_done){


            if(!ik_problem->start_walking_pattern){
                ROS_INFO("PRESS A KEY TO START WALKING PATTERN GENERATOR");
                cin.get();
                ik_problem->start_walking_pattern = true;
            }


            int stance_foot = 1;
            if(ik_problem->updateWalkingPattern(ik_problem->LFootRef, ik_problem->RFootRef,
                                                ik_problem->pelvisRef, ik_problem->comRef,
                                                ik_problem->ZMPRef,
                                                stance_foot))
            {
                if(!ik_problem->walking_pattern_finished)
                {
                    ik_problem->switchSupportFoot(robot.idynutils, stance_foot);

                    ik_problem->taskLFoot->setReference(ik_problem->LFootRef);
                    ik_problem->taskRFoot->setReference(ik_problem->RFootRef);
                    ik_problem->taskPelvis->setReference(ik_problem->pelvisRef);
                    ik_problem->taskCoM->setReference(ik_problem->comRef);

                    yarp::sig::Matrix RArmRef = ik_problem->InitialRArmRef;
                    RArmRef(0,3) = RArmRef(0,3) - 0.4 + 0.4*cos(_counter/100.0);
                    RArmRef(1,3) = RArmRef(1,3) -0.4 + 0.4*cos(_counter/100.0);
                    yarp::sig::Vector dRArmRef(6, 0.0);
                    dRArmRef(0) = -(0.4/100.0)*sin(_counter/100.0);
                    dRArmRef(1) = -(0.4/100.0)*sin(_counter/100.0);
                    ik_problem->taskRArm->setReference(RArmRef, dRArmRef);
                    _counter++;
                }
            }
            else
                ROS_WARN_ONCE("WALKING PATTERN GENERATOR RETURN ERROR OR FINISHED!");
        }

        for(RobotUtils::ftReadings::iterator it = ft_readings.begin(); it != ft_readings.end(); it++)
        {
            moveit::core::LinkModel* ft_link = robot.idynutils.moveit_robot_model->getLinkModel(
                        ft_sensors[it->first]->getReferenceFrame());
            std::string ft_joint_name = ft_link->getParentJointModel()->getName();
            int ft_index = robot.idynutils.iDyn3_model.getFTSensorIndex(ft_joint_name);

            int sign = 1.0;
            if(ft_joint_name == "l_arm_ft" || ft_joint_name == "r_arm_ft"){
                //We want the force that the robot is doing on the environment, for this we use -1 in the arms!
                sign = -1.0;
            }

            robot.idynutils.iDyn3_model.setSensorMeasurement(ft_index, sign*it->second);
        }

        /** Update OpenSoTServer **/
        ik_problem->update(_q);

        if(ik_problem->homing_done && !ik_problem->reset_solver)
        {
            qp_solver.reset();

            ROS_WARN("RESET PROBLEM AND SOLVER");
            boost::shared_ptr<general_ik_problem::ik_problem> walking_problem =
                    ik_problem->create_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

            qp_solver.reset(new OpenSoT::solvers::QPOases_sot(walking_problem->stack_of_tasks,
                                                              walking_problem->bounds,
                                                              walking_problem->damped_least_square_eps));
            ik_problem->reset_solver = true;           
        }

        _dq_ref = 0.0;
        if(qp_solver->solve(_dq_ref))
        {
            _q_ref = _q;
            _q_ref += _dq_ref;
            robot.move(_q_ref);
        }
        else{
            _dq_ref = 0.0;
            ROS_ERROR("ERROR solving stack of tasks!");
        }
    }

    double toc = yarp::os::Time::now();
    double dt = toc - tic;

    if(dt*1000.0 > get_thread_period())
        ROS_WARN("Ctrl Loop is %f [ms] while thread rate is %f [ms]", dt*1000.0,
                 get_thread_period());

}



