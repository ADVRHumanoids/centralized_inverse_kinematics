#include <yarp/os/all.h>

#include "centralized_inverse_kinematics_thread.h"
#include "centralized_inverse_kinematics_constants.h"
#include <sensor_msgs/JointState.h>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <MatrixVector.h>

#define LOG_DATA_SIZE 8

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
    _ft_measurements(),
    ik_problem(),
    _is_clik(false),
    _counter(0),
    _robot_real(get_robot_name(), get_urdf_path(), get_srdf_path())
    
{
    // setting floating base under left foot
    robot.idynutils.updateiDyn3Model(_q, true);
    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);

    _robot_real.updateiDyn3Model(_q, true);
    _robot_real.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);

    RobotUtils::ftPtrMap ft_sensors = robot.getftSensors();
    for(RobotUtils::ftPtrMap::iterator it = ft_sensors.begin();
        it != ft_sensors.end(); it++)
    {
        iDynUtils::ft_measure ft_measurement;
        ft_measurement.first = it->second->getReferenceFrame();
        yarp::sig::Vector dummy_measure(6 ,0.0);
        ft_measurement.second = dummy_measure;

        _ft_measurements.push_back(ft_measurement);
    }

    // the IMU reading is in Waist
    imuIFace = new yarp_IMU_interface(get_module_prefix(), true,
                                      get_robot_name(), "Waist");
}

bool centralized_inverse_kinematics_thread::custom_init()
{    
    // param helper link param for all the chains and the max_vel param
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    ph->linkParam( IS_CLIK_ID, &_is_clik );

    robot.sense(_q, _dq, _tau);
    _q_ref = _q;

    RobotUtils::ftReadings ft_readings = robot.senseftSensors();
    for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
        _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];

    robot.idynutils.updateiDyn3Model(_q, _ft_measurements, true);
    _robot_real.updateiDyn3Model(_q, true);
    
        
    // checking joint limit boudnb
    auto max = model.iDyn3_model.getJointBoundMax();
    auto min = model.iDyn3_model.getJointBoundMin();
    
    for (int i=0;i<_q.size();i++)
    {
        if (_q[i]>max[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<_q[i]<<") is outside maximum bound: "<<max[i]<<std::endl;
        }
        if (_q[i]<min[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<_q[i]<<") is outside minimum bound: "<<min[i]<<std::endl;
        }
    }

    std::string urdf_path = get_urdf_path();
    std::string srdf_path = get_srdf_path();
    ik_problem = boost::shared_ptr<IK_PROBLEM_TYPE_CONST>(
                new IK_PROBLEM_TYPE_CONST(robot.idynutils, urdf_path, srdf_path));

    boost::shared_ptr<general_ik_problem::ik_problem> problem =
            ik_problem->homing_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

    if(problem){
        try{ qp_solver.reset(new OpenSoT::solvers::QPOases_sot(problem->stack_of_tasks,
                                                               problem->bounds,
                                                               problem->global_constraints,
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
        }
        else{
            ROS_ERROR("ERRORS occurred during SoT initialization!");
            return false;}
    }
    else
        ROS_INFO("No homing provided, skipping");

    if(_is_clik)
        ROS_INFO("SOLVER is running in CLIK MODE");

    ROS_INFO("PRESS A KEY TO START MODULE");
        cin.get();

    return true;
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
    
    ROS_INFO("Savin LOG DATA!");
    std::cout << (log_data.size()) << std::endl;
    
    std::ofstream output_file("./test_log_data" + std::to_string(ros::Time::now().toSec()) + ".dat");
//     std::ostream_iterator<double> output_iterator(output_file, "\n");
//     std::copy(log_data.begin(), log_data.end(), output_iterator);
    if (output_file.is_open()) {
        for(int i = 0; i < log_data.size(); i += LOG_DATA_SIZE) {
            for(int j = 0; j < LOG_DATA_SIZE; j++) {
                output_file << log_data[i+j]   << " ";
            }
            output_file << "\n";
        }
        output_file.close();
    }

}

void centralized_inverse_kinematics_thread::run()
{
    double tic = yarp::os::Time::now();


    if(ik_problem && qp_solver)
    {
        /** Sense **/
        RobotUtils::ftReadings ft_readings = robot.senseftSensors();

        std::vector<double> comInfo(6);
        comInfo = ik_problem->zmpBalance.filterdata(
                    _robot_real.iDyn3_model.getCOM()[0],
                    _robot_real.iDyn3_model.getCOM()[1], get_thread_period());

        ik_problem->controlPitch.controlFlag = 1;
        ik_problem->controlRoll.controlFlag = 1;
        ik_problem->controlPitch.alfa=0;
        ik_problem->controlRoll.alfa=0;

        std::vector<double> filterAngPitch(2,0);
        std::vector<double> filterAngRoll(2,0);
        yarp::sig::Vector imu = imuIFace->sense();
        filterAngPitch = ik_problem->controlPitch.filterdata2(imu(1)-ik_problem->controlPitch.offset, imu(7));
        filterAngRoll = ik_problem->controlRoll.filterdata2(imu(0)-ik_problem->controlRoll.offset, imu(6));


        ik_problem->controlPitch.States<<filterAngPitch[0],filterAngPitch[1];
        ik_problem->controlRoll.States<<filterAngRoll[0],filterAngRoll[1];
        double refPitch[ik_problem->controlPitch.N2];
        double refRoll[ik_problem->controlPitch.N2];
        for(unsigned i=0;i<ik_problem->controlPitch.N2;i++){
            refPitch[i]=0;
            refRoll[i]=0;
        }

        Matrix3d Hiprotation = Matrix3d::Identity();
        double Pitch=ik_problem->controlPitch.apply(refPitch);
        double Roll=ik_problem->controlRoll.apply(refRoll);
        Hiprotation = Ry(Pitch);
        Hiprotation=Hiprotation*Rx(Roll);

        //ik_problem->zmpBalance.controlFlag=1;
        //ik_problem->zmpBalancey.controlFlag=1;
        //ik_problem->zmpBalance.alfa=0.5;




        yarp::sig::Vector q_measured(_q.size(), 0.0);
        robot.sense(q_measured, _dq, _tau);
        if(_is_clik)
            _q = q_measured;
        else
            _q += _dq_ref;

        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];


        /** Update Models **/
        robot.idynutils.updateiDyn3Model(_q, _ft_measurements, true);
        _robot_real.updateiDyn3Model(q_measured, true);


        if(ik_problem->reset_solver && ik_problem->homing_done){


            if(!ik_problem->start_walking_pattern){
                ROS_INFO("PRESS A KEY TO START WALKING PATTERN GENERATOR");
                cin.get();
                ik_problem->start_walking_pattern = true;
            }



            int stance_foot = 1;
            yarp::sig::Matrix hipRef(4,4); hipRef.eye();
            for(unsigned int i = 0; i < 3; ++i){
                for(unsigned int j = 0; j < 3; ++j)
                    hipRef(i,j) =  Hiprotation(i,j);
            }

            ik_problem->taskTorso->setReference(hipRef);

            double comRef[ik_problem->zmpBalance.N2];
            double comRefy[ik_problem->zmpBalancey.N2];
            double hipcontrol = 0.0;
            double hipcontroly = 0.0;

            Vector3d hipOffset;
            double fgcom[3], fgcomy[3];
            for(unsigned int i = 0; i < 3; ++i){
                fgcom[i] = comInfo[i];
                fgcomy[i] = comInfo[i+3];
            }
            ik_problem->zmpBalance.States<<comInfo[0],comInfo[1];
            ik_problem->zmpBalancey.States<<comInfo[3], comInfo[4];

            for(unsigned int i = 0; i < ik_problem->zmpBalance.N2; ++i)
                comRef[i] = ik_problem->zmpBalance.offset;
            for(unsigned int i = 0; i < ik_problem->zmpBalancey.N2; ++i)
                comRefy[i] = ik_problem->zmpBalancey.offset;
            Vector3d COMvector(0.0, 0.0, 0.35);
            hipOffset = ik_problem->controlPitch.DynamicCompensator(Hiprotation, COMvector, 80, 20);
            hipcontrol =ik_problem->zmpBalance.apply(comRef)*0.8;
            hipcontroly = ik_problem->zmpBalancey.apply(comRefy)-ik_problem->zmpBalancey.offset;

            double CoMdx = hipcontrol+ hipOffset[0];
            double CoMdy = hipcontroly*0.8 + hipOffset[1];

            yarp::sig::Vector CoMd = ik_problem->taskCoM->getReference();
           // CoMd(0) = CoMdx;
//             CoMd(1) = ik_problem->zmpBalancey.offsety-CoMdy;
            ik_problem->taskCoM->setReference(CoMd);

        // LOG DATA
        log_data.push_back(comInfo[0]);
        log_data.push_back(comInfo[4]);
        log_data.push_back(filterAngPitch[0]);
        log_data.push_back(filterAngRoll[0]);
        log_data.push_back(_robot_real.iDyn3_model.getCOM()[0]);
        log_data.push_back(_robot_real.iDyn3_model.getCOM()[1]);
        log_data.push_back(imu[0]);
        log_data.push_back(imu[1]);

         }

        /** Update OpenSoTServer **/
        ik_problem->update(_q);

        if(ik_problem->homing_done && !ik_problem->reset_solver)
        {
            qp_solver.reset();

            ROS_WARN("RESET PROBLEM AND SOLVER");
            boost::shared_ptr<general_ik_problem::ik_problem> main_problem =
                    ik_problem->create_problem(_q, robot.idynutils, get_thread_period(), get_module_prefix());

            qp_solver.reset(new OpenSoT::solvers::QPOases_sot(main_problem->stack_of_tasks,
                                                              main_problem->bounds,
                                                              main_problem->global_constraints,
                                                              main_problem->damped_least_square_eps));
            ik_problem->reset_solver = true;

            for(int i=0;i<100;i++){
                comInfo = ik_problem->zmpBalance.filterdata(
                            _robot_real.iDyn3_model.getCOM()[0],
                            _robot_real.iDyn3_model.getCOM()[1], get_thread_period());
                filterAngPitch = ik_problem->controlPitch.filterdata2(0, imu(7));
                filterAngRoll = ik_problem->controlRoll.filterdata2(0, imu(6));
                double temporal=ik_problem->controlPitch.filterstate(imu(1));
                temporal=ik_problem->controlRoll.filterstate(imu(0));

            }

            ik_problem->zmpBalance.offset = comInfo[0];
            ik_problem->zmpBalancey.offset =comInfo[3];
            yarp::sig::Vector CoMd = ik_problem->taskCoM->getReference();

            ik_problem->zmpBalancey.offsety =CoMd[1];

            for(int i=0;i<ik_problem->zmpBalance.N2+ik_problem->zmpBalance.sizeA+1;i++){
                ik_problem->zmpBalance.X[i]=ik_problem->zmpBalance.offset;
                ik_problem->zmpBalancey.X[i]=ik_problem->zmpBalancey.offset;
            }

            ik_problem->controlPitch.offset=imu[1];
            ik_problem->controlRoll.offset=imu[0];
            for(int i=0;i<ik_problem->controlPitch.N2+ik_problem->controlPitch.sizeA+1;i++){
                ik_problem->controlPitch.X[i]=0;//ik_problem->controlPitch.offset;
                ik_problem->controlPitch.X[i]=0;//ik_problem->controlPitch.offset;
            }


            ik_problem->torsoref=ik_problem->taskTorso->getReference();

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



