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

        double stepLength=0.10;
        double stepTime=0.8;
        double stepLengthy=ik_problem->DynamicWalk.HALF_HIP_WIDTH;
        double z_c=1;
        double zmpyref=ik_problem->DynamicWalk.HALF_HIP_WIDTH;
        double DSPhasePercent=0.2;
        double footEdgex=0.2;
        double clearance=0.06;
        double xFinal=0;
        RobotUtils::ftReadings ft_readings = robot.senseftSensors();

        std::vector<double> comInfo(6);
        comInfo = ik_problem->comStabilizer.filterdata(
                    _robot_real.iDyn3_model.getCOM()[0],
                    _robot_real.iDyn3_model.getCOM()[1], get_thread_period());

        std::vector<double> filterAngPitch(2,0);
        std::vector<double> filterAngRoll(2,0);
        yarp::sig::Vector imu = imuIFace->sense();
        filterAngPitch = ik_problem->controlPitch.filterdata2(imu(1)-ik_problem->controlPitch.offset, imu(7));
        filterAngRoll = ik_problem->controlRoll.filterdata2(imu(0)-ik_problem->controlRoll.offset, imu(6));

        ik_problem->controlPitch.controlFlag = 1;
        ik_problem->controlRoll.controlFlag = 1;
        ik_problem->controlPitch.alfa=0.5;
        ik_problem->controlRoll.alfa=0.5;

        ik_problem->comStabilizer.controlFlag=1;
        ik_problem->comStabilizery.controlFlag=1;


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
        Hiprotation = Ry(Pitch*0.5);
         Hiprotation=Hiprotation*Rx(Roll);
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

            //ik_problem->taskTorso->setReference(hipRef);

//            double comRef[ik_problem->comStabilizer.N2];
//            double comRefy[ik_problem->comStabilizery.N2];
//            double hipcontrol = 0.0;
//            double hipcontroly = 0.0;

            //Vector3d hipOffset;

//            ik_problem->comStabilizer.States<<comInfo[0],comInfo[1];
//            ik_problem->comStabilizery.States<<comInfo[3], comInfo[4];

//            for(unsigned int i = 0; i < ik_problem->comStabilizer.N2; ++i)
//                comRef[i] = ik_problem->comStabilizer.offset;
//            for(unsigned int i = 0; i < ik_problem->comStabilizery.N2; ++i)
//                comRefy[i] = ik_problem->comStabilizery.offset;
//            Vector3d COMvector(0.0, 0.0, 0.35);
//            hipOffset = ik_problem->controlPitch.DynamicCompensator(Hiprotation, COMvector, 80, 20);
//            hipcontrol =ik_problem->comStabilizer.apply(comRef)*0.8;
//            hipcontroly = ik_problem->comStabilizery.apply(comRefy)-ik_problem->comStabilizery.offset;

           // double CoMdx = hipcontrol+ hipOffset[0];
           // double CoMdy = hipcontroly*0.8 + hipOffset[1];

            double stepTime2=stepTime;
            if(ik_problem->DynamicWalk.ZMPstructure.stop==1 || ik_problem->DynamicWalk.ZMPstructure.stop==2){
                stepTime2=1.5*stepTime;}
            else{
                stepTime2=stepTime;
            }
            double fgcom[3], fgcomy[3];
            for(unsigned int i = 0; i < 3; ++i){
                fgcom[i] = comInfo[i];
                fgcomy[i] = comInfo[i+3];
            }
//            fgcom[0]=fgcom[0]-ik_problem->comStabilizer.offset;
//            fgcom[3]=fgcom[3]-ik_problem->comStabilizery.offset;
            ik_problem->DynamicWalk.UpdateStructure(stepLength,stepTime2,stepLengthy,z_c,zmpyref,DSPhasePercent,footEdgex);
            ik_problem->DynamicWalk.ZMP_preview(xFinal,fgcom,fgcomy,z_c);
            ik_problem->DynamicWalk.ZMPstructure.whichfoot=2;//irobot.WhichFoot;

            this->StabilizerCOP();
            yarp::sig::Vector CoMd = ik_problem->taskCoM->getReference();

            CoMd(0) = ik_problem->DynamicWalk.x_active[0]+ik_problem->comStabilizer.offset+deltaHip_ODE[0];
            //CoMd(1) = ik_problem->DynamicWalk.y_active[0];//+ik_problem->comStabilizery.offset;
            cout<<ik_problem->DynamicWalk.x_active[0]<<"__"<<deltaHip_ODE[0]<<"    "<<ik_problem->DynamicWalk.y_active[0]<<"__"<<deltaHip_ODE[1]<<endl;
            ik_problem->taskCoM->setReference(CoMd);

            yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
            LFootReference=ik_problem->taskLFoot->getReference();
            LFootReference(0,3) = ik_problem->DynamicWalk.ZMPstructure.LFX;
            LFootReference(1,3) = ik_problem->DynamicWalk.ZMPstructure.LFY-ik_problem->comStabilizery.offset;
            LFootReference(2,3) = ik_problem->DynamicWalk.ZMPstructure.LFZ+0.1435;
           // ik_problem->taskLFoot->setReference(LFootReference);

            yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
//            RFootReference=ik_problem->taskRFoot->getReference();
            RFootReference(0,3) = ik_problem->DynamicWalk.ZMPstructure.RFX;
            RFootReference(1,3) = ik_problem->DynamicWalk.ZMPstructure.RFY-ik_problem->comStabilizery.offset;
            RFootReference(2,3) = ik_problem->DynamicWalk.ZMPstructure.RFZ+0.1435;

            //ik_problem->taskRFoot->setReference(RFootReference);


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
            yarp::sig::Vector CoMd = ik_problem->taskCoM->getReference();

            for(int i=0;i<100;i++){
                comInfo = ik_problem->comStabilizer.filterdata(
                            CoMd[0],
                            CoMd[1], get_thread_period());
                filterAngPitch = ik_problem->controlPitch.filterdata2(0, imu(7));
                filterAngRoll = ik_problem->controlRoll.filterdata2(0, imu(6));
                double temporal=ik_problem->controlPitch.filterstate(imu(1));
                temporal=ik_problem->controlRoll.filterstate(imu(0));

            }

            ik_problem->comStabilizer.offset = comInfo[0];
            ik_problem->comStabilizery.offset =comInfo[3];

            ik_problem->comStabilizery.offsety =CoMd[1];

            for(int i=0;i<ik_problem->comStabilizer.N2+ik_problem->comStabilizer.sizeA+1;i++){
                ik_problem->comStabilizer.X[i]=ik_problem->comStabilizer.offset;
                ik_problem->comStabilizery.X[i]=ik_problem->comStabilizery.offset;
            }

            ik_problem->controlPitch.offset=imu[1];
            ik_problem->controlRoll.offset=imu[0];
            for(int i=0;i<ik_problem->controlPitch.N2+ik_problem->controlPitch.sizeA+1;i++){
                ik_problem->controlPitch.X[i]=0;//ik_problem->controlPitch.offset;
                ik_problem->controlPitch.X[i]=0;//ik_problem->controlPitch.offset;
            }


            ik_problem->torsoref=ik_problem->taskTorso->getReference();

            double fgcom[3], fgcomy[3];
            for(unsigned int i = 0; i < 3; ++i){
                fgcom[i] = comInfo[i];
                fgcomy[i] = comInfo[i+3];
            }

            ik_problem->DynamicWalk.Initialize(stepLength,stepTime,stepLengthy,z_c,zmpyref,DSPhasePercent,footEdgex);
            ik_problem->DynamicWalk.initStructure(fgcom,fgcomy,zmpyref,clearance);



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
Eigen::VectorXd centralized_inverse_kinematics_thread::ZMPcalculation(){

    static yarp::sig::Vector ft_left,ft_right;
    static VectorXd FTdata(12);
    robot.senseftSensor("l_leg_ft",ft_left);
    robot.senseftSensor("r_leg_ft",ft_right);
static Eigen::VectorXd LFTdata_in(6);
static Eigen::VectorXd RFTdata_in(6);
    for (int i=0; i<6; i++) {
        LFTdata_in(i) = ft_left(i);
        RFTdata_in(i) = ft_right(i);
    }
    static double forceTreshold = 40;
  //   static double FTsensorHeight = ROBOT::g_ground_to_FT_sensor.Get(); // Sensor measuring part to top of metal plate + metal plate thickness + metal plate mounting part to bottomo of foot
    static double zmpx=0, zmpy=0, lzmpx=0, lzmpy=0,lzmpz=0, rzmpx=0, rzmpy=0,rzmpz=0;

    static Eigen::VectorXd LFTdata = Eigen::VectorXd::Zero(LFTdata_in.rows());
    static Eigen::VectorXd RFTdata = Eigen::VectorXd::Zero(RFTdata_in.rows());
    static Eigen::VectorXd LFT_world = Eigen::VectorXd::Zero(LFTdata_in.rows());
    static Eigen::VectorXd RFT_world = Eigen::VectorXd::Zero(RFTdata_in.rows());

    static int cutOffFreqency = g_settings.filters_zmpCutOffFreq.Get();
    static Clpf<Eigen::VectorXd> LFTlpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get()), RFTlpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get());
    cutOffFreqency = g_settings.filters_zmpCutOffFreq.Get();
    static Clpf<Eigen::VectorXd> zmpLpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get());

    LFTdata = LFTdata_in;
    RFTdata = RFTdata_in;


    static KDL::Frame ankleToFT(KDL::Vector(0.005, 0, -0.107));
    static KDL::Frame worldToLFT, worldToRFT;


    VectorXd lAnkle(6), rAnkle(6);
    yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
    LFootReference=ik_problem->taskLFoot->getReference();
    yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
    RFootReference=ik_problem->taskRFoot->getReference();


    for(int i=0;i<3;i++){
    lAnkle[0,i] = LFootReference(i,3);
    lAnkle[0,i+3] = LFootReference(i,i);
    rAnkle[0,i] = RFootReference(i,3);
    rAnkle[0,i+3] = RFootReference(i,i);

    }

    worldToLFT = eigen2KDL(lAnkle) * ankleToFT;
    worldToRFT = eigen2KDL(rAnkle) * ankleToFT;

    if (transformWrenchKDL(LFTdata, worldToLFT, &LFT_world) ) {
      error.add("Error when calculating wrench in ZMP function");
    }
    if (transformWrenchKDL(RFTdata, worldToRFT, &RFT_world) ) {
      error.add("Error when calculating wrench in ZMP function");
    }
    //if ( (RFT_world(2) > forceTreshold) && (LFT_world(2) > forceTreshold)) {
            rzmpx = -RFT_world(4)/RFT_world(2);
            rzmpy = RFT_world(3)/RFT_world(2);
            rzmpz=0;
            lzmpx = -LFT_world(4)/LFT_world(2);
            lzmpy = LFT_world(3)/LFT_world(2);
            lzmpz=0;
            zmpx  = (LFT_world(2) * lzmpx + RFT_world(2) * rzmpx ) / ( LFT_world(2) + RFT_world(2) );
            zmpy  = (LFT_world(2) * lzmpy + RFT_world(2) * rzmpy ) / ( LFT_world(2) + RFT_world(2) );
    //} else if (RFT_world(2) > forceTreshold) {
//            zmpx  = -RFT_world(4)/RFT_world(2);
//            zmpy  = RFT_world(3)/RFT_world(2);
//    } else if (LFT_world(2) > forceTreshold ) {
//            zmpx  = -LFT_world(4)/LFT_world(2);
//            zmpy  = LFT_world(3)/LFT_world(2);
//    }
    Eigen::VectorXd zmpOut(6);

    zmpOut=ik_problem->DynamicWalk.filterdatazmp(lzmpx,rzmpx, lzmpy,rzmpy);
    return zmpOut;

}
Eigen::Vector2d centralized_inverse_kinematics_thread::Forcecalculation(){

    static yarp::sig::Vector ft_left,ft_right;
    static VectorXd FTdata(12);
    robot.senseftSensor("l_leg_ft",ft_left);
    robot.senseftSensor("r_leg_ft",ft_right);
static Eigen::VectorXd LFTdata_in(6);
static Eigen::VectorXd RFTdata_in(6);
    for (int i=0; i<6; i++) {
        LFTdata_in(i) = ft_left(i);
        RFTdata_in(i) = ft_right(i);
    }

    static double forceTreshold = 40;
  //   static double FTsensorHeight = ROBOT::g_ground_to_FT_sensor.Get(); // Sensor measuring part to top of metal plate + metal plate thickness + metal plate mounting part to bottomo of foot
    static double zmpx=0, zmpy=0, lzmpx=0, lzmpy=0,lzmpz=0, rzmpx=0, rzmpy=0,rzmpz=0;

    static Eigen::VectorXd LFTdata = Eigen::VectorXd::Zero(LFTdata_in.rows());
    static Eigen::VectorXd RFTdata = Eigen::VectorXd::Zero(RFTdata_in.rows());
    static Eigen::VectorXd LFT_world = Eigen::VectorXd::Zero(LFTdata_in.rows());
    static Eigen::VectorXd RFT_world = Eigen::VectorXd::Zero(RFTdata_in.rows());

    static int cutOffFreqency = g_settings.filters_zmpCutOffFreq.Get();
    static Clpf<Eigen::VectorXd> LFTlpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get()), RFTlpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get());
    cutOffFreqency = g_settings.filters_zmpCutOffFreq.Get();
    static Clpf<Eigen::VectorXd> zmpLpf(cutOffFreqency, ROBOT::g_motorControlLoopTime.Get());

    LFTdata = LFTlpf.filter(LFTdata_in);
    RFTdata = RFTlpf.filter(RFTdata_in);


    static KDL::Frame ankleToFT(KDL::Vector(0.005, 0, -0.107));
    static KDL::Frame worldToLFT, worldToRFT;


    VectorXd lAnkle(6), rAnkle(6);
    yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
    LFootReference=ik_problem->taskLFoot->getReference();
    yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
    RFootReference=ik_problem->taskRFoot->getReference();


    for(int i=0;i<3;i++){
    lAnkle[0,i] = LFootReference(i,3);
    lAnkle[0,i+3] = LFootReference(i,i);
    rAnkle[0,i] = RFootReference(i,3);
    rAnkle[0,i+3] = RFootReference(i,i);
    }

    worldToLFT = eigen2KDL(lAnkle) * ankleToFT;
    worldToRFT = eigen2KDL(rAnkle) * ankleToFT;
    Vector2d returnV(LFT_world(2),RFT_world(2));

}
void centralized_inverse_kinematics_thread::StabilizerCOP(){
    Vector3d CopPos_L,CopPos_R;
    double g=9.81;
    yarp::sig::Matrix massMat;
    _robot_real.iDyn3_model.getFloatingBaseMassMatrix(massMat);
    double totalMass = massMat(0,0);
    double Gmg=totalMass*g;
    double Fzmin=10;
    Eigen::VectorXd zmpOut;
    zmpOut=this->ZMPcalculation();
    CopPos_L(0)=ik_problem->DynamicWalk.ZMPstructure.zmpx[0]-ik_problem->DynamicWalk.ZMPstructure.LFX;
    CopPos_L(1)=ik_problem->DynamicWalk.ZMPstructure.zmpy[0]-ik_problem->DynamicWalk.ZMPstructure.LFY;
    CopPos_L(2)=0;
    CopPos_R(0)=ik_problem->DynamicWalk.ZMPstructure.zmpx[0]-ik_problem->DynamicWalk.ZMPstructure.RFX;
    CopPos_R(1)=ik_problem->DynamicWalk.ZMPstructure.zmpy[0]-ik_problem->DynamicWalk.ZMPstructure.RFY;
    CopPos_R(2)=0;
    Vector3d robotLzmp(zmpOut(0),zmpOut(1),zmpOut(2));
    Vector3d robotRzmp(zmpOut(3),zmpOut(4),zmpOut(5));

    Vector3d deltaZMP_L=robotLzmp - CopPos_L;
    Vector3d deltaZMP_R=robotRzmp - CopPos_R;
    Vector2d Zforce=this->Forcecalculation();
    double Fzl, Fzr;
    Fzl = Zforce(0);
    Fzr = Zforce(1);

    if(Fzl< Fzmin){
        Fzl=Fzmin;
    }
    if(Fzr< Fzmin){
        Fzr=Fzmin;
    }
    Vector3d cop_delta(0,0,0);

    cop_delta = deltaZMP_L * Fzl/(Fzl+Fzr) + deltaZMP_R * Fzr/(Fzl+Fzr);

    deltaZMPx_old=deltaZMPx_ODE;
    deltaZMPy_old=deltaZMPy_ODE;
//    deltaZMPx_ODE=COPX.applyFilter(cop_delta(0));
//    deltaZMPy_ODE=COPY.applyFilter(cop_delta(1));
    deltaZMPx_ODE=cop_delta(0);
    deltaZMPy_ODE=cop_delta(1);


    double sum0ode=0;
    double sumode[4]={0};
    /*derevative of filtered COP */
    for(int i=0;i<samples2ODE-1;i++){
        dZMP_bufferODE[0][i]=dZMP_bufferODE[0][i+1];
        dZMP_bufferODE[1][i]=dZMP_bufferODE[1][i+1];
    }//this shifts the data in array
    double ts=get_thread_period();
    dZMP_bufferODE[0][samples2ODE-1]=(deltaZMPx_ODE-deltaZMPx_old)/ts;
    dZMP_bufferODE[1][samples2ODE-1]=(deltaZMPy_ODE-deltaZMPy_old)/ts;
    // begin of ZMP derevative filtering

    for (int j=0;j<2;j++){
        for(int i=samples2ODE;i>0;i--){
            sum0ode+=i;
            sumode[j] += i*dZMP_bufferODE[j][i-1];
        }
        dZMPODE[j]=sumode[j]/sum0ode;
    }
    //std::cout<<"deltaHIP:"<<irobotDS.dt<<"   "<<sumode[1]<<std::endl;
    FzODE[0]+=1*(Fzl-Fzmin -FzODE[0])*ts;//left
    FzODE[1]+=1*(Fzr-Fzmin -FzODE[1])*ts;//right
    //2.5 not 1
    double n1=(FzODE[0]+FzODE[1])/(Gmg*0.9);//normalized unit based on GRF/mg
    if(n1>1.2)
    {
        n1=1.2;
    }
    else if(n1<0)
    {
        n1=0;
    }
    // Must normalize!!

    //---- stabilizer law: gain must < 1 to have negative feedback control
    double Kx=0.1*n1;//Kx=0.2;
    double Ky=0.2*n1;//Ky=0.4;
    double Cx=-0.005*n1;//-0.01
    double Cy= -0.01*n1;//-0.02
    /*-------------- here is the main law  -------------------*/
    deltaHip_ODE[0] = Kx*deltaZMPx_ODE +  Cx*dZMPODE[0];//delta hip x
    deltaHip_ODE[1] = Ky*deltaZMPy_ODE +  Cy*dZMPODE[1];//delta hip y


        double Fext= 1-(Fzl+Fzr)/Gmg;
        double Kz = 12; // normalized stiffness
        //double Zdamping=1.5;
        //double Cz = 2*Zdamping*sqrt(Kz); // for unit mass
        double Cz = 5; // for unit mass

        if ((Fzl+Fzr)>2*Fzmin)//normalized unit based on GRF/mg
        {
            scaleCOP += 5*(1-scaleCOP)*ts;//if load on the ground, coefficient is 1
        }
        else
        {
            scaleCOP += 5*(0-scaleCOP)*ts;// otherwise it is in the air, coefficient is 0
        }
        deltaHip_ODE[2] = scaleCOP*(Fext*ts + Cz*deltaHip_ODE[2])/(Kz*ts+Cz);//delta hip z


        if (deltaHip_ODE[2]>0.001)
        {
            deltaHip_ODE[2]=0.001;
        }
        else if (deltaHip_ODE[2]<-0.1)
        {
            deltaHip_ODE[2]=-0.1;
        }

        if (deltaHip_ODE[0]>0.14)
        {
            deltaHip_ODE[0]=0.14;
        }
        else if (deltaHip_ODE[0]<-0.10)
        {
            deltaHip_ODE[0]=-0.10;
        }
        if (deltaHip_ODE[1]>0.15)
        {
            deltaHip_ODE[1]=0.15;
        }
        else if (deltaHip_ODE[1]<-0.15)
        {
            deltaHip_ODE[1]=-0.15;
        }
}

