#include <problems/walking_problem.h>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>
#include <OpenSoT/SubTask.h>

#define mSecToSec(X) (X*0.001)

#define LAMBDA_GAIN 1.0//.7
#define COM_GAIN 1.0//0.5

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

namespace ROBOT {

CFootState g_feetState;
CsharedVar<std::string> g_robot_name;
CsharedVar<std::string> g_urdf_path;
CsharedVar<std::string> g_srdf_path;

CsharedVar<double>      g_motorControlLoopTime(0.001);
CsharedVar<double>      g_IMUrefreshTime(0.005);
CsharedVar<double>      g_totalMass(0.0);
CsharedVar<std::string> g_saveDataPath;

CsharedVar<Eigen::VectorXd> g_upBodConfigInCKinAndDYn;
CsharedVar<Eigen::VectorXd> g_upperBodyHomingPos_const;

/**
 * @brief g_ankle_height: this data should be taken from URDF!
 *  Bigman = 0.1435
 *  Hydra = 0.1
 */
CsharedVar<double> g_ankle_height(0.1435);
CsharedVar<double> g_ground_to_FT_sensor(0.0365);
Eigen::Vector3d temporary_footToAnkleShift(0.0,0.0,g_ankle_height.Get());
CsharedVar<Eigen::Vector3d> g_footToAnkleShift(temporary_footToAnkleShift);
}


walking_problem::walking_problem(iDynUtils& robot_model, std::string &urdf_path, std::string &srdf_path):
    general_ik_problem(robot_model, urdf_path, srdf_path),
    n(),
    LFootRef(4,4),
    RFootRef(4,4),
    pelvisRef(4,4),
    comRef(3,0.0),
    ZMPRef(3,0.0),
    log_com("com", logger_proto::file_type::matlab),
    log_l_foot("l_foot", logger_proto::file_type::matlab),
    log_r_foot("r_foot", logger_proto::file_type::matlab),
    log_pelvis("pelvis", logger_proto::file_type::matlab),
    log_q("q", logger_proto::file_type::matlab),
    log_com_d("com_d", logger_proto::file_type::matlab),
    log_l_foot_d("l_foot_d", logger_proto::file_type::matlab),
    log_r_foot_d("r_foot_d", logger_proto::file_type::matlab),
    log_pelvis_d("pelvis_d", logger_proto::file_type::matlab),
    log_q_d("q_d", logger_proto::file_type::matlab),
    log_zmp_d("zmp_d", logger_proto::file_type::matlab),
    controlPitch(),
    comStabilizer()
{
    LFootRef.eye();
    RFootRef.eye();
    pelvisRef.eye();

    walking_pattern_finished = true;
    homing_done = false;
    reset_solver = false;
    start_walking_pattern = false;

    //We start from left foot according to centralized_inverse_kinematics_thread.cpp
    stance_foot = STANCE_FOOT::LEFT_FOOT;

    new_world_pub = n.advertise<geometry_msgs::TransformStamped>("/anchor_to_world_pose", 1000);


    /** FROM HERE WE INITIALIZE CLOCOMOTOR **/
    std::string saveDataPath = GetEnv("WALKMAN_ROOT") + "/build/drc/walking/data/";
    yarp::sig::Matrix massMat;
    robot_model.iDyn3_model.getFloatingBaseMassMatrix(massMat);
    std::string path_to_config = GetEnv("WALKMAN_ROOT") + "/drc/walking/app/conf/inputs";

    this->pattern_generator.reset(
                new Clocomotor(robot_model.iDyn3_model.getNrOfDOFs(), 0.005, 0.005, massMat(0,0),
                               robot_model.getRobotName(),
                               _urdf_path, _srdf_path,
                               saveDataPath, path_to_config));

    /**
     * @brief walkingPatternGeneration
     *
     * Parameters:
     * WALK-MAN = 1.5, 10, 0.28, 0.05
     * HYDRA = 1.0, 10, 0.25, 0.05
     */
    walkingPatternGeneration(1.5, 10, 0.28, 0.05);

    int foo = 1;
    //Here is needed two times...
    updateWalkingPattern(LFootRef, RFootRef, pelvisRef, comRef, ZMPRef, foo);
    updateWalkingPattern(LFootRef, RFootRef, pelvisRef, comRef, ZMPRef, foo);


}

walking_problem::~walking_problem()
{

}

void walking_problem::log(const yarp::sig::Matrix& LFoot, const yarp::sig::Matrix& RFoot,
         const yarp::sig::Matrix& Pelvis, const yarp::sig::Vector& CoM,
         const yarp::sig::Vector& q,
         const yarp::sig::Matrix& LFoot_d, const yarp::sig::Matrix& RFoot_d,
         const yarp::sig::Matrix& Pelvis_d, const yarp::sig::Vector& CoM_d,
         const yarp::sig::Vector& q_d,
         const Vector &zmp_d)
{
    log_com.log(CoM);
    log_l_foot.log(LFoot.getCol(3).subVector(0,2));
    log_r_foot.log(RFoot.getCol(3).subVector(0,2));
    log_pelvis.log(Pelvis.getCol(3).subVector(0,2));
    log_q.log(q);
    log_com_d.log(CoM_d);
    log_l_foot_d.log(LFoot_d.getCol(3).subVector(0,2));
    log_r_foot_d.log(RFoot_d.getCol(3).subVector(0,2));
    log_pelvis_d.log(Pelvis_d.getCol(3).subVector(0,2));
    log_q_d.log(q_d);
    log_zmp_d.log(zmp_d);
}

boost::shared_ptr<walking_problem::ik_problem> walking_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    taskRFoot.reset();
    taskLFoot.reset();
    taskPelvis.reset();
    taskPostural.reset();
    taskTorso.reset();

    problem->stack_of_tasks.clear();
    problem->bounds.reset();

    /** Create Tasks **/
    taskRFoot.reset(new Cartesian("cartesian::RFoot",state,robot_model,
        "r_ankle","world"));
    std::cout<<"RFootRef:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(taskRFoot->getReference());
    taskRFoot->setLambda(LAMBDA_GAIN);

    taskLFoot.reset(new Cartesian("cartesian::LFoot",state,robot_model,
        "l_ankle","world"));
    std::cout<<"LFootRef:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(taskLFoot->getReference());
    taskLFoot->setLambda(LAMBDA_GAIN);

    taskPelvis.reset(new Cartesian("cartesian::Waist", state, robot_model,
        "Waist", "world"));
    std::cout<<"PelvisRef:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(taskPelvis->getReference());
    taskPelvis->setLambda(LAMBDA_GAIN);

    taskTorso.reset(new Cartesian("cartesian::Torso", state, robot_model,
        "torso", "world"));
    std::cout<<"TorsoRef:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(taskTorso->getReference());
    taskTorso->setLambda(LAMBDA_GAIN);
    yarp::sig::Matrix WTorso(6,6); WTorso.eye();
    WTorso(0,0) = 0.0; WTorso(1,1) = 0.0; WTorso(2,2) = 0.0;
    taskTorso->setWeight(WTorso);
    std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
    taskTorso->setActiveJointsMask(active_joint_mask);

    taskCoM.reset(new CoM(state, robot_model));
    yarp::sig::Matrix W(3,3); W.eye(); W(2,2) = 0.0;
    taskCoM->setWeight(COM_GAIN*W);

    taskRArm.reset(new Cartesian("cartesian::RArm", state, robot_model,
                                 "RSoftHand", "Waist"));
    active_joint_mask = taskRArm->getActiveJointsMask();
    for(unsigned int i = 0; i < robot_model.torso.getNrOfDOFs(); ++i)
        active_joint_mask[robot_model.torso.joint_numbers[i]] = false;
    taskRArm->setActiveJointsMask(active_joint_mask);
    taskRArm->setLambda(0.05*LAMBDA_GAIN);
    InitialRArmRef = taskRArm->getActualPose();
    std::cout<<"Initial RArmRef:"<<std::endl;
    cartesian_utils::printHomogeneousTransform(InitialRArmRef);

    yarp::sig::Vector com_ref(3,0.0);
    com_ref[0] = comRef(0);
    com_ref[1] = comRef(1);
    com_ref[2] = comRef(2); //1.13
    taskCoM->setReference(com_ref);
    std::cout<<"CoMRef:"<<std::endl;
    std::cout<<taskCoM->getReference()[0]<<"  "<<taskCoM->getReference()[1]<<"  "<<taskCoM->getReference()[2]<<std::endl;
    taskCoM->setLambda(LAMBDA_GAIN);

    taskPostural.reset(new Postural(state));
    taskPostural->setLambda(LAMBDA_GAIN);
    /** Create bounds **/
    JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
        robot_model.iDyn3_model.getJointBoundMax(), robot_model.iDyn3_model.getJointBoundMin())));
    VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(
        new VelocityLimits(M_PI, mSecToSec(dT), state.size())));


    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskRFoot);
    taskList.push_back(taskLFoot);
    //taskList.push_back(taskPelvis);
    taskList.push_back(taskCoM);
    taskList.push_back(taskTorso);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

//    taskList.clear();
//    taskList.push_back(taskRArm);
//    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
//        new OpenSoT::tasks::Aggregated(taskList, state.size())));

    taskList.clear();
    taskList.push_back(taskPostural);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
    bounds.push_back(boundJointLimits);
    bounds.push_back(boundsJointVelLimits);
    problem->bounds = OpenSoT::constraints::Aggregated::Ptr(
        new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E10;

return problem;
}

boost::shared_ptr<walking_problem::ik_problem> walking_problem::homing_problem(const yarp::sig::Vector& state,
                    iDynUtils& robot_model,const double dT,
                    const std::string& name_space)
{
    /** CH **/
    ConvexHull::Ptr ch = ConvexHull::Ptr(new ConvexHull(state, robot_model, 0.06));

    yarp::sig::Matrix rsole_T_world = robot_model.iDyn3_model.getPosition(
                robot_model.iDyn3_model.getLinkIndex("r_sole"), true);
    yarp::sig::Matrix waist_T_rsole = yarp::math::pinv(rsole_T_world*pelvisRef);

    /** Create Tasks **/
    taskRFoot.reset(new Cartesian("cartesian::RFoot_Waist",state,robot_model,
        robot_model.right_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
    RFootReference(0,3) = waist_T_rsole(0,3);
    RFootReference(1,3) = RFootRef(1,3);
    RFootReference(2,3) = waist_T_rsole(2,3);
    //RFootReference(0,3) = 0.093; RFootReference(1,3) = -0.14; RFootReference(2,3) = -1.09;
    taskRFoot->setReference(RFootReference);
    taskRFoot->setLambda(LAMBDA_GAIN);
    std::cout<<"Homing RFoot: "<<std::endl; cartesian_utils::printHomogeneousTransform(RFootReference);


    yarp::sig::Matrix lsole_T_world = robot_model.iDyn3_model.getPosition(
                robot_model.iDyn3_model.getLinkIndex("l_sole"), true);
    yarp::sig::Matrix waist_T_lsole = yarp::math::pinv(lsole_T_world*pelvisRef);

    taskLFoot.reset(new Cartesian("cartesian::LFoot_Waist",state,robot_model,
        robot_model.left_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
    LFootReference(0,3) = waist_T_lsole(0,3);
    LFootReference(1,3) = LFootRef(1,3);
    LFootReference(2,3) = waist_T_lsole(2,3);
    //LFootReference(0,3) = 0.093; LFootReference(1,3) = 0.14; LFootReference(2,3) = -1.09;
    taskLFoot->setReference(LFootReference);
    taskLFoot->setLambda(LAMBDA_GAIN);
    std::cout<<"Homing LFoot: "<<std::endl; cartesian_utils::printHomogeneousTransform(LFootReference);

    Cartesian::Ptr taskTorso = Cartesian::Ptr(new Cartesian("cartesian::Torso_Waist", state, robot_model,
                                                            "torso", "world"));
    std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
    taskTorso->setLambda(LAMBDA_GAIN);
    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
    taskTorso->setActiveJointsMask(active_joint_mask);
    OpenSoT::SubTask::Ptr subTaskTorso = OpenSoT::SubTask::Ptr(
        new OpenSoT::SubTask(taskTorso, OpenSoT::SubTask::SubTaskMap::range(3,5)));

    taskPostural.reset(new Postural(state));
    yarp::sig::Vector q_postural(state.size(), 0.0);
    q_postural = state;
    q_postural[robot_model.left_leg.joint_numbers[3]] = 10.0*M_PI/180.0;
    q_postural[robot_model.right_leg.joint_numbers[3]] = 10.0*M_PI/180.0;
//    q_postural[robot_model.left_arm.joint_numbers[3]] = -20.0*M_PI/180.0;
//    q_postural[robot_model.right_arm.joint_numbers[3]] = -20.0*M_PI/180.0;
//    q_postural[robot_model.left_arm.joint_numbers[0]] = 5.0*M_PI/180.0;
//    q_postural[robot_model.right_arm.joint_numbers[0]] = 5.0*M_PI/180.0;
    taskPostural->setReference(q_postural);
    taskPostural->setLambda(LAMBDA_GAIN);

    /** Create bounds **/
    yarp::sig::Vector joint_bound_max = robot_model.iDyn3_model.getJointBoundMax();
    yarp::sig::Vector joint_bound_min = robot_model.iDyn3_model.getJointBoundMin();
    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
    {
        joint_bound_max[robot_model.left_leg.joint_numbers[i]] = M_PI;
        joint_bound_min[robot_model.left_leg.joint_numbers[i]] = -M_PI;
        joint_bound_max[robot_model.right_leg.joint_numbers[i]] = M_PI;
        joint_bound_min[robot_model.right_leg.joint_numbers[i]] = -M_PI;
    }
    JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
        joint_bound_max, joint_bound_min)));
    VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(
        new VelocityLimits(1.0, mSecToSec(dT), state.size())));

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(subTaskTorso);
    taskList.push_back(taskRFoot);
    taskList.push_back(taskLFoot);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));
    problem->stack_of_tasks[0]->getConstraints().push_back(ch);

    taskList.clear();
    taskList.push_back(taskPostural);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));
    problem->stack_of_tasks[1]->getConstraints().push_back(ch);

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
    bounds.push_back(boundJointLimits);
    bounds.push_back(boundsJointVelLimits);
    problem->bounds = OpenSoT::constraints::Aggregated::Ptr(
        new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E10;

return problem;
}

bool walking_problem::switchSupportFoot(iDynUtils& robot_model, const int trj_stance_foot)
{
    if(trj_stance_foot != stance_foot){
        std::string string_stance_foot;
        if(trj_stance_foot == STANCE_FOOT::LEFT_FOOT){
            robot_model.switchAnchorAndFloatingBase("l_ankle");
            stance_foot = STANCE_FOOT::LEFT_FOOT;
            string_stance_foot = "l_ankle";}
        if(trj_stance_foot == STANCE_FOOT::RIGHT_FOOT){
            robot_model.switchAnchorAndFloatingBase("r_ankle");
            stance_foot = STANCE_FOOT::RIGHT_FOOT;
            string_stance_foot = "r_ankle";}

        geometry_msgs::TransformStamped T;
        T.header.frame_id = string_stance_foot;
        T.child_frame_id = "world";

        KDL::Frame anchor = robot_model.getAnchor_T_World();
        double qx,qy,qz,qw;
        anchor.M.GetQuaternion(qx,qy,qz,qw);
        T.transform.rotation.x = qx;
        T.transform.rotation.y = qy;
        T.transform.rotation.z = qz;
        T.transform.rotation.w = qw;
        T.transform.translation.x = anchor.p[0];
        T.transform.translation.y = anchor.p[1];
        T.transform.translation.z = anchor.p[2];
        new_world_pub.publish(T);
    }

    //robot_model.iDyn3_model.getSensorMeasurement(_ft_index, wrench_in_sensor_frame);
}

void walking_problem::generateFootSteps(const int number_of_steps, const double step_width, const double step_lenght)
{
    footPositions = MatrixXd::Zero(7, number_of_steps);

    double foot = R_FOOT;
    Vector3d footPos = Vector3d::Zero();

    footPos(1) = step_width/2.0;
    for (unsigned int i = 0; i < number_of_steps; ++i) {
        if (i < number_of_steps-1)
            footPos(0) += step_lenght;
        if (foot == R_FOOT)
            footPos(1) += -step_width;
       else
            footPos(1) += step_width;
      footPositions.block(0,i,3,1) = footPos;
      footPositions(6,i) = foot;
      foot == R_FOOT ? foot = L_FOOT : foot = R_FOOT;
    }

    stepMatrix2StepInfo(footPositions, &stepsInfo);
}

bool walking_problem::walkingPatternGeneration(const double step_time, const int number_of_steps,
                                               const double step_width, const double step_lenght)
{
    generateFootSteps(number_of_steps, step_width, step_lenght);
    if ( !pattern_generator->generatePattern(stepsInfo,step_time) ) {
        ROS_ERROR(" Failed generating pattern!");
        return false;
    }
    walking_pattern_finished = false;
    return true;
}

bool walking_problem::updateWalkingPattern(yarp::sig::Matrix& LFootRef, yarp::sig::Matrix& RFootRef,
                                           yarp::sig::Matrix& PelvisRef, yarp::sig::Vector& CoMRef, Vector &ZMPRef,
                                           int& stance_foot)
{
    //For now we fake all the measurements
    Eigen::VectorXd zero31(31), zero12(12), zero3(3);
    for(unsigned int i = 0; i < 29; ++i){
        if(i < 3) zero3(i) = 0.0;
        if(i < 12) zero12(i) = 0.0;
        zero31(i) = 0.0;}
    pattern_generator->updateRobotState(zero31, zero31, zero31, zero12, zero3, zero3);

    Eigen::VectorXd lFootRef(6), rFootRef(6), comRef(3), pelvisRef(6);
    Eigen::Vector3d zmpRef;
    bool success = pattern_generator->getNewTaskReference(&lFootRef, &rFootRef, &comRef, &pelvisRef,
                                                          &zmpRef);

    if(!success){
        walking_pattern_finished = true;
        return false;}

    KDL::Frame l_foot_ref, r_foot_ref, pelvis_ref;
    l_foot_ref = l_foot_ref.Identity();
    r_foot_ref = r_foot_ref.Identity();
    pelvis_ref = pelvis_ref.Identity();

    l_foot_ref = eigen2KDL(lFootRef);
    r_foot_ref = eigen2KDL(rFootRef);
    pelvis_ref = eigen2KDL(pelvisRef);

    LFootRef.resize(4,4);
    RFootRef.resize(4,4);
    PelvisRef.resize(4,4);
    CoMRef.resize(3);

    cartesian_utils::fromKDLFrameToYARPMatrix(l_foot_ref, LFootRef);
    cartesian_utils::fromKDLFrameToYARPMatrix(r_foot_ref, RFootRef);
    cartesian_utils::fromKDLFrameToYARPMatrix(pelvis_ref, PelvisRef);
    CoMRef[0] = comRef[0]; CoMRef[1] = comRef[1]; CoMRef[2] = comRef[2];

    ZMPRef.resize(3);
    ZMPRef[0] = zmpRef[0]; ZMPRef[1] = zmpRef[1]; ZMPRef[2] = zmpRef[2];


    if(ROBOT::g_feetState.getFootState(ROBOT::whichLeg::left) == footState::stance)
        stance_foot = STANCE_FOOT::LEFT_FOOT;
    else
        stance_foot = STANCE_FOOT::RIGHT_FOOT;

    return true;
}




