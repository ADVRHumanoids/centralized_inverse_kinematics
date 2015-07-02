#include <problems/walking_problem.h>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>
#include <OpenSoT/SubTask.h>

#define mSecToSec(X) (X*0.001)

#define LAMBDA_GAIN 1.0

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

walking_problem::walking_problem(iDynUtils& robot_model):
    general_ik_problem(),
    n(),
    _robot_model(robot_model)
{
    file_l_footd.open("positions_ref_l_foot.m");
    file_r_footd.open("positions_ref_r_foot.m");
    file_pelvisd.open("positions_ref_pelvis.m");
    file_comd.open("positions_ref_com.m");
    file_q_ref.open("q_reference.m");

    file_l_foot.open("positions_sot_l_foot.m");
    file_r_foot.open("positions_sot_r_foot.m");
    file_pelvis.open("positions_sot_pelvis.m");
    file_com.open("positions_sot_com.m");
    file_q.open("q_measured.m");

    file_l_footd<<"ref_l_foot = ["<<std::endl;
    file_r_footd<<"ref_r_foot = ["<<std::endl;
    file_pelvisd<<"ref_pelvis = ["<<std::endl;
    file_comd<<"ref_com = ["<<std::endl;
    file_q_ref<<"q_ref = ["<<std::endl;

    file_l_foot<<"sot_l_foot = ["<<std::endl;
    file_r_foot<<"sot_r_foot = ["<<std::endl;
    file_pelvis<<"sot_pelvis = ["<<std::endl;
    file_com<<"sot_com = ["<<std::endl;
    file_q<<"q = ["<<std::endl;

    walking_pattern_finished = true;
    homing_done = false;
    reset_solver = false;
    start_walking_pattern = false;

    //We start from left foot according to centralized_inverse_kinematics_thread.cpp
    stance_foot = STANCE_FOOT::LEFT_FOOT;

    new_world_pub = n.advertise<geometry_msgs::TransformStamped>("/anchor_to_world_pose", 1000);
}

walking_problem::~walking_problem()
{
    file_l_footd<<"];"<<std::endl;
    file_r_footd<<"];"<<std::endl;
    file_pelvisd<<"];"<<std::endl;
    file_comd<<"];"<<std::endl;
    file_q_ref<<"];"<<std::endl;
    file_l_footd.close();
    file_r_footd.close();
    file_pelvisd.close();
    file_comd.close();
    file_q_ref.close();

    file_l_foot<<"];"<<std::endl;
    file_r_foot<<"];"<<std::endl;
    file_pelvis<<"];"<<std::endl;
    file_com<<"];"<<std::endl;
    file_q<<"];"<<std::endl;
    file_l_foot.close();
    file_r_foot.close();
    file_pelvis.close();
    file_com.close();
    file_q.close();
}

void walking_problem::log(const yarp::sig::Matrix& LFootRef, const yarp::sig::Matrix& RFootRef,
                          const yarp::sig::Matrix &PelvisRef, const yarp::sig::Vector& CoMRef,
                          const yarp::sig::Vector& q_reference, const yarp::sig::Vector& q_measured)
{
    KDL::Frame tmp;
    double roll, pitch, yaw;

    cartesian_utils::fromYARPMatrixtoKDLFrame(LFootRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_l_footd<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(RFootRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_r_footd<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(PelvisRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_pelvisd<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    file_comd<<CoMRef[0]<<"  "<<CoMRef[1]<<"  "<<CoMRef[2]<<std::endl;

    file_q_ref<<q_reference.toString()<<std::endl;




    cartesian_utils::fromYARPMatrixtoKDLFrame(taskLFoot->getActualPose(), tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_l_foot<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(taskRFoot->getActualPose(), tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_r_foot<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(taskPelvis->getActualPose(), tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_pelvis<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    file_com<<taskCoM->getActualPosition()[0]<<"  "<<taskCoM->getActualPosition()[1]<<"  "<<taskCoM->getActualPosition()[2]<<std::endl;

    file_q<<q_measured.toString()<<std::endl;
}

boost::shared_ptr<walking_problem::ik_problem> walking_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    taskRFoot.reset();
    taskLFoot.reset();
    taskPelvis.reset();
    taskPostural.reset();

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

    taskCoM.reset(new CoM(state, robot_model));
    yarp::sig::Matrix W(3,3); W.eye(); W(2,2) = 0.0;
    taskCoM->setWeight(0.5*W);

    yarp::sig::Vector com_ref(3,0.0);
    com_ref[0] = 0.0;
    com_ref[1] = 0.0;
    com_ref[2] = 1.13;
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
        new VelocityLimits(M_PI_2, mSecToSec(dT), state.size())));


    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskRFoot);
    taskList.push_back(taskLFoot);
    taskList.push_back(taskPelvis);
    taskList.push_back(taskCoM);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

//    taskList.clear();
//    taskList.push_back(taskCoM);
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

    /** Create Tasks **/
    taskRFoot.reset(new Cartesian("cartesian::RFoot_Waist",state,robot_model,
        robot_model.right_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
    RFootReference(0,3) = 0.093; RFootReference(1,3) = -0.14; RFootReference(2,3) = -1.09;
    taskRFoot->setReference(RFootReference);
    //taskRFoot->setOrientationErrorGain(0.3);
    taskRFoot->setLambda(LAMBDA_GAIN);

    taskLFoot.reset(new Cartesian("cartesian::LFoot_Waist",state,robot_model,
        robot_model.left_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
    LFootReference(0,3) = 0.093; LFootReference(1,3) = 0.14; LFootReference(2,3) = -1.09;
    taskLFoot->setReference(LFootReference);
    //taskLFoot->setOrientationErrorGain(0.3);
    taskLFoot->setLambda(LAMBDA_GAIN);

    Cartesian::Ptr taskTorso = Cartesian::Ptr(new Cartesian("cartesian::Torso_Waist", state, robot_model,
                                                            "torso", "world"));
    std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
    //taskTorso->setOrientationErrorGain(0.3);
    taskTorso->setLambda(LAMBDA_GAIN);
    for(unsigned int i = 0; i < 6; ++i)
        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
    taskTorso->setActiveJointsMask(active_joint_mask);
    OpenSoT::SubTask::Ptr subTaskTorso = OpenSoT::SubTask::Ptr(
        new OpenSoT::SubTask(taskTorso, OpenSoT::SubTask::SubTaskMap::range(3,5)));

    taskPostural.reset(new Postural(state));
    yarp::sig::Vector q_postural(state.size(), 0.0);
    q_postural[robot_model.left_leg.joint_numbers[3]] = 10.0*M_PI/180.0;
    q_postural[robot_model.right_leg.joint_numbers[3]] = 10.0*M_PI/180.0;
    q_postural[robot_model.left_arm.joint_numbers[3]] = -20.0*M_PI/180.0;
    q_postural[robot_model.right_arm.joint_numbers[3]] = -20.0*M_PI/180.0;
    q_postural[robot_model.left_arm.joint_numbers[0]] = 5.0*M_PI/180.0;
    q_postural[robot_model.right_arm.joint_numbers[0]] = 5.0*M_PI/180.0;
    taskPostural->setReference(q_postural);
    taskPostural->setLambda(LAMBDA_GAIN);

    /** Create bounds **/
    yarp::sig::Vector joint_bound_max = robot_model.iDyn3_model.getJointBoundMax();
    yarp::sig::Vector joint_bound_min = robot_model.iDyn3_model.getJointBoundMin();
    for(unsigned int i = 0; i < 6; ++i)
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
}

bool walking_problem::walkingPatternGeneration(const double step_time, const int number_of_steps,
                                               const double step_width, const double step_lenght)
{
    generateFootSteps(number_of_steps, step_width, step_lenght);
    if ( !pattern_generator->generatePattern(footPositions,step_time) ) {
        ROS_ERROR(" Failed generating pattern!");
        return false;
    }
    walking_pattern_finished = false;
    return true;
}

bool walking_problem::updateWalkingPattern(yarp::sig::Matrix& LFootRef, yarp::sig::Matrix& RFootRef,
                                           yarp::sig::Matrix& PelvisRef, yarp::sig::Vector& CoMRef,
                                           int& stance_foot)
{
    //For now we fake all the measurements
    Eigen::VectorXd zero31(31), zero12(12), zero3(3);
    for(unsigned int i = 0; i < 29; ++i){
        if(i < 3) zero3(i) = 0.0;
        if(i < 12) zero12(i) = 0.0;
        zero31(i) = 0.0;}
    pattern_generator->updateRobotState(zero31, zero31, zero31, zero12, zero3);

    Eigen::VectorXd lFootRef(6), rFootRef(6), comRef(3), pelvisRef(6);
    bool success = pattern_generator->getNewTaskReference(&lFootRef, &rFootRef, &comRef, &pelvisRef);

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


    if(ROBOT::g_feetState.getFootState(ROBOT::whichLeg::left) == footState::stance)
        stance_foot = STANCE_FOOT::LEFT_FOOT;
    else
        stance_foot = STANCE_FOOT::RIGHT_FOOT;

    return true;
}




