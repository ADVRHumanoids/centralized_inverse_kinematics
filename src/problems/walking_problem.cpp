#include <problems/walking_problem.h>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>

#define mSecToSec(X) (X*0.001)

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

walking_problem::walking_problem():
    general_ik_problem(),
    n()
{
    file_l_foot.open("positions_ref_l_foot.m");
    file_r_foot.open("positions_ref_r_foot.m");
    file_pelvis.open("positions_ref_pelvis.m");

    file_l_foot<<"ref_l_foot = ["<<std::endl;
    file_r_foot<<"ref_r_foot = ["<<std::endl;
    file_pelvis<<"ref_pelvis = ["<<std::endl;

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
    file_l_foot<<"]"<<std::endl;
    file_r_foot<<"]"<<std::endl;
    file_pelvis<<"]"<<std::endl;
    file_l_foot.close();
    file_r_foot.close();
    file_pelvis.close();
}

void walking_problem::log(const yarp::sig::Matrix& LFootRef, const yarp::sig::Matrix& RFootRef,
                          const yarp::sig::Matrix &PelvisRef)
{
    KDL::Frame tmp;
    double roll, pitch, yaw;

    cartesian_utils::fromYARPMatrixtoKDLFrame(LFootRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_l_foot<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(RFootRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_r_foot<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;

    cartesian_utils::fromYARPMatrixtoKDLFrame(PelvisRef, tmp);
    tmp.M.GetRPY(roll, pitch, yaw);
    file_pelvis<<tmp.p[0]<<"  "<<tmp.p[1]<<"  "<<tmp.p[2]<<"  "<<roll<<"  "<<"  "<<pitch<<"  "<<yaw<<std::endl;
}

boost::shared_ptr<walking_problem::ik_problem> walking_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    KDL::Frame l_ankle_T_World; l_ankle_T_World.Identity();
    l_ankle_T_World.p[0] = 0.0; l_ankle_T_World.p[1] = -0.14; l_ankle_T_World.p[2] = -0.143;
    robot_model.setAnchor_T_World(l_ankle_T_World);

    geometry_msgs::TransformStamped T;
    T.header.frame_id = "l_ankle";
    T.child_frame_id = "world";
    T.transform.rotation.x = 0.0;
    T.transform.rotation.y = 0.0;
    T.transform.rotation.z = 0.0;
    T.transform.rotation.w = 1.0;
    T.transform.translation.x = l_ankle_T_World.p[0];
    T.transform.translation.y = l_ankle_T_World.p[1];
    T.transform.translation.z = l_ankle_T_World.p[2];
    new_world_pub.publish(T);

    taskRFoot.reset();
    taskLFoot.reset();
    taskPelvis.reset();
    taskPostural.reset();

    problem->stack_of_tasks.clear();
    problem->bounds.reset();

    /** Create Tasks **/
    taskRFoot.reset(new Cartesian("cartesian::RFoot",state,robot_model,
        robot_model.right_leg.end_effector_name,"world"));
    taskLFoot.reset(new Cartesian("cartesian::LFoot",state,robot_model,
        robot_model.left_leg.end_effector_name,"world"));
    taskPelvis.reset(new Cartesian("cartesian::Waist", state, robot_model,
        "Waist", "world"));
    taskPostural.reset(new Postural(state));
    /** Create bounds **/
    JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
        robot_model.iDyn3_model.getJointBoundMax(), robot_model.iDyn3_model.getJointBoundMin())));
    VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(
        new VelocityLimits(1.0, mSecToSec(dT), state.size())));

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskRFoot);
    taskList.push_back(taskLFoot);
    taskList.push_back(taskPelvis);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

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
    problem->damped_least_square_eps = 2E2;

return problem;
}

boost::shared_ptr<walking_problem::ik_problem> walking_problem::homing_problem(const yarp::sig::Vector& state,
                    iDynUtils& robot_model,const double dT,
                    const std::string& name_space)
{
    /** Create Tasks **/
    taskRFoot.reset(new Cartesian("cartesian::RFoot_Waist",state,robot_model,
        robot_model.right_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix RFootReference(4,4); RFootReference = RFootReference.eye();
    RFootReference(0,3) = 0.093; RFootReference(1,3) = -0.14; RFootReference(2,3) = -1.09;
    taskRFoot->setReference(RFootReference);

    taskLFoot.reset(new Cartesian("cartesian::LFoot_Waist",state,robot_model,
        robot_model.left_leg.end_effector_name,"Waist"));
    yarp::sig::Matrix LFootReference(4,4); LFootReference = LFootReference.eye();
    LFootReference(0,3) = 0.093; LFootReference(1,3) = 0.14; LFootReference(2,3) = -1.09;
    taskLFoot->setReference(LFootReference);

    taskPostural.reset(new Postural(state));
    yarp::sig::Vector q_postural(state.size(), 0.0);
    /** Create bounds **/
    JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
        robot_model.iDyn3_model.getJointBoundMax(), robot_model.iDyn3_model.getJointBoundMin())));
    VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(
        new VelocityLimits(0.4, mSecToSec(dT), state.size())));

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskRFoot);
    taskList.push_back(taskLFoot);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

    taskList.clear();
    taskList.push_back(taskPostural);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
        new OpenSoT::tasks::Aggregated(taskList, state.size())));

    std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
    //bounds.push_back(boundJointLimits);
    bounds.push_back(boundsJointVelLimits);
    problem->bounds = OpenSoT::constraints::Aggregated::Ptr(
        new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E2;

return problem;
}

bool walking_problem::switchSupportFoot(iDynUtils& robot_model, const int trj_stance_foot)
{
    if(trj_stance_foot != stance_foot){
        if(trj_stance_foot == STANCE_FOOT::LEFT_FOOT)
            robot_model.switchAnchorAndFloatingBase(robot_model.left_leg.end_effector_name);
        if(trj_stance_foot == STANCE_FOOT::RIGHT_FOOT)
            robot_model.switchAnchorAndFloatingBase(robot_model.right_leg.end_effector_name);
    }
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




