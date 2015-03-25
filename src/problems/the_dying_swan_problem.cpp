#include <problems/the_dying_swan_problem.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/SubTask.h>


#define mSecToSec(X) (X*0.001)

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::interfaces::yarp::tasks;
using namespace yarp::sig;

the_dying_swan_problem::the_dying_swan_problem():
    general_ik_problem()
{}

boost::shared_ptr<the_dying_swan_problem::ik_problem> the_dying_swan_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    /** Create tasks **/
        /** 1) Cartesian RSole && Cartesian RSoftHand**/
        taskRSole = Cartesian::Ptr(new Cartesian("cartesian::r_sole",state,robot_model, "r_sole","world"));

        taskRWrist = Cartesian::Ptr(new Cartesian("cartesian::RSoftHand",state,robot_model, "RSoftHand","world"));
        taskRWrist->setOrientationErrorGain(0.1);
        taskRWrist_interface = YCartesian::Ptr(new YCartesian(robot_model.getRobotName(), name_space, taskRWrist));

        taskCoM = CoM::Ptr(new CoM(state, robot_model));
        taskCoM_interface = YCoM::Ptr(new YCoM(robot_model.getRobotName(), name_space, taskCoM));

        taskTorsoHead = Cartesian::Ptr(new Cartesian("cartesian::TorsoHead", state, robot_model, "gaze", "world"));
        taskTorsoHead_interface = YCartesian::Ptr(new YCartesian(robot_model.getRobotName(), name_space, taskTorsoHead));
        std::vector<bool> active_joint_mask = taskTorsoHead->getActiveJointsMask();
        for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
            active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
        taskTorsoHead->setActiveJointsMask(active_joint_mask);
        OpenSoT::SubTask::Ptr subTaskTorsoHead_x(
                    new OpenSoT::SubTask(taskTorsoHead, OpenSoT::SubTask::SubTaskMap::range(3,3)));
        OpenSoT::SubTask::Ptr subTaskTorsoHead_z(
                    new OpenSoT::SubTask(taskTorsoHead, OpenSoT::SubTask::SubTaskMap::range(5,5)));

        /** 2) Postural **/
        Postural::Ptr taskPostural(Postural::Ptr(new Postural(state)));
        taskPostural_interface = YPostural::Ptr(new YPostural(robot_model.getRobotName(), name_space,
                                                              robot_model, taskPostural));

    /** Create bounds **/
        /** 1) bounds joint limits **/
        JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
                                                                            robot_model.iDyn3_model.getJointBoundMax(),
                                                                            robot_model.iDyn3_model.getJointBoundMin())));
        /** 2) bounds joint velocities **/
        VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(new VelocityLimits(0.3, mSecToSec(dT), state.size())));


    /** Create Augmented (aggregated) tasks  and stack of tasks**/
        /** 1) Higher priority Stack **/
        std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
        taskList.push_back(taskRSole);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));

        /** 2) Second priority Stack **/
        taskList.clear();
        taskList.push_back(taskCoM);
        taskList.push_back(taskRWrist);
        taskList.push_back(subTaskTorsoHead_x);
        taskList.push_back(subTaskTorsoHead_z);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));

        /** 3) Third stack **/
        taskList.clear();
        taskList.push_back(taskPostural);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));

    /** Add bounds to problem **/
        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
        bounds.push_back(boundJointLimits);
        bounds.push_back(boundsJointVelLimits);
        problem->bounds = OpenSoT::constraints::Aggregated::Ptr(new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E2;

    return problem;
}
