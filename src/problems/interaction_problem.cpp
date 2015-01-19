#include <problems/interaction_problem.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/tasks/velocity/Interaction.h>

#define mSecToSec(X) (X*0.001)

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

interaction_problem::interaction_problem():
    general_ik_problem()
{

}

boost::shared_ptr<interaction_problem::ik_problem> interaction_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    /** Create tasks **/
        /** 1) Force r_wrist **/
        Interaction::Ptr taskRWrist(Interaction::Ptr(new Interaction("interaction::r_wrist",state,robot_model,
                                                          "r_wrist", "world", "r_arm_ft")));
        yarp::sig::Matrix C(6,6); C = C.eye();
        C.submatrix(0,2,0,2) = 1E-8 * C.submatrix(0,2,0,2);
        C.submatrix(3,5,3,5) = 1E-8 * C.submatrix(3,5,3,5);
        taskRWrist->setCompliance(C);
            /** 1.1) We want to control r_wrist in /world frame using only the joints in the arms **/
            std::vector<bool> active_joint_mask = taskRWrist->getActiveJointsMask();
            for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
                active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
            taskRWrist->setActiveJointsMask(active_joint_mask);

        yarp::sig::Vector desired_wrench(6, 0.0); desired_wrench[0] = 25.0; desired_wrench[1] = 10.0; desired_wrench[2] = 10.0;
        taskRWrist->setReferenceWrench(desired_wrench);

        /** 2) Postural **/
        Postural::Ptr taskPostural(Postural::Ptr(new Postural(state)));

    /** Create bounds **/
        /** 1) bounds joint limits **/
        JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
                                                                            robot_model.iDyn3_model.getJointBoundMax(),
                                                                            robot_model.iDyn3_model.getJointBoundMin())));
        /** 2) bounds joint velocities **/
        VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(new VelocityLimits(0.1, mSecToSec(dT), state.size())));


    /** Create Augmented (aggregated) tasks  and stack of tasks**/
        /** 1) Higher priority Stack **/
        std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
        taskList.push_back(taskRWrist);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));

        /** 2) Second stack **/
        taskList.clear();
        taskList.push_back(taskPostural);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));

    /** Add bounds to problem **/
        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
        bounds.push_back(boundJointLimits);
        bounds.push_back(boundsJointVelLimits);
        problem->bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E2;

    return problem;
}
