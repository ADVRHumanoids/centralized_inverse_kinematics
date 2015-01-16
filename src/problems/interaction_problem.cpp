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
    /** Create Constraints **/
        /** 1) Constraint Convex Hull **/
        ConvexHull::Ptr constraintConvexHull(ConvexHull::Ptr(new ConvexHull(state, robot_model, 0.06)));

        /** 2) CoM Velocity **/
        CoMVelocity::Ptr constraintCoMVel(CoMVelocity::Ptr(new CoMVelocity(Vector(3,0.01), mSecToSec(dT), state, robot_model)));

    /** Create tasks **/
        /** 1) Force r_wrist **/
        Interaction::Ptr taskRWrist(Interaction::Ptr(new Interaction("interaction::r_sole",state,robot_model,
                                                          "r_wrist","world", )));

        /** 2) Cartesian Waist **/
        Cartesian::Ptr taskWaist(Cartesian::Ptr(new Cartesian("cartesian::Waist",state,robot_model,"Waist","world")));

        /** 3) Cartesian Torso **/
        Cartesian::Ptr taskTorso(Cartesian::Ptr(new Cartesian("cartesian::torso",state,robot_model,"torso","world")));
            /** 3.1) We want to control torso in /world frame using only the three joints in the torso **/
            std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
            for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
                active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
            taskTorso->setActiveJointsMask(active_joint_mask);
            /** 3.2) We are interested only in the orientation of the torso **/
            yarp::sig::Matrix W_torso(6,6); W_torso = W_torso.eye();
            W_torso(0,0) = 0.0; W_torso(1,1) = 0.0; W_torso(2,2) = 0.0;
            taskTorso->setWeight(W_torso);

        /** 4) Postural **/
        Postural::Ptr taskPostural(Postural::Ptr(new Postural(state)));
        taskPostural->setLambda(0.0);

        /** 5) Mininimize Acceleration **/
        //MinimizeAcceleration::Ptr taskMinimizeAcceleration(MinimizeAcceleration::Ptr(new MinimizeAcceleration(state)));

    /** Associate interfaces to tasks **/
        YRSoleCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                        new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space, taskRSole));
        YWaistCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                        new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space, taskWaist));
        YTorsoCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                        new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space, taskTorso));
        YPostural = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>(
                        new OpenSoT::interfaces::yarp::tasks::YPostural(robot_model.getRobotName(), name_space, robot_model,
                                                                        taskPostural));
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
        taskList.push_back(taskRSole);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
            /** 1.1) Add constraints to the stack **/
            problem->stack_of_tasks[0]->getConstraints().push_back(constraintConvexHull);
            problem->stack_of_tasks[0]->getConstraints().push_back(constraintCoMVel);

        /** 2) Second stack **/
        taskList.clear();
        taskList.push_back(taskWaist);
        taskList.push_back(taskTorso);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
            /** 2.1) Add constraints to the stack **/
            problem->stack_of_tasks[1]->getConstraints().push_back(constraintConvexHull);
            problem->stack_of_tasks[1]->getConstraints().push_back(constraintCoMVel);



        /** 3) Third stack **/
        taskList.clear();
        taskList.push_back(taskPostural);
        //taskList.push_back(taskMinimizeAcceleration);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
            /** 3.1) Add constraints to the stack **/
            problem->stack_of_tasks[2]->getConstraints().push_back(constraintConvexHull);
            problem->stack_of_tasks[2]->getConstraints().push_back(constraintCoMVel);

    /** Add bounds to problem **/
        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
        bounds.push_back(boundJointLimits);
        bounds.push_back(boundsJointVelLimits);
        problem->bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E2;

    return problem;
}
