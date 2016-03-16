#include <problems/wb_manip_problem.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/SubTask.h>

#define mSecToSec(X) (X*0.001)

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::interfaces::yarp::tasks;
using namespace yarp::sig;

wb_manip_problem::wb_manip_problem(iDynUtils &robot_model, string urdf_path, string srdf_path):
    general_ik_problem(robot_model, urdf_path, srdf_path)
{}

wb_manip_problem::~wb_manip_problem()
{}

boost::shared_ptr<wb_manip_problem::ik_problem> wb_manip_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    double lambda = 100.0;

    /** Create Constraints **/
        /** 1) Constraint Convex Hull **/
        ConvexHull::Ptr constraintConvexHull(ConvexHull::Ptr(new ConvexHull(state, robot_model, 0.05)));

        /** 2) CoM Velocity **/
        CoMVelocity::Ptr constraintCoMVel(CoMVelocity::Ptr(new CoMVelocity(Vector(3,0.01), mSecToSec(dT),
                                                                           state, robot_model)));

    /** Create Tasks **/
        /** 1) Cartesian R_Wrist & L_Wrist**/
        taskRWrist = Cartesian::Ptr(new Cartesian("cartesian::r_arm", state, robot_model,
                                                  robot_model.right_arm.end_effector_name, "world"));
        taskRWrist->setOrientationErrorGain(0.5);
        taskRWrist->setLambda(lambda);
        taskLWrist = Cartesian::Ptr(new Cartesian("cartesian::l_arm", state, robot_model,
                                                  robot_model.left_arm.end_effector_name, "world"));
        taskLWrist->setOrientationErrorGain(0.5);
        taskLWrist->setLambda(lambda);

        /** 2) Cartesian Torso **/
        Cartesian::Ptr taskTorso(Cartesian::Ptr(new Cartesian("cartesian::torso",state,robot_model,"torso","world")));
        taskTorso->setOrientationErrorGain(0.5);
        taskTorso->setLambda(lambda);
            /** 3.1) We want to control torso in /world frame using only the three joints in the torso **/
            std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
            for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
                active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
            taskTorso->setActiveJointsMask(active_joint_mask);

            /** 3.2) We are interested only in the orientation of the torso **/
            OpenSoT::SubTask::Ptr taskTorsoOrientation(OpenSoT::SubTask::Ptr(
                                new OpenSoT::SubTask(taskTorso, OpenSoT::Indices::range(3,5))));


        /** 2) Cartesian RSole **/
        taskRSole = Cartesian::Ptr(new Cartesian("cartesian::r_sole",state,robot_model,
                                                  robot_model.right_leg.end_effector_name,"world"));
        taskRSole->setLambda(lambda);
        /** 2) Cartesian Waist **/
        Cartesian::Ptr taskWaist(Cartesian::Ptr(new Cartesian("cartesian::Waist",state,robot_model,"Waist","world")));
        taskWaist->setOrientationErrorGain(0.5);
        taskWaist->setLambda(lambda);
            /** 3.2) We are interested only in the orientation**/
            OpenSoT::SubTask::Ptr taskWaistOrientation(OpenSoT::SubTask::Ptr(
                                new OpenSoT::SubTask(taskWaist, OpenSoT::Indices::range(3,5))));

        /** 3) Minimize Joint Acc **/
        MinimizeAcceleration::Ptr taskMinimizeAcceleration(MinimizeAcceleration::Ptr(new MinimizeAcceleration(state)));

        /** 4) Postural **/
        Postural::Ptr taskPostural(Postural::Ptr(new Postural(state)));
        taskPostural->setLambda(lambda);

     /** Associate interfaces to tasks **/
        YRSoleCartesian = YCartesian::Ptr(new YCartesian(robot_model.getRobotName(), name_space, taskRSole));
        YRWristCartesian = YCartesian::Ptr(new YCartesian(robot_model.getRobotName(), name_space, taskRWrist));
        YLWristCartesian = YCartesian::Ptr(new YCartesian(robot_model.getRobotName(), name_space, taskLWrist));

    /** Create bounds **/
        /** 1) bounds joint limits **/
        JointLimits::ConstraintPtr boundJointLimits(JointLimits::ConstraintPtr(new JointLimits(state,
                                                                robot_model.iDyn3_model.getJointBoundMax(),
                                                                robot_model.iDyn3_model.getJointBoundMin())));
        /** 2) bounds joint velocities **/
        VelocityLimits::ConstraintPtr boundsJointVelLimits(VelocityLimits::ConstraintPtr(new VelocityLimits(1.0,
                                                                mSecToSec(dT), state.size())));

    /** Create Augmented (aggregated) tasks  and stack of tasks**/
        std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
        /** 1) Higher priority Stack **/
        taskList.push_back(taskRSole);
        taskList.push_back(taskTorsoOrientation);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                              new OpenSoT::tasks::Aggregated(taskList, state.size())));
        taskList.clear();
        /** 2) Second Stack **/
        taskList.push_back(taskRWrist);
        taskList.push_back(taskLWrist);
        taskList.push_back(taskWaistOrientation);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                              new OpenSoT::tasks::Aggregated(taskList, state.size())));
        taskList.clear();
        /** 3) Third Stack **/
        taskList.push_back(taskMinimizeAcceleration);
        taskList.push_back(taskPostural);
        problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(
                                              new OpenSoT::tasks::Aggregated(taskList, state.size())));
        taskList.clear();
        /** Add constraints to all the stacks **/
        for(unsigned int i = 0; i < problem->stack_of_tasks.size(); ++i){
            problem->stack_of_tasks[i]->getConstraints().push_back(constraintConvexHull);
            problem->stack_of_tasks[i]->getConstraints().push_back(constraintCoMVel);}

    /** Add bounds to problem **/
        std::list<OpenSoT::constraints::Aggregated::ConstraintPtr> bounds;
        bounds.push_back(boundJointLimits);
        bounds.push_back(boundsJointVelLimits);
        problem->bounds = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(bounds, state.size()));

    /** Set damped leas squares fator **/
    problem->damped_least_square_eps = 2E2;

    return problem;
}


