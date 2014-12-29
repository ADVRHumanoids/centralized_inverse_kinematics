#include <openSoTServer.h>
#include <boost/shared_ptr.hpp>

#define mSecToSec(X) (X*0.001)

openSoTServer::openSoTServer()
{

}

void openSoTServer::create_problem(const yarp::sig::Vector& state, iDynUtils& robot_model, const double dT,
                                   const std::string& name_space)
{
    ///TASK RSOLE
    taskCartesianRSole = boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
                            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_sole",state,robot_model,
                                                                    robot_model.right_leg.end_effector_name,
                                                                    "world"));
    YRSoleCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                                                 taskCartesianRSole));

    ///TASK WAIST
    taskCartesianWaist = boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
                            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",state,robot_model,
                                                                    "Waist",
                                                                    "world"));
    YWaistCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                                                 taskCartesianWaist));

    ///TASK TORSO
    taskCartesianTorso = boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
    new OpenSoT::tasks::velocity::Cartesian("cartesian::torso",state,robot_model,
                                            "torso",
                                            "world"));
    YTorsoCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
    new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                         taskCartesianTorso));
    std::vector<bool> active_joint_mask = taskCartesianTorso->getActiveJointsMask();
    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
    taskCartesianTorso->setActiveJointsMask(active_joint_mask);
    yarp::sig::Matrix W_torso(6,6); W_torso = W_torso.eye();
    W_torso(0,0) = 0.0; W_torso(1,1) = 0.0; W_torso(2,2) = 0.0;
    YTorsoCartesian->taskCartesian->setWeight(W_torso);

    ///TASK POSTURAL
    taskPostural = boost::shared_ptr<OpenSoT::tasks::velocity::Postural>(new OpenSoT::tasks::velocity::Postural(state));
    YPostural = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>(
                new OpenSoT::interfaces::yarp::tasks::YPostural(robot_model.getRobotName(), name_space, robot_model,
                                                           taskPostural));
    ///CONSTRAINT CH
    constraintConvexHull = boost::shared_ptr<OpenSoT::constraints::velocity::ConvexHull>(
                new OpenSoT::constraints::velocity::ConvexHull(state, robot_model, 0.05));
    ///CONSTRAINT CoM VEL
    constraintCoMVelocity = boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity>(
                new OpenSoT::constraints::velocity::CoMVelocity(yarp::sig::Vector(3,0.03), mSecToSec(dT), state, robot_model));

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskPostural->getConstraints().push_back(constraintConvexHull);
    taskPostural->getConstraints().push_back(constraintCoMVelocity);
    taskList.push_back(taskPostural);
    _task0 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task0->getConstraints().push_back(constraintConvexHull);
//    _task0->getConstraints().push_back(constraintCoMVelocity);

    taskList.clear();
    taskCartesianWaist->getConstraints().push_back(constraintConvexHull);
    taskCartesianWaist->getConstraints().push_back(constraintCoMVelocity);
    taskList.push_back(taskCartesianWaist);
    taskList.push_back(taskCartesianTorso);
    _task1 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task1->getConstraints().push_back(constraintConvexHull);
//    _task1->getConstraints().push_back(constraintCoMVelocity);

    taskList.clear();
    taskCartesianRSole->getConstraints().push_back(constraintConvexHull);
    taskCartesianRSole->getConstraints().push_back(constraintCoMVelocity);
    taskList.push_back(taskCartesianRSole);
    _task2 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task2->getConstraints().push_back(constraintConvexHull);
//    _task2->getConstraints().push_back(constraintCoMVelocity);

    /** Bounds initialization **/
    boundsJointLimits = OpenSoT::constraints::velocity::JointLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::JointLimits(
                                state,
                                robot_model.iDyn3_model.getJointBoundMax(),
                                robot_model.iDyn3_model.getJointBoundMin()));
    boundsJointVelocity = OpenSoT::constraints::velocity::VelocityLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::VelocityLimits(0.1, mSecToSec(dT), state.size()));

    /** Create sot **/
    _stack_of_tasks.push_back(_task2);
    _stack_of_tasks.push_back(_task1);
    _stack_of_tasks.push_back(_task0);

    /** Create Aggregated Bounds **/
    _bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(
                    new OpenSoT::constraints::Aggregated(boundsJointLimits, boundsJointVelocity, state.size()));

    /** Solver Init **/
    _qpOasesSolver = OpenSoT::solvers::QPOases_sot::SolverPtr(
        new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _bounds, 2E2));
}

void openSoTServer::reset_tasks_and_constraints()
{
    /** Task & Constraint reset **/
    taskPostural.reset();
    taskCartesianRSole.reset();
    taskCartesianTorso.reset();
    taskCartesianWaist.reset();
    constraintConvexHull.reset();
    constraintCoMVelocity.reset();
    _task0.reset();
    _task1.reset();
    _task2.reset();
    _stack_of_tasks.clear();

    /** Bounds reset **/
    boundsJointLimits.reset();
    boundsJointVelocity.reset();
    _bounds.reset();

    /** Interface reset **/
    YPostural.reset();
    YRSoleCartesian.reset();
    YTorsoCartesian.reset();
    YWaistCartesian.reset();
}

void openSoTServer::reset_problem()
{
    reset_tasks_and_constraints();
    reset_solver();
}

void openSoTServer::reset_solver()
{
    _qpOasesSolver.reset();
}

void openSoTServer::update(const Vector &state)
{
    _task0->update(state);
    _task1->update(state);
    _task2->update(state);
    _bounds->update(state);
}

bool openSoTServer::solve(Vector &solution)
{
    return _qpOasesSolver->solve(solution);
}
