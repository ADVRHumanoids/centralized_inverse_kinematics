#include <openSoTServer.h>
#include <boost/shared_ptr.hpp>

#define mSecToSec(X) (X*0.001)

openSoTServer::openSoTServer()
{

}

void openSoTServer::create_simple_walking_problem(const yarp::sig::Vector& state , 
						  iDynUtils &robot_model, 
						  const double dT, const string &name_space)
{
    // updating model : TODO: needed?
    robot_model.updateiDyn3Model(state, true);
    
    taskCartesianRAnkle = boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
                            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_ankle",state,robot_model,
                                                                    "RHipMot",
                                                                    "r_ankle"));
    YRAnkleCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                                                 taskCartesianRAnkle));
    
    taskCartesianLAnkle = boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
                            new OpenSoT::tasks::velocity::Cartesian("cartesian::l_ankle",state,robot_model,
                                                                    "LHipMot",
                                                                    "l_ankle"));
    
    YLAnkleCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                                                 taskCartesianLAnkle));
    
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskCartesianRAnkle);
    taskList.push_back(taskCartesianLAnkle);
    _task0 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
    
     /** Bounds initialization **/
    boundsJointLimits = OpenSoT::constraints::velocity::JointLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::JointLimits(
                                state,
                                robot_model.iDyn3_model.getJointBoundMax(),
                                robot_model.iDyn3_model.getJointBoundMin()));
    boundsJointVelocity = OpenSoT::constraints::velocity::VelocityLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::VelocityLimits(M_PI/2.0, mSecToSec(dT), state.size()));
    
    /** Create Aggregated Bounds **/
    _bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(
                    new OpenSoT::constraints::Aggregated(boundsJointLimits, boundsJointVelocity, state.size()));
    
    _stack_of_tasks.push_back(_task0);

    /** Solver Init **/
    _qpOasesSolver = OpenSoT::solvers::QPOases_sot::SolverPtr(
        new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _bounds, 2E2));
}

void openSoTServer::create_problem(const yarp::sig::Vector& state, iDynUtils& robot_model, const double dT,
                                   const std::string& name_space)
{
    /** Task initialization **/
    taskPostural = boost::shared_ptr<OpenSoT::tasks::velocity::Postural>(new OpenSoT::tasks::velocity::Postural(state));
    yPostural = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>(
                new OpenSoT::interfaces::yarp::tasks::YPostural(robot_model.getRobotName(), name_space, robot_model,
                                                           taskPostural));

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskPostural);
    _task0 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));


    /** Bounds initialization **/
    boundsJointLimits = OpenSoT::constraints::velocity::JointLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::JointLimits(
                                state,
                                robot_model.iDyn3_model.getJointBoundMax(),
                                robot_model.iDyn3_model.getJointBoundMin()));
    boundsJointVelocity = OpenSoT::constraints::velocity::VelocityLimits::ConstraintPtr(
                            new OpenSoT::constraints::velocity::VelocityLimits(0.3, mSecToSec(dT), state.size()));

    /** Create sot **/
    _stack_of_tasks.push_back(_task0);

    /** Create Aggregated Bounds **/
    _bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(
                    new OpenSoT::constraints::Aggregated(boundsJointLimits, boundsJointVelocity, state.size()));

    /** Solver Init **/
    _qpOasesSolver = OpenSoT::solvers::QPOases_sot::SolverPtr(
        new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _bounds, 1E0));
}

void openSoTServer::reset_tasks_and_constraints()
{
    /** Task reset **/
    taskPostural.reset();
    _task0.reset();
    _stack_of_tasks.clear();

    /** Bounds reset **/
    boundsJointLimits.reset();
    boundsJointVelocity.reset();
    _bounds.reset();

    /** Interface reset **/
    yPostural.reset();
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
    _bounds->update(state);
}

bool openSoTServer::solve(Vector &solution)
{
    return _qpOasesSolver->solve(solution);
}
