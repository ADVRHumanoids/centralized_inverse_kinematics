#include <openSoTServer.h>
#include <boost/shared_ptr.hpp>

openSoTServer::openSoTServer()
{

}

void openSoTServer::create_problem(const yarp::sig::Vector& state, iDynUtils& robot_model, const double dT)
{
    /** Task initialization **/
    taskPostural = boost::shared_ptr<OpenSoT::tasks::velocity::Postural>(new OpenSoT::tasks::velocity::Postural(state));

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
                            new OpenSoT::constraints::velocity::VelocityLimits(0.3, dT, state.size()));

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
}

void openSoTServer::reset_problem(const yarp::sig::Vector& state)
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
