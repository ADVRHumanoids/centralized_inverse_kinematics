#include <openSoTServer.h>
#include <boost/shared_ptr.hpp>

openSoTServer::openSoTServer()
{

}

bool openSoTServer::create_problem(const yarp::sig::Vector& state)
{
    taskPostural = boost::shared_ptr<OpenSoT::tasks::velocity::Postural>(new OpenSoT::tasks::velocity::Postural(state));
    
}

bool openSoTServer::reset_tasks()
{
    taskPostural.reset();
}

bool openSoTServer::reset_problem(const yarp::sig::Vector& state)
{
    reset_tasks();
    reset_solver();
}


bool openSoTServer::reset_solver()
{
    _qpOasesSolver.reset();
    
}

