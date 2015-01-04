#ifndef __GENERAL_IK_PROBLEM__
#define __GENERAL_IK_PROBLEM__

#include <yarp/sig/Vector.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/interfaces/yarp/tasks/YPostural.h>
#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>

class general_ik_problem
{
public:
    struct ik_problem
    {
        std::vector<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr> stack_of_tasks;
        OpenSoT::constraints::Aggregated::Ptr bounds;
        double damped_least_square_eps;
    };

    boost::shared_ptr<ik_problem> problem;

    general_ik_problem()
    {
        problem = boost::shared_ptr<ik_problem>(new ik_problem);
    }

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                                                         iDynUtils& robot_model,
                                                         const double dT,
                                                         const std::string& name_space) = 0;

    virtual bool update(const yarp::sig::Vector& state)
    {
        for(unsigned int i = 0; i < problem->stack_of_tasks.size(); ++i)
            problem->stack_of_tasks[i]->update(state);
        problem->bounds->update(state);
    }
};
#endif
