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
    /**
     * @brief The ik_problem struct contains all the data needed to describe an IK Problem
     */
    struct ik_problem
    {
        /**
         * @brief stack_of_tasks is the vector of tasks
         */
        std::vector<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr> stack_of_tasks;

        /**
         * @brief bounds are the bounds of the IK Problem
         */
        OpenSoT::constraints::Aggregated::Ptr bounds;

        /**
         * @brief global_constraints are constraints that are applied to all the tasks
         */
        OpenSoT::constraints::Aggregated::Ptr global_constraints;

        /**
         * @brief damped_least_square_eps parameter for the damping factor of the solver
         */
        double damped_least_square_eps;
    };

    boost::shared_ptr<ik_problem> problem;

    general_ik_problem(iDynUtils &robot_model, std::string& urdf_path, std::string& srdf_path):
        _robot_model(robot_model),
        _urdf_path(urdf_path),
        _srdf_path(srdf_path)
    {
        problem = boost::shared_ptr<ik_problem>(new ik_problem);
    }

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                                                         iDynUtils& robot_model,
                                                         const double dT,
                                                         const std::string& name_space) = 0;

    virtual boost::shared_ptr<ik_problem> homing_problem(const yarp::sig::Vector& state,
                                                         iDynUtils& robot_model,
                                                         const double dT,
                                                         const std::string& name_space) = 0;

    /**
     * @brief update_ik_problem updates all the internal object of the ik_problem
     * @param state actual joint values
     * @return true
     */
    virtual bool update_ik_problem(const yarp::sig::Vector& state)
    {
        for(unsigned int i = 0; i < problem->stack_of_tasks.size(); ++i)
            problem->stack_of_tasks[i]->update(state);
        if(problem->bounds)
            problem->bounds->update(state);
        if(problem->global_constraints)
            problem->global_constraints->update(state);

        return true;
    }

    /**
     * @brief run contains user code to handle object of the derived class
     * @param state actual joint values
     * @return true or false
     */
    virtual bool run(const yarp::sig::Vector& state) = 0;

    /**
     * @brief update both update_ik_problem() and run()
     * @param state actual joint values
     * @return true or false
     */
    virtual bool update(const yarp::sig::Vector& state)
    {
        bool a = update_ik_problem(state);
        bool b = run(state);

        return a && b;
    }
protected:
    /**
     * @brief _robot_model is an internal reference to an external robot_model that will not be used to
     * create tasks.
     */
    iDynUtils& _robot_model;

    /**
     * @brief urdf_path is the path to the used urdf
     */
    std::string _urdf_path;

    /**
     * @brief srdf_path is the path to the used srdf
     */
    std::string _srdf_path;
};
#endif
