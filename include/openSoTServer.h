#ifndef __OPENSOTSERVER__
#define __OPENSOTSERVER__
#include <yarp/sig/Vector.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>

class openSoTServer {
private:
    /**
     * @brief stack of OpenSoT tasks
     * 
     */
    std::vector<boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>>> _stack_of_tasks;
    
    /**
     * @brief qpOases OpenSoT solver
     * 
     */
    boost::shared_ptr<OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>> _qpOasesSolver;

    /**
     * @brief task0 first stack
     */
    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task0;

    /**
     * @brief _bounds aggreated bounds
     */
    boost::shared_ptr<OpenSoT::constraints::Aggregated > _bounds;
    
    /**
     * @brief reset_tasks_and_constraints reset the current stack of tasks
     * 
     * @return true on success
     */
    void reset_tasks_and_constraints();
public:
    /**
     * @brief constructor
     * 
     */
    openSoTServer(); 
    
    /**
     * @brief create_problem create a default problem with these tasks: 
     * 	      - 
     * 	      -
     * 
     * @param state current state of the robot (eg. q, the joint position)
     * @param robot_model idynutils of a robot
     */
    void create_problem(const yarp::sig::Vector& state , iDynUtils &robot_model, const double dT);
    
    /**
     * @brief reset_solver reset the current solver
     * 
     */
    void reset_solver();
    
    /**
     * @brief reset_problem reset the current problem: it resets the tasks and the solver
     * 
     * @param state ...
     */
    void reset_problem( const yarp::sig::Vector& state );
    
    /**
     * @brief destructor
     * 
     */
    ~openSoTServer();


    void update(const yarp::sig::Vector& state);

    
    /****************************************************
     *  This set of tasks will be replaced by a factory *
     * **************************************************/
    
    /**
     * @brief postural task ( in joint space )
     * 
     */
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> taskPostural;
    
    /**
     * @brief joint limits bound
     * 
     */
    boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointLimits;

    /**
     * @brief joint velocity limits bound
     */
    boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointVelocity;
};

#endif //__OPENSOTSERVER__
