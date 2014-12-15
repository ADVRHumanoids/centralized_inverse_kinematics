#ifndef __OPENSOTSERVER__
#define __OPENSOTSERVER__
#include <yarp/sig/Vector.h>

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
     * @brief reset_tasks reset the current stack of tasks
     * 
     * @return true on success
     */
    bool reset_tasks();
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
     * @return true on success
     */
    bool create_problem( const yarp::sig::Vector& state );
    
    /**
     * @brief reset_solver reset the current solver
     * 
     * @return true on success
     */
    bool reset_solver();
    
    /**
     * @brief reset_problem reset the current problem: it resets the tasks and the solver
     * 
     * @param state ...
     * @return true if both reset tasks and reset solver success
     */
    bool reset_problem( const yarp::sig::Vector& state );
    
    /**
     * @brief destructor
     * 
     */
    ~openSoTServer();
    
    
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
};

#endif //__OPENSOTSERVER__
