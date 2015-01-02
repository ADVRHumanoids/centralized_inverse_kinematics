#ifndef __OPENSOTSERVER__
#define __OPENSOTSERVER__
#include <yarp/sig/Vector.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/interfaces/yarp/tasks/YPostural.h>
#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>


class openSoTServer {
public:
    struct ik_problem
    {
        std::vector<boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>>> stack_of_tasks;
        boost::shared_ptr<OpenSoT::constraints::Aggregated > bounds;
        double damped_least_square_eps;
    };

private:    
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
     * @brief task1 first stack
     */
    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task1;

    /**
     * @brief task1 first stack
     */
    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task2;
    
    /**
     * @brief reset_tasks_and_constraints reset the current stack of tasks
     */
    void reset_tasks_and_constraints();

    boost::shared_ptr<ik_problem> _problem;
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
    boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);
    
    /**
     * @brief reset_solver reset the current solver
     * 
     */
    void reset_solver();
    
    /**
     * @brief reset_problem reset the current problem: it resets the tasks and the solver
     */
    void reset_problem();
    
    /**
     * @brief destructor
     * 
     */
    ~openSoTServer(){}

    /**
     * @brief update updates tasks and constraints
     * @param state actual state of the robot
     */
    void update(const yarp::sig::Vector& state);

    /**
     * @brief solve solve the problem
     * @param solution computed solution
     * @return true if problem is solvable
     */
    bool solve(yarp::sig::Vector& solution);

    
    /****************************************************
     *  This set of tasks will be replaced by a factory *
     * **************************************************/
    
    /**
     * @brief postural task ( in joint space )
     * 
     */
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> taskPostural;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural> YPostural;

    /**
     * @brief taskCartesianRSole Task on leg
     */
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianRSole;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YRSoleCartesian;

    /**
     * @brief taskCartesianWaist Task on Waist
     */
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianWaist;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YWaistCartesian;

    /**
     * @brief taskCartesianRSole Task on Torso
     */
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianTorso;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YTorsoCartesian;

    boost::shared_ptr<OpenSoT::constraints::velocity::ConvexHull> constraintConvexHull;
    boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity> constraintCoMVelocity;

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
