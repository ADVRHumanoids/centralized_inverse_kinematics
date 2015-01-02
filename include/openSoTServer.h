#ifndef __SIMPLE_PROBLEM__
#define __SIMPLE_PROBLEM__

#include <general_ik_problem.h>


class simple_problem: public general_ik_problem {
public:

private:    
//    /**
//     * @brief task0 first stack
//     */
//    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task0;

//    /**
//     * @brief task1 first stack
//     */
//    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task1;

//    /**
//     * @brief task1 first stack
//     */
//    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task2;
    
//    /**
//     * @brief reset_tasks_and_constraints reset the current stack of tasks
//     */
//    void reset_tasks_and_constraints();

//    boost::shared_ptr<ik_problem> _problem;
public:
    /**
     * @brief constructor
     * 
     */
    simple_problem();
    
    /**
     * @brief create_problem create a default problem with these tasks: 
     * 	      - 
     * 	      -
     * 
     * @param state current state of the robot (eg. q, the joint position)
     * @param robot_model idynutils of a robot
     */
    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

//    /**
//     * @brief update updates tasks and constraints
//     * @param state actual state of the robot
//     */
//    void update(const yarp::sig::Vector& state);
    
//    /****************************************************
//     *  This set of tasks will be replaced by a factory *
//     * **************************************************/
    
//    /**
//     * @brief postural task ( in joint space )
//     *
//     */
//    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> taskPostural;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural> YPostural;

//    /**
//     * @brief taskCartesianRSole Task on leg
//     */
//    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianRSole;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YRSoleCartesian;

//    /**
//     * @brief taskCartesianWaist Task on Waist
//     */
//    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianWaist;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YWaistCartesian;

//    /**
//     * @brief taskCartesianRSole Task on Torso
//     */
//    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianTorso;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YTorsoCartesian;

//    boost::shared_ptr<OpenSoT::constraints::velocity::ConvexHull> constraintConvexHull;
    boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity> constraintCoMVelocity;

//    /**
//     * @brief joint limits bound
//     *
//     */
//    boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointLimits;

//    /**
//     * @brief joint velocity limits bound
//     */
//    boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector> > boundsJointVelocity;
};

#endif //__SIMPLE_PROBLEM__
