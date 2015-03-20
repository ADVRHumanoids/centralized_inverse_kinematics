#ifndef __THE_DYING_SWAN_PROBLEM__
#define __THE_DYING_SWAN_PROBLEM__

#include <general_ik_problem.h>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <fstream>

class the_dying_swan_problem: public general_ik_problem {
public:
    the_dying_swan_problem();

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

    OpenSoT::tasks::velocity::Cartesian::Ptr taskRWrist;
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr taskRWrist_interface;

    OpenSoT::tasks::velocity::Cartesian::Ptr taskRSole;


    ~the_dying_swan_problem();
};

#endif //__THE_DYING_SWAN_PROBLEM__
