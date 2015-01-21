#ifndef __INTERACTION_PROBLEM__
#define __INTERACTION_PROBLEM__

#include <general_ik_problem.h>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <fstream>

class interaction_problem: public general_ik_problem {
public:
    interaction_problem();

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);
    OpenSoT::tasks::velocity::Interaction::Ptr taskRWrist;

    ofstream file;

    void recordWrench();

    ~interaction_problem();


};

#endif //__SIMPLE_PROBLEM__
