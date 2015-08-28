#ifndef __SIMPLE_PROBLEM__
#define __SIMPLE_PROBLEM__

#include <general_ik_problem.h>


class simple_problem: public general_ik_problem {
public:
    simple_problem(iDynUtils& robot_model, std::string urdf_path, std::string srdf_path);
    
    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>  YPostural;

    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YRSoleCartesian;

    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YWaistCartesian;

    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YTorsoCartesian;

};

#endif //__SIMPLE_PROBLEM__
