#ifndef __WB_MANIP_PROBLEM__
#define __WB_MANIP_PROBLEM__

#include <general_ik_problem.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <fstream>

class wb_manip_problem: public general_ik_problem {
public:
    wb_manip_problem(iDynUtils& robot_model, std::string urdf_path, std::string srdf_path);

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

    OpenSoT::tasks::velocity::Cartesian::Ptr taskRWrist;
    OpenSoT::tasks::velocity::Cartesian::Ptr taskLWrist;
    OpenSoT::tasks::velocity::Cartesian::Ptr taskRSole;

    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YRSoleCartesian;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YRWristCartesian;
    boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian> YLWristCartesian;

    ofstream file;


    ~wb_manip_problem();


};

#endif //__WB_MANIP_PROBLEM__
