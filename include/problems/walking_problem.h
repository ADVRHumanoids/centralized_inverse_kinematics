#ifndef __WALKING_PROBLEM__
#define __WALKING_PROBLEM__

#include <general_ik_problem.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <walking/locomotor.h>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class walking_problem: public general_ik_problem {

    enum STANCE_FOOT{
        LEFT_FOOT = 1,
        RIGHT_FOOT = -1
    };

public:
    walking_problem();

    boost::shared_ptr<ik_problem> homing_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

    virtual boost::shared_ptr<ik_problem> create_problem(const yarp::sig::Vector& state,
                        iDynUtils& robot_model,const double dT,
                        const std::string& name_space);

    OpenSoT::tasks::velocity::Cartesian::Ptr taskRFoot;
    OpenSoT::tasks::velocity::Cartesian::Ptr taskLFoot;
    OpenSoT::tasks::velocity::Cartesian::Ptr taskPelvis;
    OpenSoT::tasks::velocity::Postural::Ptr taskPostural;

    yarp::sig::Matrix LFootRef;
    yarp::sig::Matrix RFootRef;
    yarp::sig::Matrix pelvisRef;
    yarp::sig::Vector comRef;

    ~walking_problem();

    ofstream file_r_foot;
    ofstream file_l_foot;
    ofstream file_pelvis;

    bool switchSupportFoot(iDynUtils& robot_model, const int trj_stance_foot);

    boost::shared_ptr<Clocomotor> pattern_generator;
    MatrixXd footPositions;

    bool walking_pattern_finished;

    bool homing_done;

    bool reset_solver;

    bool start_walking_pattern;

private: void generateFootSteps(const int number_of_steps, const double step_width, const double step_lenght);
    int stance_foot;
    ros::NodeHandle n;
    ros::Publisher new_world_pub;
public: bool walkingPatternGeneration(const double step_time, const int number_of_steps,
                                      const double step_width, const double step_lenght);
    bool updateWalkingPattern(yarp::sig::Matrix& LFootRef, yarp::sig::Matrix& RFootRef,
                              yarp::sig::Matrix& PelvisRef, yarp::sig::Vector& CoMRef,
                              int& stance_foot);
    void log(const yarp::sig::Matrix& LFootRef, const yarp::sig::Matrix& RFootRef,
             const yarp::sig::Matrix& PelvisRef);

    virtual bool update(const yarp::sig::Vector& state)
    {
        ros::spinOnce();

        for(unsigned int i = 0; i < problem->stack_of_tasks.size(); ++i)
            problem->stack_of_tasks[i]->update(state);
        problem->bounds->update(state);

        if(!homing_done)
        {
            if(yarp::math::norm(taskRFoot->getb()) <= 1e-3 &&
               yarp::math::norm(taskLFoot->getb()) <= 1e-3)
            {
                ROS_WARN("HOMING DONE!");
                homing_done = true;
            }
        }
    }
};

#endif //__WALKING_PROBLEM__
