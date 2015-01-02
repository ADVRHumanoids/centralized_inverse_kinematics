#include <openSoTServer.h>
#include <boost/shared_ptr.hpp>

#define mSecToSec(X) (X*0.001)

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace yarp::sig;

simple_problem::simple_problem():
    general_ik_problem()
{}

boost::shared_ptr<simple_problem::ik_problem> simple_problem::create_problem(const Vector& state,
                                                                             iDynUtils& robot_model, const double dT,
                                                                             const std::string& name_space)
{
    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(boost::shared_ptr<Cartesian>(
                    new Cartesian("cartesian::r_sole",state,robot_model,robot_model.right_leg.end_effector_name,"world")));
    YRSoleCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                    new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                    boost::static_pointer_cast<Cartesian>(taskList.front())));
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
    problem->stack_of_tasks[0]->getConstraints().push_back(boost::shared_ptr<ConvexHull>(new ConvexHull(state, robot_model, 0.05)));
    problem->stack_of_tasks[0]->getConstraints().push_back(boost::shared_ptr<CoMVelocity>(new CoMVelocity(Vector(3,0.03),
                                                                                                mSecToSec(dT), state,
                                                                                                robot_model)));

    taskList.clear();
    taskList.push_back(boost::shared_ptr<Cartesian>(new Cartesian("cartesian::Waist",state,robot_model,"Waist","world")));
    YWaistCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
                    new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
                                                    boost::static_pointer_cast<Cartesian>(taskList.front())));

    boost::shared_ptr<Cartesian> taskTorso(boost::shared_ptr<Cartesian>(new Cartesian("cartesian::torso",state,robot_model,"torso","world")));
    std::vector<bool> active_joint_mask = taskTorso->getActiveJointsMask();
    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
    taskTorso->setActiveJointsMask(active_joint_mask);
    yarp::sig::Matrix W_torso(6,6); W_torso = W_torso.eye();
    W_torso(0,0) = 0.0; W_torso(1,1) = 0.0; W_torso(2,2) = 0.0;
    taskTorso->setWeight(W_torso);
    YTorsoCartesian = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
        new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space, taskTorso));


    taskList.push_back(taskTorso);
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
    problem->stack_of_tasks[1]->getConstraints().push_back(boost::shared_ptr<ConvexHull>(new ConvexHull(state, robot_model, 0.05)));
    problem->stack_of_tasks[1]->getConstraints().push_back(boost::shared_ptr<CoMVelocity>(new CoMVelocity(yarp::sig::Vector(3,0.03),
                                                                                                mSecToSec(dT), state,
                                                                                                robot_model)));


    taskList.clear();
    taskList.push_back(boost::shared_ptr<Postural>(new Postural(state)));
    YPostural = boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>(
                    new OpenSoT::interfaces::yarp::tasks::YPostural(robot_model.getRobotName(), name_space, robot_model,
                                                    boost::static_pointer_cast<Postural>(taskList.front())));
    problem->stack_of_tasks.push_back(OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size())));
    problem->stack_of_tasks[2]->getConstraints().push_back(boost::shared_ptr<ConvexHull>(new ConvexHull(state, robot_model, 0.05)));
    problem->stack_of_tasks[2]->getConstraints().push_back(boost::shared_ptr<CoMVelocity>(new CoMVelocity(yarp::sig::Vector(3,0.03),
                                                                                                mSecToSec(dT), state,
                                                                                                robot_model)));

    problem->bounds = boost::shared_ptr<OpenSoT::constraints::Aggregated>(
                    new OpenSoT::constraints::Aggregated(JointLimits::ConstraintPtr(new JointLimits(state,
                                                                 robot_model.iDyn3_model.getJointBoundMax(),
                                                                 robot_model.iDyn3_model.getJointBoundMin())),
                                                         VelocityLimits::ConstraintPtr(new VelocityLimits(
                                                                 0.1, mSecToSec(dT), state.size())), state.size()));

    problem->damped_least_square_eps = 2E2;

    return problem;
}

//boost::boost::shared_ptr<simple_problem::ik_problem> simple_problem::create_problem(const yarp::sig::Vector& state,
//                                                                             iDynUtils& robot_model, const double dT,
//                                                                             const std::string& name_space)
//{
//    ///TASK RSOLE
//    taskCartesianRSole = boost::boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
//                            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_sole",state,robot_model,
//                                                                    robot_model.right_leg.end_effector_name,
//                                                                    "world"));
//    YRSoleCartesian = boost::boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
//                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
//                                                                 taskCartesianRSole));

//    ///TASK WAIST
//    taskCartesianWaist = boost::boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
//                            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",state,robot_model,
//                                                                    "Waist",
//                                                                    "world"));
//    YWaistCartesian = boost::boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
//                new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
//                                                                 taskCartesianWaist));

//    ///TASK TORSO
//    taskCartesianTorso = boost::boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian>(
//    new OpenSoT::tasks::velocity::Cartesian("cartesian::torso",state,robot_model,
//                                            "torso",
//                                            "world"));
//    YTorsoCartesian = boost::boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YCartesian>(
//    new OpenSoT::interfaces::yarp::tasks::YCartesian(robot_model.getRobotName(), name_space,
//                                         taskCartesianTorso));
//    std::vector<bool> active_joint_mask = taskCartesianTorso->getActiveJointsMask();
//    for(unsigned int i = 0; i < robot_model.left_leg.getNrOfDOFs(); ++i)
//        active_joint_mask[robot_model.left_leg.joint_numbers[i]] = false;
//    taskCartesianTorso->setActiveJointsMask(active_joint_mask);
//    yarp::sig::Matrix W_torso(6,6); W_torso = W_torso.eye();
//    W_torso(0,0) = 0.0; W_torso(1,1) = 0.0; W_torso(2,2) = 0.0;
//    YTorsoCartesian->taskCartesian->setWeight(W_torso);

//    ///TASK POSTURAL
//    taskPostural = boost::boost::shared_ptr<OpenSoT::tasks::velocity::Postural>(new OpenSoT::tasks::velocity::Postural(state));
//    YPostural = boost::boost::shared_ptr<OpenSoT::interfaces::yarp::tasks::YPostural>(
//                new OpenSoT::interfaces::yarp::tasks::YPostural(robot_model.getRobotName(), name_space, robot_model,
//                                                           taskPostural));
//    ///CONSTRAINT CH
//    constraintConvexHull = boost::boost::shared_ptr<OpenSoT::constraints::velocity::ConvexHull>(
//                new OpenSoT::constraints::velocity::ConvexHull(state, robot_model, 0.05));
//    ///CONSTRAINT CoM VEL
//    constraintCoMVelocity = boost::boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity>(
//                new OpenSoT::constraints::velocity::CoMVelocity(yarp::sig::Vector(3,0.03), mSecToSec(dT), state, robot_model));

//    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
//    taskList.push_back(taskPostural);
//    _task0 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task0->getConstraints().push_back(constraintConvexHull);
//    _task0->getConstraints().push_back(constraintCoMVelocity);

//    taskList.clear();
//    taskList.push_back(taskCartesianWaist);
//    taskList.push_back(taskCartesianTorso);
//    _task1 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task1->getConstraints().push_back(constraintConvexHull);
//    _task1->getConstraints().push_back(constraintCoMVelocity);

//    taskList.clear();
//    taskList.push_back(taskCartesianRSole);
//    _task2 = OpenSoT::tasks::Aggregated::TaskPtr(new OpenSoT::tasks::Aggregated(taskList, state.size()));
//    _task2->getConstraints().push_back(constraintConvexHull);
//    _task2->getConstraints().push_back(constraintCoMVelocity);

//    /** Bounds initialization **/
//    boundsJointLimits = OpenSoT::constraints::velocity::JointLimits::ConstraintPtr(
//                            new OpenSoT::constraints::velocity::JointLimits(
//                                state,
//                                robot_model.iDyn3_model.getJointBoundMax(),
//                                robot_model.iDyn3_model.getJointBoundMin()));
//    boundsJointVelocity = OpenSoT::constraints::velocity::VelocityLimits::ConstraintPtr(
//                            new OpenSoT::constraints::velocity::VelocityLimits(0.1, mSecToSec(dT), state.size()));

//    /** Create sot **/
//    _problem->stack_of_tasks.push_back(_task2);
//    _problem->stack_of_tasks.push_back(_task1);
//    _problem->stack_of_tasks.push_back(_task0);

//    /** Create Aggregated Bounds **/
//    _problem->bounds = boost::boost::shared_ptr<OpenSoT::constraints::Aggregated>(
//                    new OpenSoT::constraints::Aggregated(boundsJointLimits, boundsJointVelocity, state.size()));

//    _problem->damped_least_square_eps = 2E2;

//    return _problem;
//}

//void openSoTServer::reset_problem()
//{
//    /** Task & Constraint reset **/
//    taskPostural.reset();
//    taskCartesianRSole.reset();
//    taskCartesianTorso.reset();
//    taskCartesianWaist.reset();
//    constraintConvexHull.reset();
//    constraintCoMVelocity.reset();
//    _task0.reset();
//    _task1.reset();
//    _task2.reset();

//    /** Bounds reset **/
//    boundsJointLimits.reset();
//    boundsJointVelocity.reset();

//    /** Interface reset **/
//    YPostural.reset();
//    YRSoleCartesian.reset();
//    YTorsoCartesian.reset();
//    YWaistCartesian.reset();
//}

//void openSoTServer::update(const Vector &state)
//{
//    _task0->update(state);
//    _task1->update(state);
//    _task2->update(state);
//    _problem->bounds->update(state);
//}
