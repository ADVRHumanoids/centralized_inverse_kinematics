#include <yarp/os/all.h>
#include <GYM/generic_module.hpp>
#include <cstdlib>
#include <ros/ros.h>

#include "centralized_inverse_kinematics_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();
    ros::init(argc, argv, "centralized_inverse_kinematics");

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set centralized_inverse_kinematics_initial_config.ini as default
    // to specify another config file, run with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "centralized_inverse_kinematics_initial_config.ini" );
    //rf.setDefaultConfigFile( "centralized_inverse_kinematics_coman_config.ini" );

    rf.setDefaultContext( "centralized_inverse_kinematics" );
    rf.configure(argc, argv);
    // create my module
    centralized_inverse_kinematics_module centralized_inverse_kinematics_mod = centralized_inverse_kinematics_module( argc, 
														      argv, 
														      "centralized_inverse_kinematics",
														      MODULE_PERIOD, 
														      rf );
        
    // yarp network deinitialization
    yarp.fini();
    
    // run the module
    centralized_inverse_kinematics_mod.runModule( rf );
    
    exit(EXIT_SUCCESS);
}
