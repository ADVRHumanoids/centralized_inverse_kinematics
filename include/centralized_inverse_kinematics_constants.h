#ifndef centralized_inverse_kinematics_CONSTANTS_H_
#define centralized_inverse_kinematics_CONSTANTS_H_

/**
  This variable is used to specify the type of IK Problem that is going to be used
  in the module. This variable CAN NOT be changed in runtime.
**/
#define IK_PROBLEM_TYPE_CONST walking_problem
#ifndef IK_PROBLEM_TYPE_CONST
    IK_PROBLEM_TYPE_CONST general_ik_problem
#endif

/**
  These variables are used inside the ParamHelper. These variables CAN BE changed
  in runtime.
**/
#define IS_CLIK_ID 1
#define IS_CLIK_SIZE 1

#endif
