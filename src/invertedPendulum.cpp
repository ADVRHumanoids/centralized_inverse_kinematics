#include "invertedPendulum.h"
/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

InvertedPendulum::InvertedPendulum(){}
InvertedPendulum::~InvertedPendulum() {}
/**
 * @brief InvertedPendulum::timeTo
 * This function provides the time from the actual pos to reach the final pos geven the
   initial and final speeds
 * @param actualPos    COM position
 * @param actualSpeed  COM speed
 * @param finalPos     desired final position of the COM
 * @param finalSpeed   stimated speed at the finalPos
 * @param z_c          COM high
 * @return Time to reach the desired final states
 */
double InvertedPendulum::timeTo(double actualPos,double actualSpeed,double finalPos,double finalSpeed, double z_c){


    //double zc=0.41; //z_c;
    double Tc=sqrt(this->zcmodel/9.81);

    double temp2=actualPos+Tc*actualSpeed;
    if (temp2==0)
        return 0;
    double temp=((finalPos+finalSpeed*Tc)/temp2);

    if (temp<1)
        return 0;

    return Tc*log(temp);
}
/**
 * @brief InvertedPendulum::finalVel
 * Given a desired final position and the actual states return the final speed at the
    desired final pos
 * @param actualPos      COM actual position
 * @param actualSpeed    COM actual speed
 * @param desireFinalPos Desired final position
 * @param zc             COM high
 * @return COM Stimated Final speed at the final position
 */
double InvertedPendulum::finalVel(double actualPos,double actualSpeed,double desireFinalPos,double zc){

    double g=9.81;
    double temp=pow(actualSpeed,2)+g/zc*(pow(desireFinalPos,2)-pow(actualPos,2));
    if (temp>=0){
        return sqrt(temp);
    }
    else{
        return 0;
    }
}
double InvertedPendulum::finalVel2(double energy,double desireFinalPos,double zc){

    double g=9.81;
    double temp=energy+g/zc*pow(desireFinalPos,2);
    if (temp>=0){
        return sqrt(temp);
    }
    else{
        return 0;
    }
}
/**
 * @brief InvertedPendulum::newPos
 * given the speed at the final/initial position of the step and the speed in the stand position
    returns the landing position to recover the mean speedthe y axis

    This function is use for the synchronization of

 * @param meanSpeed       Speed in the mid upright position
 * @param newFinalSpeed   final speed at present step undesrtood as initial speed
 *                        for the next step
 * @param zc              COM high
 * @return  COM Position to revover the cyclic conditions of the gait
 *
 */
double InvertedPendulum::newPos(double meanSpeed,double newFinalSpeed,double zc){


    double g=9.81;
    double temp=(pow(newFinalSpeed,2)-pow(meanSpeed,2))*zc/g;
    if (temp>=0){
        return sqrt(temp);
    }
    else{
        return 0;
    }
}
/**
 * @brief InvertedPendulum::newVelY
 * Given the actual states and the expected landing time return the
    COM final velocity at the landing position

    This function is used to synchronizes y axis

 * @param actualPosition  COM actual position
 * @param actualSpeed     COM actual speed
 * @param RemainTime      Time to reach the final position
 * @param zc              COM high
 * @return                COM Final speed at the reached position
 */

double InvertedPendulum::newVelY(double actualPosition, double actualSpeed,double RemainTime,double zc){

    double Tc=sqrt(zc/9.81);
    double finalSpeed= actualPosition/Tc*sinh(RemainTime/Tc)+actualSpeed*cosh(RemainTime/Tc);
    if (finalSpeed>2)
        return 2;
    if (finalSpeed<-2)
        return -2;
    return finalSpeed;
}

/**
 * @brief InvertedPendulum::newPosY
 *  Given the actual states and the expected landing time return the
    COM foot placement to have 0 stand velocity

 * @param actualPosition COM actual position
 * @param actualSpeed    COM actual speed
 * @param RemainTime     Time to reach the final position
 * @param zc             COM high
 * @return               COM Final speed at the reached position
 */
double InvertedPendulum::newPosY(double actualPosition, double actualSpeed,double RemainTime,double zc){
    /*
    */
    double Tc=sqrt(zc/9.81);
    double temp=actualPosition*cosh(RemainTime/Tc)+Tc*actualSpeed*sinh(RemainTime/Tc);
    if (temp>1.3)
        return 1.3;
    if (temp<-1.3)
        return -1.3;
    return temp;
}

/**
 * @brief InvertedPendulum::landingPos
 *
 * Given the actual COM States return the final position of the COM knowing the expected
    landing speed
 *
 * @param actualPosition     COM actual position
 * @param actualSpeed        COM actual speed
 * @param finalSpeed         Time to reach the final position
 * @param zc                 COM high
 * @return     Foot position for the gait recovery
 */

double InvertedPendulum::landingPos(double actualPosition, double actualSpeed,double finalSpeed,double zc){

    double g=9.81;
    double a=pow(actualPosition,2)*g/zc+(pow(finalSpeed,2)-pow(actualSpeed,2));
    if (a<=0)
        return 0;
    a= sqrt(a*zc/g);
    if (a>0.9)
        return 0.9;
    return a;
}

/**
 * @brief InvertedPendulum::arrivalTime
 * given the actual states and final condition returns the landing time

 *
 * @param time          remaining time to reach the final position
 * @param finalPos      desired COM final position
 * @param finalSpeed    stimated COM final speed
 * @param actualPos     actual COM position
 * @param actualSpeed   actual COM position
 * @param zc            COM high
 * @return  Remainig time to reach the final state
 */

double InvertedPendulum::arrivalTime(double time,double finalPos,double finalSpeed,double actualPos,double actualSpeed,double zc){
    /*
    */
    double g=9.81;
    double temp=0;
    double Tc=sqrt(zc/g);
    double a=(actualPos+Tc*actualSpeed);
    if (a!=0){
        temp=double((finalPos+finalSpeed*Tc)/a);
    }
    else{
        temp=double((actualPos-Tc*actualSpeed)/(finalPos-Tc*finalSpeed));
    }

    if (temp>=double(1.0)){
     time=Tc*log(temp);
    }
    else if (temp<0){
        time=time;
    }
    else{
        time=0;
    }
    if (time>=TsCart){
        return time;
    }
    else{
        return 0;
    }
}

/**
 * @brief InvertedPendulum::relativePos
 * Given the the expected final speed and the desired 0 speed position
    returns the relative landing position to allows this motion


 * @param finalSpeed  stimated COM final speed
 * @param stopPos     location of the COM landing position
 * @param zc          COM hight
 * @return            distance to the COM in the next step so that the energy is recovered
 */

double InvertedPendulum::relativePos(double finalSpeed,double stopPos,double zc){
    /*
    */
    double temp=(pow(stopPos,2)+pow(finalSpeed,2)*zc/9.81);
    if (temp<=0)
        return 0;
    return sqrt(temp);
}

/**
 * @brief InvertedPendulum::finalPosY
 * This function provides the landing leg position to sincronize the next step
   given the time tao that is the expected time for the next step and the landing
   speed in the present step
 *
 * @param finalvel  stimated final velocity
 * @param tao       Time to perform the next step
 * @return  location of the foot in Y to synchronize the y axis
 */
double InvertedPendulum::finalPosY(double finalvel,double tao){
    //This function provides the landing leg position to sincronize the next step
    //given the time tao that is the expected time for the next step and the landing
    //speed in the present step
    double Tc=sqrt(this->zcmodel/9.81);

    double temp1=(exp(tao/Tc)-1);
    if (temp1==0)
        return 0;
    double temp=-Tc*finalvel*(exp(tao/Tc)+1)/temp1;
    // if (temp<0)
    //     return 0;
    if (temp>0.7)
        return 0.7;
    else if (temp<-0.7)
        return -0.7;
    return temp;
}
double InvertedPendulum::finalPosY2(double finalvel,double tao){
    //This function provides the landing leg position to sincronize the next step
    //given the time tao that is the expected time for the next step and the landing
    //speed in the present step
    double Tc=sqrt(this->zcmodel/9.81);

    double temp1=sinh(tao/Tc);
    if (temp1==0)
        return 0;
    double temp=-Tc*finalvel*cosh(tao/Tc)/temp1;//*temp1/temp1;
    // if (temp<0)
    //     return 0;
    if (temp>0.2)
        return 0.2;
    else if (temp<-0.2)
        return -0.2;
    return temp;
}

