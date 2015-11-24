/**
    @file IntegralControl.h
    @author Juan Alejandro Castano, Zhibin Li, juan[dot]castano[at]iit[dot]it, zhibin[dot]li[at]iit[dot]it

    @section License
    Copyright (C) 2014 Department of Advanced Robotics, Italian Institute of Technology, all rights reserved.

    @section Description
    Integral Control header file, mainly used for upper body attitude control, can also be used for other control tasks.
*/

#ifndef ORIENTATIONCONTROL_H
#define ORIENTATIONCONTROL_H
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "FilterHClass.h"
using namespace Eigen;
using namespace std;

class IntegralControl
{
public:

    IntegralControl();
    ~IntegralControl();
    IntegralControl(double LQRgains[2],double sampleTime,std::string FILEG,std::string FILEH,std::string FILEF);
    std::vector<double> filterdata2(double alfa,double alfad);
    void initfilters();

    /**
     * @brief integralCtrl::apply
     * Apply the control law with either LQR or MPC controller
     * accoriding to the controlFlag variable
     * controlFlag==0 for LQR;   controlFlag==1 for MPC
     * @param ref array of size control.N2
     * @return the desired angle
     */
    double apply(double *ref);
    Vector3d DynamicCompensator(Matrix3d R,Vector3d COMvector , double mb, double mleg);
    /**
     * @brief States Position and velocity
     */
    int constraints;
    Vector2d States;
    /**
     * controlFlag==0 for LQR;   controlFlag==1 for MPC
     */
    int controlFlag;
    /*
     * MPC control parameters*/
    double alfa;
    int Nu;
    int N2;
    double TsCart;
    double offset;
    VectorXd spline;
    double counts;
    std::vector<double> X    ;
    int sizeA;

    double filterstate(double x);
private:
    //SS discrete system
    Matrix2d A;
    Vector2d B;
    Vector2d C;
    double freq;
    FilterH Filteralfa;
    FilterH Filteralfad;
    FilterH outFilter;
    FilterH stateFilter;
    double controleffort;
    double LQRgains[2];
    double sampletime;

    int sizeB;
    int sizeC;
    int sizeD;
    //double alfa;

    //int Nu;
    //int N2;


    //tranfer function for MPC
    VectorXd Ampc;
    VectorXd Bmpc;
    //Filter tranfer function for MPc
    VectorXd Cmpc;
    VectorXd Dmpc;
    //MPC internal variables

    std::vector<double> U    ;
    std::vector<double> NF;
    std::vector<double> N;

    MatrixXd F;
    MatrixXd invH;
    VectorXd ConstraintB;
    MatrixXd ConstraintA;
    MatrixXd invG;

    double filterout();

    double LQRcontroller(Vector2d States,double reference);
    void importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF);
    void MPC(double Yt,double *Wt);
    double applyControl(Vector2d States,double controlEffort);
    VectorXd saturateMPC(double *Err, double *Ybase,std::vector<double> U);


};

#endif // ORIENTATIONCONTROL_H
