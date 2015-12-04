#ifndef MPCClass_H
#define MPCClass_H
/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
using namespace std;
using namespace Eigen;

    class MPCClass
    {
        int sizeA;
        int sizeB;
        int sizeC;
        int sizeD;
        Matrix3d A;
        Vector3d B;
        Vector3d C;

    public:

        //tranfer function for MPC
        VectorXd Ampc;
        VectorXd Bmpc;
        //Filter tranfer function for MPc
        VectorXd Cmpc;
        VectorXd Dmpc;
        //MPC internal variables
         std::vector<double> X	;
         std::vector<double> U	;
         std::vector<double> NF;
         std::vector<double> N;
         MatrixXd F;
         MatrixXd invH;
         VectorXd ConstraintB;
         MatrixXd ConstraintA;
         MatrixXd invG;
         Vector3d States;
        int controlFlag;
        double alfa;
        int Nu;
        int N2;
        int constraint;
          MPCClass();
         ~MPCClass();
            void importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF);
            void MPC(double Yt,double *Wt);
            double saturate(double PseudoZMP,double desiredZMP,double actualPosition,double z_c,double delta);
            VectorXd saturateMPC(double *Err, double *Ybase,std::vector<double> U);
    };

#endif // MPCClass_H
