/*
    Created date: 28 Oct 2014
    Copyright (C) Department of Advanced Robotics, Italian Institute of Technology
    Developer: Zhibin Li
    Email: zhibin.li@iit.it
    Description: this file provides the simple matrix operation based on Eigen. You can use KDL to replace
*/

#ifndef _MatrixVector_H// header guards
#define _MatrixVector_H

#include <Eigen/Dense>
using namespace Eigen;

extern Matrix3d Rx(double);
extern Matrix3d Ry(double);
extern Matrix3d Rz(double);
extern Matrix3d Rodrigues(Vector3d w,double theta);

inline double round( double d ){ return floor( d + 0.5 );}

#endif

