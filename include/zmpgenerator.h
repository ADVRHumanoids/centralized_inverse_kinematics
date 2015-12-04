/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/
#ifndef ZMPGENERATOR_H
#define ZMPGENERATOR_H
#include <vector>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <iostream>
/**
 * @brief The zmpgenerator class
 * Generation of X and Y ZMP trajectories for
 * a variable preview window and described in a  generalized way
 * Assume that for the future step the sequence is a set of similar steps
 * i.e same timing, same lenght
 */

class zmpgenerator
{
private:

    std::vector<double> zmp;
    double Poly3(double s0,double t0,double sf,double tf,double time);
    std::vector<double> FilterZmp(std::string whichMatrix,std::vector<double> zmp);
    std::vector<double> addOffset(std::vector<double> vector,double offset);
public:

    int preview_window;
    int matrix_size;
    zmpgenerator();
    ~zmpgenerator();
    zmpgenerator(int window, int mSize);
    std::vector<std::vector<double> > zmpMx;
    std::vector<std::vector<double> > zmpMy;
    std::vector<double> ZmpUpdatex(double footPosition,double SSRemainTime,int SSPhaseSize,
       int DSPhaseSize,double T,double stepLength,double xfinal,double position);
    std::vector<double> ZmpUpdatey(double footPosition,double SSRemainTime,int SSPhaseSize,double DSRemainTime,int DSPhaseSize,
        double T,double stepLength,double ydes,double position);



};

#endif // ZMPGENERATOR_H
