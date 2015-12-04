#ifndef INVERTEDPENDULUM_H
#define INVERTEDPENDULUM_H
/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/
#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;

     class InvertedPendulum
    {   private:
         double zcmodel=0.48;
        public:
            double TsCart;
            InvertedPendulum();
            ~InvertedPendulum(void);
            double finalVel(double actualPos,double actualSpeed,double desireFinalPos,double zc);
            double newPos(double meanSpeed,double newFinalSpeed,double zc);
            double finalVel2(double energy,double desireFinalPos,double zc);
            double landingPos(double actualPosition, double actualSpeed,double finalSpeed,double zc);
            double newVelY(double actualPosition, double actualSpeed,double RemainTime,double zc);
            double newPosY(double actualPosition, double actualSpeed,double RemainTime,double zc);
            double landPos(double finalSpeed);
            double finalPosY(double finalvel,double tao);
            double finalPosY2(double finalvel,double tao);
            double timeTo(double actualPos,double actualSpeed,double finalPos,double finalSpeed,double z_c);
            double arrivalTime(double time,double finalPos,double finalSpeed,double actualPos,double actualSpeed,double zc);
            double relativePos(double finalSpeed,double stopPos,double zc);

     };


#endif // INVERTEDPENDULUM_H
