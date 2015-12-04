#ifndef PreviewClass_H
#define PreviewClass_H
/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
using namespace std;
class PreviewClass
    {
    private:
        int N2;
        std::vector<double> Gd;
        double Gd1[3];
        double GI;

        double saturate(double PseudoZMP,double desiredZMP,double actualPosition,double z_c,double delta);
    public:
        PreviewClass(void);
        ~PreviewClass();
            void loadGains(std::string FILE);
            double previewControl(double *returnState,double states[3],double sysdA[3][3],double sysdB[3],double sysdC[3],std::vector<double> zmp,double limit,double TsCart,double z_c);
            };

#endif
