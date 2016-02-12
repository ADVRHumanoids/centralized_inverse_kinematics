#ifndef WALKING
#define WALKING

#include <stdio.h> 
#include <math.h>
#include <vector>
#include <iostream>
#include "previewClass.h"
#include "mpcClass.h"
#include "invertedPendulum.h"
#include "zmpgenerator.h"
#include <fstream>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;



#include "FilterHClass.h"

namespace dynamicWalk {

const double Tprev=2.0;
const double TsCart=0.005;
const int MSizey=2;
const int MSizex=2;
const int N1=1;
const int Nu=10;
const int N2=400;
const int sizeA=4;
const int sizeB=4;
const int sizeC=1;
const int sizeD=4;
const  double halfHip=0.14;
static double Gd[N2];
static double Gd1[3];
static double GI;
static double invG[Nu][N2];
static double Hmatrix[Nu][Nu];
static double Fmatrix[Nu][N2];
const int s=N2;
static double freq=20;


class DynamicWalkClass {
	public:

    InvertedPendulum pendulumModel;
    PreviewClass controlP;
    MPCClass controlM;
    MPCClass controlMY;
    zmpgenerator zmpref2;
    double HALF_HIP_WIDTH;
    double Fgcomx[3],Fgcomy[3];
    double steptime;
    double relativepos[2];
    double relativeflag;

		ofstream in;
        struct ZMPStructure_S{
        //flags
            double clearance, whichfoot;
            double ConstraintB[2*Nu];
            double ConstraintA[2*Nu][Nu];
            double finalSpeed;
            double startwalk;
            double stop;
            double DSRemainTimey;
            int walkingcase;
            int stepCount;
            double SStime;
            double positiony;
            int goLeft,	goRight, footChange;
            //measures
            double SSRemainTime, DSRemainTime;
            double position, position2;
            double footEdgeY;
            double actualPos, actualSpeed;
            double actualPosY, actualSpeedy;
            //wlaking characteristics
            double stepLength, stepLength1, stepLengthy;
            double offset, desirexf;
            double z_c;
            double SSPhaseSize, DSPhaseSize;
            double footEdgex;
            double xfinal;
            double zmpyref;
            double zeroSpeed, zeroSpeedy, zeroSpeed1;
            double zmpLength, zmpLengthy;
            //matrix and vectors

            double zmpy[N2];
            double DSReference;

            double zmpx[N2];
            //dynamic variables
            double time, footPosition, footy, desiredXf;
            double LFX,RFX,LFZ,RFZ,LFY,RFY;
            double yTemp;
            double leftFoot,rightFoot;
        };
		ZMPStructure_S ZMPstructure;
		public:
		
			double x_cart[3];
			double y_cart[3];
			double x_active[3];
            double x_active2[3];
			double y_active[3];
            double y_active2[3];
            FilterH FilterL;
			FilterH FilterLd;
			FilterH FilterRd;
			FilterH FilterR;
			FilterH FilterLandL;
			FilterH FilterLandR;
            FilterH FilterlandyL;
            FilterH FilterlandyR;
            FilterH Filtertime;


            FilterH Filterx;
            FilterH Filtery;
            FilterH Filterdx;
            FilterH Filterdy;
            FilterH Filterddx;
            FilterH Filterddy;
            FilterH FilterCOPX;
            FilterH FilterCOPY;
            FilterH FilterDeltaHIPX;
            FilterH FilterDeltaHIPY;
            FilterH Filterlzmpx;
            FilterH Filterrzmpx;
            FilterH Filterlzmpy;
            FilterH Filterrzmpy;
            std::vector<double> filterdata(double gcomx,double gcomy,double TsCart);
            Eigen::VectorXd filterCOP(double COPX,double COPY);
            Eigen::VectorXd filterDHIP(double HIPX,double HIPY);
            Eigen::VectorXd filterdatazmp(double lzmpx,double rzmpx,double lzmpy,double rzmpy);
            void MPC_constrained(double *X, double *U, double *NF, double *N, double Yt, double *Wt,double z_c,double acceleration,char);
            void UpdateStructure(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex);
            void initStructure(double gcomx[3],double gcomy[3],double zmpyref,double clearance);
            void Initialize(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex);
			double Poly3(double s0,double t0,double sf,double tf,double time);
            double meancond(double initialPos,double initialSpeed,double zc);
			double swingLeg(double swingFoot,double expectedPos,double remainTime,double T);
			double yfunct(double ydes,double remainTime,double legPos,double T);
			double zfunct(double footref,double footPosition,double xdes,double actualPosition,double clearance) ;
            void initialConditions(double*conditions,double step_length,double TsCarttep,double zc);
            void zmpTrajectory(double Tc,double xfinal);
            void feetUpdate();
            void footChange();
            void footReference();
            void ZMP_preview( double xfinal, double gcomx[3],double gcomy[3],double Z_C);
            void initfilters();
		public:
			
			DynamicWalkClass();
            DynamicWalkClass(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex);
            void ZMP_MPC(double xfinal,double gcomx[3],double gcomy[3],double Z_C);
};
}
#endif
