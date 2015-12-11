#ifndef ZMPStabilizer_H
#define ZMPStabilizer_H
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include "FilterHClass.h"

using namespace Eigen;
using namespace std;

class ZMPStabilizer
{
public:

    ZMPStabilizer();
    ~ZMPStabilizer();
    ZMPStabilizer(double LQRgains[2], double sampleTime,std::string FILE);
    /**
     * @brief integralCtrl::apply
     * Apply the control law with either LQR or MPC controller
     * accoriding to the controlFlag variable
     * controlFlag==0 for LQR;   controlFlag==1 for MPC
     * @param ref array of size control.N2
     * @return the desired angle
     */
    double Fgcomx[3],Fgcomy[3];
    FilterH Filterx;
    FilterH Filtery;
    FilterH Filterdx;
    FilterH Filterdy;
    FilterH Filterddx;
    FilterH Filterddy;
    FilterH Filteralfa;
    FilterH Filteralfad;
    FilterH Filterbeta;
    FilterH Filterbetad;
    FilterH outFilter;
    double sampletime;
    std::vector<double> filterdata2(double alfa,double alfad,double beta,double betad);

    std::vector<double> filterdata(double gcomx,double gcomy,double TsCart);
    double apply(double *ref);
    /**
     * @brief States Position and velocity
     */
    Vector2d States;
    double offset;
    double offsety;
    /**
     * controlFlag==0 for LQR;   controlFlag==1 for MPC
     */
    int controlFlag;
    /*
     * MPC control parameters*/
    double alfa;
    int Nu;
    int N2;
    double z_c;
    double g;
    double m;
    int constraint;
    double acclimit;
    int sizeA;
    std::vector<double> X	;
private:
    //SS discrete system
    Matrix2d A;
    Vector2d B;
    Vector2d C;
    double D;
    double zmp;

    double freq;
    double controleffort;
    double LQRgains[2];



    int sizeB;
    int sizeC;
    int sizeD;


     VectorXd Ampc;
     VectorXd Bmpc;
     //Filter tranfer function for MPc
     VectorXd Cmpc;
     VectorXd Dmpc;
     //MPC internal variables

      std::vector<double> U	;
      std::vector<double> NF;
      std::vector<double> N;
      MatrixXd F;
      MatrixXd invH;
      VectorXd ConstraintB;
      MatrixXd ConstraintA;
      MatrixXd invG;


    void initfilters();
    double filterout();
    double LQRcontroller(Vector2d States,double reference);
    void importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF);
    void MPC(double Yt,double *Wt);
    double applyControl(Vector2d States,double controlEffort);
    VectorXd saturateMPC(double *Err, double *Ybase,std::vector<double> U);
};
#endif
