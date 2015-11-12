/**
    @file IntegralControl.cpp
    @author Juan Alejandro Castano, Zhibin Li, juan[dot]castano[at]iit[dot]it, zhibin[dot]li[at]iit[dot]it

    @section License
    Copyright (C) 2014 Department of Advanced Robotics, Italian Institute of Technology, all rights reserved.

    @section Description
    Integral Controller Class, mainly used for upper body attitude control, can also be used for other control tasks.
    Develop for sample time of 5 ms with a second order system and invG as gains for the MPC controller,
    gains and sizes are also pre-defined.
*/

#include "IntegralControl.h"
IntegralControl::IntegralControl()
{
    std::string FILEG="invGbalance.txt";
    std::string FILEH="invHbalance.txt";
    std::string FILEF="Fbalance.txt";
    constraints=0;
    this->sampletime=0.005;
    /*LQR GAINS DEFINITION*/
//    Qc=100;
//    R =Qc*1e-7;
    this->LQRgains[1]=84;
    this->LQRgains[0]=3532;
    /*SS description in discrate form*/
    this->A<<1,this->sampletime,
            0,1;
    this->B<<pow(this->sampletime,2)/2,
            this->sampletime;

    /*******************************/
    /*MPC VARIABLES*/
    /*SYSTEM AND FILTER TRANSFER FUCNTION (TF) LENGHTS*/
    this->sizeA=3;
    this->sizeB=3;
    this->sizeC=3;
    this->sizeD=3;
    this->States<<0,0;
    this->Ampc.resize(this->sizeA);
    this->Bmpc.resize(this->sizeB);
    this->Cmpc.resize(this->sizeC);
    this->Dmpc.resize(this->sizeD);
    /*TF IN discrete time for a second order integrator*/
    this->Ampc<<1, -2, 1;
    this->Bmpc<<0,pow(sampletime,2)/2,pow(sampletime,2)/2;
    /*LOW PASS FILTER (
     *
     * [H,I]=butter(1,[0.001 0.9])
     *
     * )*/
//    this->Cmpc<<0.1234,0,-0.1234;
//    this->Dmpc<<1.0000,-1.7525,    0.7533;

//    this->Cmpc<<0,1;
//    this->Dmpc<<1,-2,1;

    this->Cmpc<<0.2742,0,-0.2742;
    this->Dmpc<<1,-0.5485, 0.4515;
//    this->Cmpc<< 0,1;
//    this->Dmpc<<1,-2,1;
    /* tunning parameter, should be consistent with invG*/
    this->Nu=4;
    this->N2=10;
    this->alfa=0;
    this->controlFlag==0;
    /*Initializations*/
    this->X.resize(N2+sizeA+1)    ;
    this->U.resize(N2+sizeB+1)    ;
    this->NF.resize(N2+sizeC+1)    ;
    this->N.resize(N2+sizeD+1)    ;
    this->invG.resize(Nu,N2);
    this->invH.resize(Nu,Nu);
    this->F.resize(Nu,N2);



    this->importGmatrix(FILEG,FILEH,FILEF);

    this->ConstraintB.resize(4*N2);
    this->ConstraintA.resize(4*N2,Nu);


    this->ConstraintA.block(0,0,N2,Nu)=this->F.transpose();
    this->ConstraintA.block(N2,0,N2,Nu)=-this->F.transpose();

    this->ConstraintA.block(2*N2,0,N2,Nu)=this->F.transpose();
    this->ConstraintA.block(3*N2,0,N2,Nu)=-this->F.transpose();

    this->freq=50;
    this->TsCart=0.005;
    this->initfilters();

}
/**
 * @brief IntegralControl::IntegralControl
 * Contructor; develop for a second order system sizes
 * of vectors and MPC parameters are pre-defined
 * @param LQRgains Gain scehdule for the MPC controller
 * @param sampleTime sample time defined by user
 * @param FILE Addres to the desired gain .txt file for the MPC
 */
IntegralControl::IntegralControl(double LQRgains[2],double sampleTime,std::string FILEG,std::string FILEH,std::string FILEF)
{

    this->sampletime=sampleTime;
    this->LQRgains[1]=LQRgains[1];
    this->LQRgains[0]=LQRgains[0];
    /*SS description in discrate form*/
    this->A<<1,this->sampletime,0,this->sampletime;
    this->B<<pow(this->sampletime,2)/2,this->sampletime;

    /*******************************/
    /*MPC VARIABLES*/
    /*SYSTEM AND FILTER TRANSFER FUCNTION (TF) LENGHTS*/
    this->sizeA=3;
    this->sizeB=3;
    this->sizeC=3;
    this->sizeD=3;

    this->Ampc.resize(this->sizeA);
    this->Bmpc.resize(this->sizeB);
    this->Cmpc.resize(this->sizeC);
    this->Dmpc.resize(this->sizeD);
    /*TF IN discrete time for a second order integrator*/
    this->Ampc<<1, -2, 1;
    this->Bmpc<<0,pow(sampletime,2)/2,pow(sampletime,2)/2;
    /*LOW PASS FILTER (
     *
     * [H,I]=butter(1,[0.001 0.9])
     *
     * )*/
    this->Cmpc<<0.1246,0,-0.1246;
    this->Dmpc<<1.0000,-1.7421 ,0.7508;
    /* tunning parameter, should be consistent with invG*/
    this->Nu=7;
    this->N2=10;
    this->alfa=0;
    /*Initializations*/
    this->X.resize(N2+sizeA+1)    ;
    this->U.resize(N2+sizeB+1)    ;
    this->NF.resize(N2+sizeC+1)    ;
    this->N.resize(N2+sizeD+1)    ;
    this->invG.resize(Nu,N2);
    this->invH.resize(Nu,Nu);
    this->F.resize(Nu,N2);


    this->importGmatrix(FILEG,FILEH,FILEF);

}
IntegralControl::~IntegralControl(){}
/**
 * @brief IntegralControl::LQRcontroller
 *
 * @param States contains the states of the system i.e. X,Xd
 * @param reference Desired set point
 * @return desire state X (k+1)
 */

void IntegralControl::initfilters(){
    Filteralfa.butterworth   (TsCart,this->freq*0.5,1);
    Filteralfad.butterworth   (TsCart,this->freq*0.5,1);
}
std::vector<double> IntegralControl::filterdata2(double alfa,double alfad){
    std::vector<double> returnvector(2,0);
    returnvector[0]   = Filteralfa.applyFilter  (alfa);
    returnvector[1]   = Filteralfad.applyFilter  (alfad);
    return returnvector;
}
double IntegralControl::LQRcontroller(Vector2d States, double reference)
{
        double LQRinput=this->LQRgains[0]*(reference-States(0))+this->LQRgains[1]*(-States(0));
        this->controleffort=LQRinput;
        return applyControl(States,LQRinput);

}
/**
 * @brief IntegralControl::importGmatrix
 * load the gains for the MPC controller
 * @param FILE addres to the gains to be load
 */
void IntegralControl::importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF){

    ifstream in;
      //model matrix
        in.open(FILEG.c_str());
       // in.open("../../src/RealGmatrix.txt");

        for (int i=0;i<Nu;i++){
            for (int j=0;j<N2;j++){
                if(!in.eof()){
                    in>>invG(i,j);
                }
            }
        }
        //optimization Hmatrix
        in.close();

        in.open(FILEH.c_str());
        for (int i=0;i<Nu;i++){
            for (int j=0;j<Nu;j++){
                if(!in.eof()){
                    in>>this->invH(i,j);
                }
            }
        }
        //optimization F matrix
        in.close();
        in.open(FILEF.c_str());
        for (int i=0;i<Nu;i++){
            for (int j=0;j<N2;j++){
                if(!in.eof()){
                    in>>F(i,j);
                }
            }
        }
        in.close();

}
/**
 * @brief MPCClass::MPC Apply EPSAC control technique to the
 * cart table model
 *
 * @param Yt real ZMP state
 * @param Wt zmp trajectory of size N2
 */
void IntegralControl::MPC(double Yt,double *Wt){


    double Uopt[Nu], Ybase[N2], Err[N2];

   // 1= shift EPSAC database and store base input
    int k=0;
    double Ut=U[N2];
    for (k = (sizeB+N2); k >=1 ; k--) {
        U[k] = U[k-1];      // [Corrimiento a la derecha]
    }
    for (k = (sizeA+N2); k >=1 ; k--) {
        X[k] = X[k-1];      // [Corrimiento a la derecha]
    }
    for (k = (sizeD+N2); k >=1 ; k--) {
        N[k] = N[k-1];      // [Corrimiento a la derecha]
    }
    for (k = (sizeC+N2); k >=1 ; k--) {
        NF[k] = NF[k-1];     // [Corrimiento a la derecha]
    }
    for (k=0; k<=N2; k++){
        U[k]=Ut;       // Store Control Action
    }
    // 2= compute EPSAC base-response x(t+k/t), k=0..N2
    // Model Output
    double X_b=0,X_a=0;
    int j_b=0,j_a=0;
    for (k=N2;k>=0;k--){
        X_b=0;
        for(j_b=0;j_b<=(sizeB-1);j_b++){
            X_b=Bmpc(j_b)*U[k+j_b]+X_b;
        }
        X_a=0;
        for(j_a=1;j_a<=(sizeA-1);j_a++){
            X_a=Ampc(j_a)*X[k+j_a]+X_a;
        }
        X[k]=X_b-X_a;
        //cout<<X[k+1]<<"    "<<X[k]<<endl;
    }
    // 3= compute n(t)=y(t)-x(call) with x(call) the model output
    N[N2]=Yt-X[N2];
    // 4= compute nf(t)=D/C*n(call) and put nf(t+k/t)=0, k=1..N2
    double NF_d=0,NF_c=0;
    int j_d=0,j_c=0;
    NF_d=0;
    for(j_d=0;j_d<=(sizeD-1);j_d++){
        NF_d=Dmpc(j_d)*N[N2+j_d]+NF_d;
    }
    NF_c=0;
    int aux_en;
    if(sizeC>=1){
        aux_en=0;
    }
    else{
        aux_en=1;
    }
    for(j_c=aux_en;j_c<=(sizeC-1);j_c++){
        NF_c=Cmpc(j_c)*NF[N2+j_c]+NF_c;
    }
    NF[N2]=NF_d-NF_c;
    for(k=0;k<N2;k++){
        NF[k]=0;
    }
    // 5= compute n(t+k/t)=C/D*nf(t+k/t), k=1..N2
    double N_d=0,N_c=0;
    for (k=N2-1;k>=0;k--){
        N_c=0;
        int aux_en;
        if(sizeC>=1){
            aux_en=0;
        }
        else{
            aux_en=1;
        }
        for(j_c=aux_en;j_c<=(sizeC-1);j_c++){
            N_c=Cmpc(j_c)*NF[k+j_c+1]+N_c;
        }
        N_d=0;
        for(j_d=1;j_d<=(sizeD-1);j_d++){
            N_d=Dmpc(j_d)*N[k+j_d]+N_d;
        }
        N[k]=N_c-N_d;
    }

    // 6= calcula Ybase=X(N2-N1+1:-1:1)+N(N2-N1+1:-1:1); k=N1....N2
    for(k=N2-1;k>=0;k--){
        Ybase[N2-1-k]=N[k]+X[k];
    }
    X[N2]=Yt; // SERIE-PARALELO
    // 7= New control input U=U+ inv(G'*G)*G'*Err
    double Ref[N2+1];
    Ref[0]=Yt;
    for (k=0;k<N2;k++){

        Ref[k+1]=alfa*Ref[k]+(1-alfa)*Wt[k];
    }
    for(k=0;k<N2;k++){
        Err[k]=(Ref[k+1]-Ybase[k]);
    }


    if(this->constraints){
        VectorXd prueba;
        prueba=this->saturateMPC(Err,Ybase,U);
        U[N2]=U[N2]+prueba[0];
        return;
    }
    else{
        for(k=0;k<Nu;k++){
            Uopt[k]=0;
        }
        for(int i=0;i<Nu;i++){
            for(k=0;k<N2;k++){
                Uopt[i] += invG(i,k)*Err[k];
            }
        }
        U[N2]=U[N2]+Uopt[0];
        double Umax=100;
        if(U[N2]>Umax)
            U[N2]=Umax;
        else if(U[N2]<-Umax)
            U[N2]=-Umax;
        return;
    }

}
/**
 * @brief IntegralControl::applyControl
 * @param States Vector that contains the states of the system X, Xd
 * @param controlEffort control Effort to be apply to the control law
 *                      AX+BU
 * @return desired state X in (K+1)
 */
double IntegralControl::applyControl(Vector2d States,double controlEffort){
    Vector2d temporal=this->A*States+this->B*controlEffort;
    return temporal(0);
}
/**
 * @brief integralCtrl::apply
 * Apply the control law with either LQR or MPC controller
 * accoriding to the controlFlag variable
 * controlFlag==0 for LQR;   controlFlag==1 for MPC
 * @param ref array of size control.N2
 * @return the desired angle
 */
double IntegralControl::apply(double *ref){
    if (this->controlFlag==0){
        return this->LQRcontroller(this->States,ref[this->Nu]);
    }
    else{
        this->MPC(States(0),ref);
        return this->applyControl(this->States,this->U[this->N2]);
    }

}
Vector3d IntegralControl::DynamicCompensator(Matrix3d R,Vector3d COMvector , double mb, double mleg)
{
    //static double mb=PELVIS_MASS+TORSO_MASS;
    //static double mleg=THIGH_MASS+CALF_MASS;
    static double m1=mb;
    static double m2=mleg;
    Matrix3d I = Matrix3d::Identity(3,3);
    Vector3d delta_r = -m1/(m1+m2)*(R-I)*COMvector;
    return delta_r;

}
VectorXd IntegralControl::saturateMPC(double *Err, double *Ybase,std::vector<double> U){

    int m1=this->ConstraintA.cols();
    int n1=this->ConstraintA.rows();
    VectorXd Error(N2);
    for (int i=0;i<N2;i++){
        Error(i)=-Err[i];
    }
    double maxy=20;
    double miny=-20;
    VectorXd maxConstraint(N2);
    VectorXd minConstraint(N2);
    for (int i=0;i<N2;i++){
        maxConstraint(i)=maxy-Ybase[0];
        minConstraint(i)=Ybase[0]-miny;
    }
    this->ConstraintB.block(0,0,N2,1)=maxConstraint;
    this->ConstraintB.block(N2,0,N2,1)=minConstraint;

    double dmaxy=0.24500;
    double dminy=-0.24500;
    VectorXd dmaxConstraint(N2);
    VectorXd dminConstraint(N2);
    for (int i=0;i<N2;i++){
        dmaxConstraint(i)=(X[N2-i]+dmaxy)-Ybase[i];
        dminConstraint(i)=Ybase[i]-(X[N2-i]+dminy);
    }
    this->ConstraintB.block(2*N2,0,N2,1)=dmaxConstraint;
    this->ConstraintB.block(3*N2,0,N2,1)=dminConstraint;

    VectorXd f(Nu);
    VectorXd eta(Nu);
    f=this->F*Error;
    eta=-this->invH*f;

    int kk=0;
    double evalConstraint;
    for (int i=0;i<n1;i++){
        evalConstraint=0;
        evalConstraint=this->ConstraintA.row(i)*eta;

        if(evalConstraint>this->ConstraintB[i]){
            kk=kk+1;
            break;
        }
    }

    if (kk==0){
       return eta;

    }
    MatrixXd P(n1,n1);
    P.resize(n1,n1);
    P=this->ConstraintA*(this->invH*this->ConstraintA.transpose());
    VectorXd d(n1);
    d=(this->ConstraintA*(this->invH*f)+this->ConstraintB);

    int m=d.rows();
    VectorXd x_ini=VectorXd::Zero(m);
    VectorXd lambda=x_ini;
    double al=10;
    double w=0;
    for (int km=0;km<38;km++){
        VectorXd lambda_p=lambda;
        for (int i=0;i<m;i++){

            w=P.row(i)*lambda-P(i,i)*lambda(i);
            w=w+d(i);
            double la=-w/P(i,i);
            if (la>=0){
                lambda(i)=la;
            }
            else{
                lambda(i)=0;
            }

        }
         al=(lambda-lambda_p).transpose()*(lambda-lambda_p);
        if (al<10e-8){
            break;
        }
    }
    MatrixXd temporal=(this->invH*this->ConstraintA.transpose());
    return -this->invH*f-temporal*lambda;
}
