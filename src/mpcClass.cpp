#include "mpcClass.h"
/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

MPCClass::MPCClass(){

    this->Nu=10;
    this->N2=400;
    this->alfa=0;
    this->controlFlag==0;
    constraint=0;
    std::string FILEG="../../src/invGzmp.txt";
    std::string FILEH="../../src/invHzmp.txt";
    std::string FILEF="../../src/Fzmp.txt";

    this->sizeA=4;
    this->sizeB=4;
    this->sizeC=2;
    this->sizeD=2;

    this->States<<0,0,0;
    this->Ampc.resize(this->sizeA);
    this->Bmpc.resize(this->sizeB);
    this->Cmpc.resize(this->sizeC);
    this->Dmpc.resize(this->sizeD);
    /*TF IN discrete time for a second order integrator*/
    this->Ampc<<1, -3, 3, -1;
    this->Bmpc<<0,  -270.11e-6, 540.3e-6,  -270.11e-6;
    /*LOW PASS FILTER (
     *
     * [H,I]=butter(1,[0.001 0.9])
     *
     * )*/
    this->Cmpc<<0,1;
    this->Dmpc<<1.0000,-1;

    /*Initializations*/
    this->X.resize(N2+sizeA+1)	;
    this->U.resize(N2+sizeB+1)	;
    this->NF.resize(N2+sizeC+1)	;
    this->N.resize(N2+sizeD+1)	;
    this->invG.resize(Nu,N2);
    this->invH.resize(Nu,Nu);
    this->F.resize(Nu,N2);



    this->importGmatrix(FILEG,FILEH,FILEF);

    this->ConstraintB.resize(2*N2);
    this->ConstraintA.resize(2*N2,Nu);


    this->ConstraintA.block(0,0,N2,Nu)=this->F.transpose();
    this->ConstraintA.block(N2,0,N2,Nu)=-this->F.transpose();


}
MPCClass::~MPCClass() {}
/**
 * @brief MPCClass::importGmatrix
 * @param FILE locates the matrix G to be load
     * i.e FILE="../../src/Gd.txt";
     *
     */

void MPCClass::importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF){

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
void MPCClass::MPC(double Yt,double *Wt){


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
        //cout<<X[k+1]<<"	"<<X[k]<<endl;
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

    if(constraint){
        VectorXd optimalU;
        optimalU=this->saturateMPC(Err,Ybase,U);
        U[N2]=U[N2]+optimalU[0];
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
        return;
    }

}
/**
 * @brief integralCtrl::applyControl
 * @param States Vector that contains the states of the system X, Xd
 * @param controlEffort control Effort to be apply to the control law
 *                      AX+BU
 * @return desired state X in (K+1)
 */
double MPCClass::saturate(double PseudoZMP,double desiredZMP,double actualPosition,double z_c,double delta){

        double g=9.81;
        //
        if  (PseudoZMP > (desiredZMP+ delta)){
            PseudoZMP = (desiredZMP+ delta);
        }
        else if( PseudoZMP < (desiredZMP- delta) ){
            PseudoZMP = (desiredZMP- delta);
        }
        return  (actualPosition - PseudoZMP)*g/z_c;
}
VectorXd MPCClass::saturateMPC(double *Err, double *Ybase,std::vector<double> U){

    int m1=this->ConstraintA.cols();
    int n1=this->ConstraintA.rows();
    VectorXd Error(N2);
    for (int i=0;i<N2;i++){
        Error(i)=-Err[i];
    }

    double maxy=0.15;
    double miny=-0.15;
    VectorXd maxConstraint(N2);
    VectorXd minConstraint(N2);
    for (int i=0;i<N2;i++){
        maxConstraint(i)=maxy-Ybase[0];
        minConstraint(i)=Ybase[0]-miny;
    }
    this->ConstraintB.block(0,0,N2,1)=maxConstraint;
    this->ConstraintB.block(N2,0,N2,1)=minConstraint;

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
            //cout<<"I:   "<<i<<endl;
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
    for (int km=0;km<3;km++){
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
