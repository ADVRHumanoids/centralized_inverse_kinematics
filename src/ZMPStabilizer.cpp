#include "ZMPStabilizer.h"

/**
 * @brief integralCtrl::integralCtrl
 * Default Contructor; develop for sample time of 5 ms
 * with a second order system and invGbalance as gains for the
 * MPC controller, gains and sizes are also pre-defined
 */

ZMPStabilizer::ZMPStabilizer()
{
    /* tunning parameter, should be consistent with invG*/
    this->Nu=4;
    this->N2=20;
    this->alfa=0;
    this->controlFlag=0;
    constraint=1;
    std::string FILEG="invGzmp2.txt";
    std::string FILEH="invHzmp2.txt";
    std::string FILEF="Fzmp2.txt";

    this->zmp=0;
    this->g=9.81;
    this->z_c=1;
    this->sampletime=0.005;
    /*LQR GAINS DEFINITION*/
//    Qc=100;
//    R =Qc*1e-7;
    this->LQRgains[1]=5.05;//8.4677;

    this->LQRgains[0]=8.68;//18.1173;
    /*SS description in discrate form*/
    this->A<<1,this->sampletime,
            0,1;
    this->B<<pow(this->sampletime,2)/2,
            this->sampletime;
    this->C<<1,0;
    this->D=-this->z_c/this->g;
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
    this->Bmpc<<-0.0540,0.1081,-0.0540;
    /*LOW PASS FILTER (
     *
     * [H,I]=butter(1,[0.001 0.9])
     *
     * )*/
    this->Cmpc<< 0.0929 , 0 ,-0.0929;
    this->Dmpc<<1.0000,-1.7828,0.8141;
//    this->Cmpc<<0.3375,0,-0.3375;

//    this->Dmpc<<1.0000,-0.6751 ,0.3249;

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

//    this->ConstraintA.block(0,0,Nu,Nu)=MatrixXd::Identity(Nu,Nu);
//    this->ConstraintA.block(Nu,0,Nu,Nu)=-MatrixXd::Identity(Nu,Nu);

//    this->ConstraintA.block(2*Nu,0,Nu,Nu)=MatrixXd::Identity(Nu,Nu);
//    this->ConstraintA.block(3*Nu,0,Nu,Nu)=-MatrixXd::Identity(Nu,Nu);


    this->ConstraintA.block(0,0,N2,Nu)=this->F.transpose();
    this->ConstraintA.block(N2,0,N2,Nu)=-this->F.transpose();

//    this->ConstraintA.block(4*Nu+2*N2,0,N2,Nu)=this->F.transpose();
//    this->ConstraintA.block(4*Nu+3*N2,0,N2,Nu)=-this->F.transpose();

    this->freq=20;
    this->initfilters();
}
/**
 * @brief integralCtrl::integralCtrl
 * Contructor; develop for a second order system sizes
 * of vectors and MPC parameters are pre-defined
 * @param LQRgains Gain scehdule for the MPC controller
 * @param sampleTime sample time defined by user
 * @param FILE Addres to the desired gain .txt file for the MPC
 */
ZMPStabilizer::~ZMPStabilizer(){}
/**
 * @brief integralCtrl::LQRcontroller
 *
 * @param States contains the states of the system i.e. X,Xd
 * @param reference Desired set point
 * @return desire state X (k+1)
 */
void ZMPStabilizer::initfilters(){
        Filterx.butterworth     (this->sampletime,this->freq*1,1);
        Filterdx.butterworth    (this->sampletime,this->freq*1,1);
        Filterddx.butterworth   (this->sampletime,this->freq*4,1);
        Filtery.butterworth     (this->sampletime,this->freq*4,1);
        Filterdy.butterworth    (this->sampletime,this->freq*4,1);
        Filterddy.butterworth   (this->sampletime,this->freq*4,3);
        Filteralfa.butterworth  (this->sampletime,this->freq*4,1);
        Filteralfad.butterworth (this->sampletime,this->freq*4,1);
        Filterbeta.butterworth  (this->sampletime,this->freq*4,1);
        Filterbetad.butterworth (this->sampletime,this->freq*4,1);
        this->Fgcomx[0]=0;this->Fgcomx[1]=0;this->Fgcomx[2]=0;
        this->Fgcomy[0]=0;this->Fgcomy[1]=0;this->Fgcomy[2]=0;
        outFilter.butterworth   (this->sampletime,this->freq*0.14,1);
        for (int i=1;i<30;i++)
            outFilter.applyFilter(0);
}

std::vector<double> ZMPStabilizer::filterdata(double gcomx,double gcomy,double TsCart){
    double gcom_old,gdcom_old,gcom_oldy,gdcom_oldy=0;

    gcom_old    = this->Fgcomx[0];
    gdcom_old   = this->Fgcomx[1];
    this->Fgcomx[0]   = Filterx.applyFilter(gcomx);	// gcom is the estimated COM in world coordinate
    double speedx	= (this->Fgcomx[0]-gcom_old)/TsCart;


    gcom_oldy	= this->Fgcomy[0];
    gdcom_oldy	= this->Fgcomy[1];
    this->Fgcomy[0]	= gcomy;	// gcom is the estimated COM in world coordinate
    double speedy	= (this->Fgcomy[0]-gcom_oldy)/TsCart;

    this->Fgcomx[1]=speedx;
    this->Fgcomy[1]=speedy;

    this->Fgcomx[2] = (this->Fgcomx[1]-gdcom_old)/TsCart;
    this->Fgcomy[2] = (this->Fgcomy[1]-gdcom_oldy)/TsCart;

    std::vector<double> returnvector(6,0);


    returnvector[0]   = this->Fgcomx[0];//gcomx; // gcom is the estimated COM in world coordinate
    returnvector[1]   = Filterdx.applyFilter  (this->Fgcomx[1]);
    returnvector[2]   = Filterddx.applyFilter (this->Fgcomx[2]);

    returnvector[3]   = Filtery.applyFilter   (gcomy); // gcom is the estimated COM in world coordinate
    returnvector[4]   = Filterdy.applyFilter  (this->Fgcomy[1]);
    returnvector[5]   = Filterddy.applyFilter (this->Fgcomy[2]);

    return returnvector;
}
std::vector<double> ZMPStabilizer::filterdata2(double alfa,double alfad,double beta,double betad){
    std::vector<double> returnvector(4,0);
    returnvector[0]   = Filteralfa.applyFilter  (alfa);
    returnvector[1]   = Filteralfad.applyFilter  (alfad);
    returnvector[2]   = Filterbeta.applyFilter  (beta);
    returnvector[3]   = Filterbetad.applyFilter  (betad);
    return returnvector;
}
double ZMPStabilizer::filterout(){
    return outFilter.applyFilter  (this->U[N2]);
}

double ZMPStabilizer::LQRcontroller(Vector2d States, double reference)
{
        double LQRinput=this->LQRgains[0]*(reference-States(0))+this->LQRgains[1]*(-States(0));
        this->controleffort=LQRinput;
        return applyControl(States,LQRinput);

}
/**
 * @brief integralCtrl::importGmatrix
 * load the gains for the MPC controller
 * @param FILE addres to the gains to be load
 */
void ZMPStabilizer::importGmatrix(std::string FILEG,std::string FILEH,std::string FILEF){
    ifstream in;
      //model matrix
        in.open(FILEG.c_str());
       // in.open("../../src/RealGmatrix.txt");

        for (int i=0;i<Nu;i++){
            for (int j=0;j<N2;j++){
                if(!in.eof()){
                    in>>invG(i,j);
                    cout<<invG(i,j)<<endl;
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
void ZMPStabilizer::MPC(double Yt,double *Wt){


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
        N[k]=(N_c-N_d);
    }

    // 6= calcula Ybase=X(N2-N1+1:-1:1)+N(N2-N1+1:-1:1); k=N1....N2
    for(k=N2-1;k>=0;k--){
        Ybase[N2-1-k]=N[k]+X[k];
    }
    //X[N2]=Yt; // SERIE-PARALELO
    // 7= New control input U=U+ inv(G'*G)*G'*Err
    double Ref[N2+1];
    Ref[0]=Yt;
    for (k=0;k<N2;k++){

        Ref[k+1]=alfa*Ref[k]+(1-alfa)*Wt[k];
    }
    for(k=0;k<N2;k++){
        Err[k]=(Ref[k+1]-Ybase[k]);
    }
     if(this->constraint){
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
double ZMPStabilizer::applyControl(Vector2d States,double controlEffort){
    Vector2d temporal=this->A*States+this->B*controlEffort;

    this->States=temporal;
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
double ZMPStabilizer::apply(double *ref){
    if (this->controlFlag==0){
        return this->LQRcontroller(this->States,ref[0]);
    }
    else{
        this->zmp=States(0)-(this->z_c/this->g)*this->U[this->N2];
        this->MPC(this->zmp,ref);
        double uplimit=0.15;
        double lowlimit=0.05;

        if (this->constraint==0){
            uplimit=2000000.15;
            lowlimit=2000000.05;
        }
        double PseudoZMP=States(0)-(this->z_c/this->g)*this->U[this->N2];
        if  (PseudoZMP > (ref[0]+uplimit)){
            PseudoZMP = (ref[0]+uplimit);
        }
        else if( PseudoZMP < (ref[0]-lowlimit) ){
            PseudoZMP = (ref[0]-lowlimit);
        }
        this->U[this->N2]=States(0) - PseudoZMP*g/z_c;
        U[N2]=filterout();
        return this->applyControl(this->States,this->U[this->N2]);
    }

}

VectorXd ZMPStabilizer::saturateMPC(double *Err, double *Ybase,std::vector<double> U){

    int m1=this->ConstraintA.cols();
    int n1=this->ConstraintA.rows();
    VectorXd Error(N2);
    for (int i=0;i<N2;i++){
        Error(i)=-Err[i];
    }
//    double Umax=100000;
//    double Umin=-100000;

//    for (int i=0;i<Nu;i++){
//        this->ConstraintB(i)=Umax*pow(Umax,i)-U[N2-i];
//        this->ConstraintB(i+Nu)=U[N2-i]-Umin*pow(Umax,i);
//    }

//    double dU=U[N2+1]-U[N2];
//    double deltaUmax=8000000;
//    double deltaUmin=-8000000;

//    this->ConstraintB.block(2*Nu,0,Nu,1)=(VectorXd::Ones(Nu,1))*(deltaUmax+dU);
//    this->ConstraintB.block(3*Nu,0,Nu,1)=(VectorXd::Ones(Nu,1))*(-deltaUmin-dU);


    double maxy=0.15e15;
    double miny=-0.5e15;
    VectorXd maxConstraint(N2);
    VectorXd minConstraint(N2);
    for (int i=0;i<N2;i++){
        maxConstraint(i)=maxy-Ybase[0];
        minConstraint(i)=Ybase[0]-miny;
    }
    this->ConstraintB.block(0,0,N2,1)=maxConstraint;
    this->ConstraintB.block(N2,0,N2,1)=minConstraint;

//    double dmaxy=2e-2;
//    double dminy=-2e-2;
//    VectorXd dmaxConstraint(N2);
//    VectorXd dminConstraint(N2);
//    for (int i=0;i<N2;i++){
//        dmaxConstraint(i)=(X[N2-i]+dmaxy)-Ybase[i];
//        dminConstraint(i)=Ybase[i]-(X[N2-i]+dminy);
//    }
//    this->ConstraintB.block(4*Nu+2*N2,0,N2,1)=dmaxConstraint;
//    this->ConstraintB.block(4*Nu+3*N2,0,N2,1)=dminConstraint;

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
        if (al<10e-3){
            break;
        }
    }
    MatrixXd temporal=(this->invH*this->ConstraintA.transpose());
    return -this->invH*f-temporal*lambda;
}
