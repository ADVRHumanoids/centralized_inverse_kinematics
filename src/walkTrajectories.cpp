#include "walkTrajectories.h"
using namespace dynamicWalk;

DynamicWalkClass::DynamicWalkClass(){
    //default constructor

    x_cart[0]=x_cart[1]=x_cart[2]=0;
    y_cart[0]=y_cart[1]=y_cart[2]=0;

    x_active[0]=x_active[1]=x_active[2]=0;
    y_active[0]=y_active[1]=y_active[2]=0;

    this->HALF_HIP_WIDTH=0.14;


    //initialize in the default values
    this->Initialize(0.2,0.9,HALF_HIP_WIDTH*2,0.8,HALF_HIP_WIDTH,0.2,0.05);
    std::string FILE="invGZMP.txt";
    std::string FILE2="Gd.txt";
    std::string FILEH="invHZMP.txt";
    std::string FILEF="FZMP.txt";

    this->pendulumModel.TsCart=TsCart;
    controlM.importGmatrix(FILE,FILEH,FILEF);
    controlMY.importGmatrix(FILE,FILEH,FILEF);
    controlP.loadGains(FILE2);
    this->initfilters();

}
DynamicWalkClass::DynamicWalkClass(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex){
    //constructor with parameter
    double Tc=sqrt(z_c/9.81);
    relativeflag=0;

    //using initialization parameter
    this->Initialize(stepLength,stepTime,stepLengthy,z_c,zmpyref,DSPhasePercent,footEdgex);
    std::string FILEG="invGZMP.txt";
    std::string FILE2="Gd.txt";
    std::string FILEH="invHZMP.txt";
    std::string FILEF="FZMP.txt";

    controlM.importGmatrix(FILEG,FILEH,FILEF);
    controlMY.importGmatrix(FILEG,FILEH,FILEF);
    controlP.loadGains(FILE2);
    pendulumModel.TsCart=TsCart;
    this->initfilters();
}
void DynamicWalkClass::initfilters(){

    //Filter positionY
    FilterlandyL.butterworth(TsCart,5,1);
    FilterlandyR.butterworth(TsCart,5,1);
    //Filter land y into LAND L/R
    FilterL.butterworth		(TsCart,2,1);
    FilterR.butterworth		(TsCart,2,1);
    //Filter Speed
    FilterLd.butterworth	(TsCart,5,1);
    FilterRd.butterworth	(TsCart,5,1);
    //Filter pos Left/Right 2
    FilterLandL.butterworth	(TsCart,5,1);
    FilterLandR.butterworth	(TsCart,5,1);
    //Filter TAO
    Filtertime.butterworth	(TsCart,5,1);

    FilterLandR.initialValue(0);
    FilterLandL.initialValue(0);

Filterx.butterworth     (TsCart,dynamicWalk::freq*4,1);
Filterdx.butterworth    (TsCart,dynamicWalk::freq*4,1);
Filterddx.butterworth   (TsCart,dynamicWalk::freq*2,1);
Filtery.butterworth     (TsCart,dynamicWalk::freq*4,1);
Filterdy.butterworth    (TsCart,dynamicWalk::freq*1,1);
Filterddy.butterworth   (TsCart,dynamicWalk::freq*1,3);

FilterCOPX.butterworth  (TsCart,dynamicWalk::freq*0.5,1);
FilterCOPY.butterworth (TsCart,dynamicWalk::freq*0.5,1);
FilterDeltaHIPX.butterworth  (TsCart,dynamicWalk::freq*0.5,1);
FilterDeltaHIPY.butterworth (TsCart,dynamicWalk::freq*0.5,1);

Filterlzmpx.butterworth  (TsCart,dynamicWalk::freq*0.4,1);
Filterrzmpx.butterworth  (TsCart,dynamicWalk::freq*0.4,1);
Filterlzmpy.butterworth  (TsCart,dynamicWalk::freq*0.4,1);
Filterrzmpy.butterworth  (TsCart,dynamicWalk::freq*0.4,1);
this->Fgcomx[0]=0;this->Fgcomx[1]=0;this->Fgcomx[2]=0;
this->Fgcomy[0]=0;this->Fgcomy[1]=0;this->Fgcomy[2]=0;
}
std::vector<double> DynamicWalkClass::filterdata(double gcomx,double gcomy,double TsCart){
//double filterini=0;




double gcom_old,gdcom_old,gcom_oldy,gdcom_oldy=0;

    gcom_old    = this->Fgcomx[0];
    gdcom_old   = this->Fgcomx[1];
    //this->Fgcomx[0]   = gcomx*cos(pos[2])+gcomy*sin(pos[2]);	// gcom is the estimated COM in world coordinate
    this->Fgcomx[0]   = gcomx;	// gcom is the estimated COM in world coordinate
    double speedx	= (this->Fgcomx[0]-gcom_old)/TsCart;


    gcom_oldy	= this->Fgcomy[0];
    gdcom_oldy	= this->Fgcomy[1];
    //this->Fgcomy[0]	= -gcomx*sin(pos[2])+gcomy*cos(pos[2]);	// gcom is the estimated COM in world coordinate
    this->Fgcomy[0]	= gcomy;	// gcom is the estimated COM in world coordinate
    double speedy	= (this->Fgcomy[0]-gcom_oldy)/TsCart;

    this->Fgcomx[1]=speedx;
    this->Fgcomy[1]=speedy;

    //this->Fgcomx[1]=speedx*cos(pos[2])+speedy*sin(pos[2]);
    //this->Fgcomy[1]=-speedx*sin(pos[2])+speedy*cos(pos[2]);

    this->Fgcomx[2] = (this->Fgcomx[1]-gdcom_old)/TsCart;
    this->Fgcomy[2] = (this->Fgcomy[1]-gdcom_oldy)/TsCart;

    std::vector<double> returnvector(6,0);


    returnvector[0]   = gcomx; // gcom is the estimated COM in world coordinate
    returnvector[1]   = Filterdx.applyFilter  (this->Fgcomx[1]);
    returnvector[2]   = Filterddx.applyFilter (this->Fgcomx[2]);

    returnvector[3]   = Filtery.applyFilter   (gcomy); // gcom is the estimated COM in world coordinate
    returnvector[4]   = Filterdy.applyFilter  (this->Fgcomy[1]);
    returnvector[5]   = Filterddy.applyFilter (this->Fgcomy[2]);

    return returnvector;
}
Eigen::VectorXd DynamicWalkClass::filterCOP(double COPX,double COPY){
    Eigen::VectorXd returnvector(2);
    returnvector[0]   = FilterCOPX.applyFilter  (COPX);
    returnvector[1]   = FilterCOPY.applyFilter  (COPY);
    return returnvector;
}
Eigen::VectorXd DynamicWalkClass::filterDHIP(double HIPX,double HIPY){
    Eigen::VectorXd returnvector(2);
    returnvector[0]   = FilterDeltaHIPX.applyFilter  (HIPX);
    returnvector[1]   = FilterDeltaHIPY.applyFilter  (HIPY);
    return returnvector;
}
Eigen::VectorXd DynamicWalkClass::filterdatazmp(double lzmpx,double rzmpx,double lzmpy,double rzmpy){

    Eigen::VectorXd returnvector(6);
    returnvector[0]   = Filterlzmpx.applyFilter  (lzmpx);
    returnvector[1]   = Filterlzmpy.applyFilter  (lzmpy);
    returnvector[2]   = 0;

    returnvector[3]   = Filterrzmpy.applyFilter  (rzmpx);
    returnvector[4]   = Filterrzmpy.applyFilter  (rzmpy);
    returnvector[5]   = 0;
    return returnvector;
}
void DynamicWalkClass::initStructure(double gcomx[3],double gcomy[3],double zmpyref,double clearance){

        this->ZMPstructure.stop=0;
        this->ZMPstructure.xfinal=gcomx[0];
        this->ZMPstructure.actualPos=gcomx[0];
        this->ZMPstructure.actualPosY=gcomy[0];
        this->ZMPstructure.actualSpeed=gcomx[1];
        this->ZMPstructure.actualSpeedy=gcomy[1];
        this->ZMPstructure.footPosition=gcomx[0];
        this->ZMPstructure.leftFoot=gcomx[0];
        this->ZMPstructure.rightFoot=gcomx[0];
        this->ZMPstructure.LFX=gcomx[0];
        this->ZMPstructure.RFX=gcomx[0];
        this->ZMPstructure.clearance=clearance;
        this->ZMPstructure.position=this->ZMPstructure.desirexf;
        for (int i=0;i<zmpref2.matrix_size;i++){

            for (int j=0;j<N2;j++){
                zmpref2.zmpMx[i][j]=gcomx[0];
            }
        }
        for (int i=0;i<zmpref2.matrix_size;i++){
            for (int j=0;j<N2;j++){

                    zmpref2.zmpMy[i][j]=gcomy[0];

            }
        }
        for (int j=0;j<N2;j++){
                this->ZMPstructure.zmpx[j]=(gcomx[0]);
                this->ZMPstructure.zmpy[j]=(gcomy[0]);
        }

        x_active[0]=(gcomx[0]);
        y_active[0]=(gcomy[0]);
        x_active[1]=(gcomx[1]);
        y_active[1]=(gcomy[1]);
        x_active[2]=(gcomx[2]);
        y_active[2]=(gcomy[2]);

}
void DynamicWalkClass::Initialize(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex){

    double Tc=sqrt(z_c/9.81);
    double conditionx[2];
    double conditiony[2];

    this->ZMPstructure.clearance=0.05;
    this->ZMPstructure.zmpLength=N2;
    this->ZMPstructure.zmpLengthy=N2;
    //flags
    this->ZMPstructure.stepCount=0;
    this->ZMPstructure.goLeft=0;
    this->ZMPstructure.goRight=0;
    this->ZMPstructure.footChange=1;
    //measures
    this->ZMPstructure.actualPos=0;
    this->ZMPstructure.actualSpeed=0;
    this->ZMPstructure.actualPosY=0;
    this->ZMPstructure.actualSpeedy=0;
    this->ZMPstructure.xfinal=0;
    //wlaking characteristics
    this->ZMPstructure.stepLength=stepLength;
    this->ZMPstructure.stepLengthy=stepLengthy;
    this->ZMPstructure.offset=stepLength/2;
    this->ZMPstructure.desirexf=this->ZMPstructure.offset/2;
    this->ZMPstructure.stepLength1=stepLength;
    this->ZMPstructure.z_c=z_c;
    //internal varibles

    for (int i=0;i<zmpref2.matrix_size;i++){
           for (int j=0;j<N2;j++){
               zmpref2.zmpMy[i][j]=0;

           }
       }
    for (int i=0;i<zmpref2.matrix_size;i++){
        for (int j=0;j<N2;j++){

            zmpref2.zmpMx[i][j]=0;
        }
    }
    for (int j=0;j<int(Tprev/TsCart+0.5);j++){
            this->ZMPstructure.zmpx[j]=0;
            this->ZMPstructure.zmpy[j]=0;
    }
    initialConditions(conditionx,stepLength,stepTime,z_c);
    this->ZMPstructure.zeroSpeed1=meancond(conditionx[0],conditionx[1],z_c);
    this->ZMPstructure.zeroSpeed=0;

    initialConditions(conditiony,stepLengthy,stepTime,z_c);
    this->ZMPstructure.zeroSpeedy=meancond(conditiony[0],conditiony[1],z_c);
    this->ZMPstructure.DSPhaseSize=int((stepTime/TsCart+0.5)*DSPhasePercent);
    this->ZMPstructure.SSPhaseSize=int((stepTime/TsCart+0.5)*(1-DSPhasePercent));
    this->ZMPstructure.DSReference=(conditionx[0]*cosh(this->ZMPstructure.DSPhaseSize*TsCart/Tc)+Tc*conditionx[1]*sinh(this->ZMPstructure.DSPhaseSize*TsCart/Tc));
    this->ZMPstructure.DSReference=stepLength/2+this->ZMPstructure.DSReference;


    this->ZMPstructure.zmpyref=zmpyref;
    //dynamic variables
    this->ZMPstructure.time=0;
    this->ZMPstructure.footPosition=0;
    this->ZMPstructure.footy=zmpyref;
    this->ZMPstructure.SSRemainTime=0;
    this->ZMPstructure.yTemp=0;
    this->ZMPstructure.leftFoot=0;
    this->ZMPstructure.rightFoot=0;
    //feet
    this->ZMPstructure.LFX=0;
    this->ZMPstructure.RFX=0;
    this->ZMPstructure.LFZ=0;
    this->ZMPstructure.RFZ=0;
    this->ZMPstructure.LFY=zmpyref;
    this->ZMPstructure.RFY=-zmpyref;


    this->ZMPstructure.footEdgex=footEdgex;
    this->ZMPstructure.footEdgeY=footEdgex;
    this->ZMPstructure.desiredXf=conditionx[1];
    //matrix and vectors
}
void DynamicWalkClass::UpdateStructure(double stepLength,double stepTime,double stepLengthy,double z_c,double zmpyref,double DSPhasePercent,double footEdgex){
    double Tc=sqrt(z_c/9.81);
    double conditionx[2];
    double conditiony[2];

    this->ZMPstructure.stepLength=stepLength;
    this->ZMPstructure.stepLengthy=stepLengthy;
    this->ZMPstructure.stepLength1=stepLength;
    this->ZMPstructure.z_c=z_c;
    this->ZMPstructure.zmpyref=zmpyref;
    //dynamic variables

    initialConditions(conditionx,stepLength,stepTime,z_c);
    this->ZMPstructure.zeroSpeed1=meancond(conditionx[0],conditionx[1],z_c);

    initialConditions(conditiony,stepLengthy,stepTime,z_c);
    this->ZMPstructure.zeroSpeedy=meancond(conditiony[0],conditiony[1],z_c);

     this->ZMPstructure.DSPhaseSize=int((stepTime/TsCart+0.5)*DSPhasePercent);
    this->ZMPstructure.SSPhaseSize=int((stepTime/TsCart+0.5)*(1-DSPhasePercent));
    this->ZMPstructure.DSReference=(conditionx[0]*cosh(this->ZMPstructure.DSPhaseSize*TsCart/Tc)+Tc*conditionx[1]*sinh(this->ZMPstructure.DSPhaseSize*TsCart/Tc));
    this->ZMPstructure.DSReference=stepLength/2+this->ZMPstructure.DSReference;
    this->ZMPstructure.footEdgex=footEdgex;
    this->ZMPstructure.desiredXf=conditionx[1];
}
void DynamicWalkClass::initialConditions(double*conditions,double step_length,double TsCarttep,double zc){
    double g=9.81;
    double Tc=sqrt(zc/g);
    double B=exp(-TsCarttep/Tc);
    double a=B-1;
    double b=B*step_length/Tc;
    double c=B*pow(step_length,2)/(4*pow(Tc,2))+g*pow(step_length,2)/(4*zc);

    double xf1=( -b+sqrt( pow(b,2)-4*a*c ) )/( 2*a );
    double xf2=( -b-sqrt( pow(b,2)-4*a*c ) )/( 2*a );
    if (xf1>0){
        conditions[1]=xf1;
    }
    else if (xf2>0){
        conditions[1]=xf2;
    }
    else{
        cout<<"ill condition fot initial conditions"<<endl;
    }
    conditions[0]=-step_length/2;

}
double DynamicWalkClass::meancond(double initialPos,double initialSpeed,double zc){
    double g=9.81;
    double temp=pow(initialSpeed,2)-g*pow(initialPos,2)/(zc);//energy
    return sqrt(temp);//meanSpeed
}
void DynamicWalkClass::ZMP_preview      (double xfinal,double gcomx[3],double gcomy[3],double Z_C){
    //THis funtion provides the response of the preview controller using for it
    //  a fixed set of gains Gd, Gd1, GI according to the procedure in Bram V.
    //  the controller uses the feedback in velocity and position and provides the updated data
    //  for the hip and feet
    // initialize variables

    double z_c=this->ZMPstructure.z_c;
    double ggg=9.81;
    double Tc_MPC=sqrt(z_c/ggg);
    // Gains
    // System description (Cart table model with Zc=0.41)
    double sysdA[3][3]={{1,dynamicWalk::TsCart, pow(dynamicWalk::TsCart,2)/2},{ 0, 1, dynamicWalk::TsCart},{ 0, 0, 1}};
    double sysdB[3]={pow(dynamicWalk::TsCart,3)/6, pow(dynamicWalk::TsCart,2)/2, dynamicWalk::TsCart};
    double sysdC[3]={1, 0, -z_c/ggg};


    // simulated feedback
    x_cart[0]=x_active[0];
    x_cart[1]=x_active[1];
    x_cart[2]=x_active[2];
    y_cart[0]=y_active[0];
    y_cart[1]=y_active[1];
    y_cart[2]=y_active[2];

    //real feedbacks
    //x_cart[0]=gcomx[0]; // position
    //x_cart[1]=gcomx[1];     // velocity
    //x_cart[2]=gcomx[2];     // acceleration

    // y_cart[0]=gcomy[0];
     //y_cart[1]=gcomy[1];
     //y_cart[2]=gcomy[2];

    // updates
    this->ZMPstructure.actualPos     =(x_cart[0]);
    this->ZMPstructure.actualPosY    =(y_cart[0]);
    this->ZMPstructure.actualSpeed   =(gcomx[1]);
    this->ZMPstructure.actualSpeedy  =(gcomy[1]);

     //ZMP generation
    zmpTrajectory(Tc_MPC,xfinal);
    //feet trajectory generation
    if(this->ZMPstructure.walkingcase!=5 && this->ZMPstructure.stop!=-1)
        feetUpdate();

    std::vector<double> zmpx1;
    std::vector<double> zmpy1;
    for (int i=0;i<N2;i++){
        zmpx1.push_back(this->ZMPstructure.zmpx[i]);
        zmpy1.push_back(this->ZMPstructure.zmpy[i]);
    }
    double uysat=controlP.previewControl(gcomy,y_cart,sysdA,sysdB,sysdC,zmpy1,this->ZMPstructure.footEdgeY,TsCart,this->ZMPstructure.z_c);
    for(int k=0; k<3; k++){
        double xTemp=0;
        for(int i=0; i<3; i++){
            xTemp+=sysdA[k][i] * y_cart[i];
        }
        y_active[k] = xTemp+ sysdB[k] * uysat;
    }

    double uxsat=controlP.previewControl(gcomx,x_cart,sysdA,sysdB,sysdC,zmpx1,this->ZMPstructure.footEdgex,TsCart,this->ZMPstructure.z_c);
    for(int k=0; k<3; k++){
        double xTemp=0;
        for(int i=0; i<3; i++){
            xTemp+=sysdA[k][i] * x_cart[i];
        }
        x_active[k] = xTemp+ sysdB[k] * uxsat;
    }

}
void DynamicWalkClass::ZMP_MPC(double xfinal,double gcomx[3],double gcomy[3],double Z_C){
    //THis funtion provides the response of the preview controller using for it
    //  a fixed set of gains Gd, Gd1, GI according to the procedure in Bram V.
    //  the controller uses the feedback in velocity and position and provides the updated data
    //  for the hip and feet
    // initialize variables


    double z_c=0.61;//this->ZMPstructure.z_c;

    double ezmpy=0;
    double ggg=9.81;
    double Tc_MPC=sqrt(z_c/ggg);

    // System description (Cart table model with Zc=0.41)
    double Wt[N2];
    double Wty[N2];
    double sysdA[3][3]={{1,dynamicWalk::TsCart, pow(dynamicWalk::TsCart,2)/2},{ 0, 1, dynamicWalk::TsCart},{ 0, 0, 1}};
    double sysdB[3]={pow(dynamicWalk::TsCart,3)/6, pow(dynamicWalk::TsCart,2)/2, dynamicWalk::TsCart};
    double sysdC[3]={1, 0, -z_c/ggg};
    double zmpx2=0;
    double zmpy2=0;
    // simulated feedback
    x_cart[0]=x_active[0];
    x_cart[1]=x_active[1];
    x_cart[2]=x_active[2];
    y_cart[0]=y_active[0];
    y_cart[1]=y_active[1];
    y_cart[2]=y_active[2];

    //real feedbacks
    //feet trajectory generation
    //x_cart[0]=gcomx[0]; // position
    x_cart[1]=gcomx[1];     // velocity
    //x_cart[2]=gcomx[2];     // acceleration

//     y_cart[0]=gcomy[0];
//     y_cart[1]=gcomy[1];
//     y_cart[2]=gcomy[2];

    // updates
    this->ZMPstructure.actualPos     =(x_cart[0]);
    this->ZMPstructure.actualPosY    =(y_cart[0]);
    this->ZMPstructure.actualSpeed   =(gcomx[1]);
    this->ZMPstructure.actualSpeedy  =(gcomy[1]);




      //ZMP generation
    zmpTrajectory(Tc_MPC,xfinal);
    //feet trajectory generation
    if(this->ZMPstructure.walkingcase!=5 && this->ZMPstructure.stop!=-1)
        feetUpdate();

//    if(this->ZMPstructure.stop==-1){
//        this->ZMPstructure.LFZ=0;
//        this->ZMPstructure.RFZ=0;
//    }



//    for(int k=0;k<N2; k++){
//       Wty[k]=this->ZMPstructure.zmpy[k];
//   }
//   double  Yty=0;
//   for(int k=0;k<3; k++){
//       Yty+=sysdC[k]*y_cart[k];
//   }

//   controlMY.MPC( Yty, Wty);
//   double uy=controlMY.U[N2];
//   double Py[3]={0,0,0};
//   for(int k=0; k<3; k++){
//       double yTemp=0;
//       for(int i=0; i<3; i++){
//           yTemp+=sysdA[k][i] * gcomy[i];
//       }
//       Py[k] = yTemp + sysdB[k] * uy;
//       }
//   Yty=0;
//   for(int k=0;k<3; k++){
//       Yty+=sysdC[k]*Py[k];
//   }
//   Py[2]=controlMY.saturate(Yty,this->ZMPstructure.zmpy[0],y_cart[0],z_c,this->ZMPstructure.footEdgeY);
//   double uysat=(Py[2]-y_cart[2])/TsCart;
//   for(int k=0; k<3; k++){
//       double yTemp=0;
//       for(int i=0; i<3; i++){
//           yTemp+=sysdA[k][i] * y_cart[i];
//       }
//       y_active[k] = yTemp+ sysdB[k] * uysat;
//   }
//   controlMY.U[N2]=uysat;


    std::vector<double> zmpy1;
    for (int i=0;i<N2;i++){
        zmpy1.push_back(this->ZMPstructure.zmpy[i]);
    }
    double uysat=controlP.previewControl(gcomy,y_cart,sysdA,sysdB,sysdC,zmpy1,this->ZMPstructure.footEdgeY,TsCart,this->ZMPstructure.z_c);
    for(int k=0; k<3; k++){
        double xTemp=0;
        for(int i=0; i<3; i++){
            xTemp+=sysdA[k][i] * y_cart[i];
        }
        y_active[k] = xTemp+ sysdB[k] * uysat;
    }

     for(int k=0;k<N2; k++){
        Wt[k]=this->ZMPstructure.zmpx[k];
    }
    double  Yt=0;
    for(int k=0;k<3; k++){
        Yt+=sysdC[k]*x_cart[k];
    }

    controlM.MPC( Yt, Wt);
    double u=controlM.U[N2];
    double Px[3]={0,0,0};
    for(int k=0; k<3; k++){
        double xTemp=0;
        for(int i=0; i<3; i++){
            xTemp+=sysdA[k][i] * gcomx[i];
        }
        Px[k] = xTemp + sysdB[k] * u;
        }
    Yt=0;
    for(int k=0;k<3; k++){
        Yt+=sysdC[k]*Px[k];
    }
    Px[2]=controlM.saturate(Yt,this->ZMPstructure.zmpx[0],x_cart[0],z_c,this->ZMPstructure.footEdgex);
    double uxsat=(Px[2]-x_cart[2])/TsCart;
    for(int k=0; k<3; k++){
        double xTemp=0;
        for(int i=0; i<3; i++){
            xTemp+=sysdA[k][i] * x_cart[i];
        }
        x_active[k] = xTemp+ sysdB[k] * uxsat*1.2;
    }
    controlM.U[N2]=uxsat*1.2;



    zmpy2=0;
    ezmpy = 0;


}
double DynamicWalkClass::Poly3(double s0,double t0,double sf,double tf,double time){
    if (tf==t0)
        return 0;
    double a0=s0;
    double a2=3/pow((tf-t0),2)*(sf-s0);
    double a3=-2/pow((tf-t0),3)*(sf-s0);
    double t=time-t0;
    if (time<t0 ){
        return s0;
    }
    else if (time>tf){
        return sf;
    }
    else{
        return a0+a2*pow(t,2)+a3*pow(t,3);
    }
}
double DynamicWalkClass::swingLeg(double swingFoot,double expectedPos,double remainTime,double T){
    expectedPos=expectedPos;
    double k=0;
    double positionSwing=0;
    if (remainTime>T){
        positionSwing=((expectedPos-swingFoot)*T/(remainTime));
    }
    else{
        positionSwing=0; //expectedPos-swingFoot;
    }
    return swingFoot+positionSwing;

}
double DynamicWalkClass::zfunct(double footref,double footPosition,double xdes,double actualPosition,double clearance) {
    //second order function taking as reference point for the hyperbola the starting point
    //of the swinging, the mid stance position, and the desire landing position in the saggital
    //plane if feet are symetric when the swing phas start the the mid stance foot as reference for
    //the fucntion is shift (improve in the future)
    double zmax=clearance;
    double SF=(footPosition-footref);
    xdes=xdes-footref;
    double X=actualPosition-footref;
    double pos=0;
    if (xdes==0){
         return 0;
    }
    else if (SF==0 || SF==xdes){
        SF=xdes/2;
        double temp=(SF*((xdes)-SF));
        if (temp==0)
            return 0;
        pos=-zmax/temp*(pow(X,2)-(xdes)*X);
    }
    else if(X==xdes){
        pos=0;
    }
    else {
        double temp=(SF*((xdes)-SF));
        if(temp==0)
            return 0;
        pos=-zmax/temp*(pow(X,2)-(xdes)*X);
    }
    if (pos<0){
        pos=0;
    }

    return pos;
}
double DynamicWalkClass::yfunct(double ydes,double remainTime,double legPos,double T){
    //First order function that depends on the saggital remaining time
    double pos;
    if (remainTime>T){
        pos=(ydes-legPos)*T/remainTime;
    }
    else{
        pos=0;
    }
    return legPos+pos;
}
void DynamicWalkClass::footChange(){
    //This function change the stand foot according to the conditions
    // if a change is allow and if the step is finish

    if (this->ZMPstructure.SStime>=0.01 && this->ZMPstructure.goRight==0 && this->ZMPstructure.footChange==1 || ((this->ZMPstructure.walkingcase==2 || this->ZMPstructure.walkingcase==4) && this->ZMPstructure.goRight==0 && this->ZMPstructure.footChange==1)){
        //stand foot take the left foot y position
        this->ZMPstructure.footy=this->ZMPstructure.LFY;
        this->ZMPstructure.yTemp=this->ZMPstructure.footy-this->ZMPstructure.zmpyref;
        this->ZMPstructure.goRight=1;
        this->ZMPstructure.goLeft=0;
        this->ZMPstructure.footChange=0;
        this->ZMPstructure.rightFoot=this->ZMPstructure.RFX;
        if (this->ZMPstructure.xfinal<this->ZMPstructure.footPosition)
            this->ZMPstructure.xfinal=this->ZMPstructure.footPosition;
        cout<<this->steptime*TsCart<<endl;
        this->steptime=0;

    }

    else if (this->ZMPstructure.SStime>=0.01 && this->ZMPstructure.goLeft==0  && this->ZMPstructure.footChange==1 || ((this->ZMPstructure.walkingcase==2 || this->ZMPstructure.walkingcase==4) && this->ZMPstructure.goLeft==0 && this->ZMPstructure.footChange==1)){
       //stand foot take the right foot y position
        this->ZMPstructure.footy=this->ZMPstructure.RFY;
        this->ZMPstructure.yTemp=this->ZMPstructure.footy+this->ZMPstructure.zmpyref;
        this->ZMPstructure.goLeft=1;
        this->ZMPstructure.goRight=0;
        this->ZMPstructure.footChange=0;
        this->ZMPstructure.leftFoot=this->ZMPstructure.LFX;
        if (this->ZMPstructure.xfinal<this->ZMPstructure.footPosition)
            this->ZMPstructure.xfinal=this->ZMPstructure.footPosition;
        cout<<this->steptime*TsCart<<endl;
        this->steptime=0;

    }
}
void DynamicWalkClass::footReference(){
    //Update reference at the new step
    if  (this->ZMPstructure.goLeft){
        /*this->ZMPstructure.LFY=LF[1];
        this->ZMPstructure.RFY=RF[1];*/
        this->ZMPstructure.footy=this->ZMPstructure.LFY;
        this->ZMPstructure.yTemp=this->ZMPstructure.footy-this->ZMPstructure.zmpyref;
        this->ZMPstructure.footPosition=this->ZMPstructure.LFX;
        this->ZMPstructure.footChange=1;
        this->ZMPstructure.position=this->ZMPstructure.desirexf;
        this->ZMPstructure.stepCount=this->ZMPstructure.stepCount+1;
        relativeflag=0;
    }
    else if  (this->ZMPstructure.goRight){
        /*this->ZMPstructure.RFY=RF[1];
        this->ZMPstructure.LFY=LF[1];*/
        this->ZMPstructure.footy=this->ZMPstructure.RFY;
        this->ZMPstructure.yTemp=this->ZMPstructure.footy+this->ZMPstructure.zmpyref;
        this->ZMPstructure.footPosition=this->ZMPstructure.RFX;
        this->ZMPstructure.footChange=1;
        this->ZMPstructure.position=this->ZMPstructure.desirexf;
        this->ZMPstructure.stepCount=this->ZMPstructure.stepCount+1;
        relativeflag=1;
    }
}
void DynamicWalkClass::feetUpdate(){
    //This function provides the feet update function for the swing foot
    footChange();
    if  (this->ZMPstructure.goLeft && this->ZMPstructure.footChange==0){
        //this->ZMPstructure.LFY=LF[1];

        this->ZMPstructure.LFX=(swingLeg(this->ZMPstructure.LFX,this->ZMPstructure.footPosition+this->ZMPstructure.desirexf+this->ZMPstructure.position,this->ZMPstructure.time,TsCart));
        this->ZMPstructure.LFZ=(zfunct(this->ZMPstructure.leftFoot,this->ZMPstructure.footPosition,this->ZMPstructure.footPosition+this->ZMPstructure.desirexf+this->ZMPstructure.position,this->ZMPstructure.LFX,this->ZMPstructure.clearance));
        this->ZMPstructure.LFY=(yfunct(this->ZMPstructure.positiony,this->ZMPstructure.time,this->ZMPstructure.LFY,TsCart));

        this->ZMPstructure.goRight=0;

    }
    else if (this->ZMPstructure.goRight && this->ZMPstructure.footChange==0){
        //this->ZMPstructure.RFY=RF[1];
        this->ZMPstructure.RFX=(swingLeg(this->ZMPstructure.RFX,this->ZMPstructure.footPosition+this->ZMPstructure.desirexf+this->ZMPstructure.position,this->ZMPstructure.time,TsCart));
        this->ZMPstructure.RFZ=(zfunct(this->ZMPstructure.rightFoot,this->ZMPstructure.footPosition,this->ZMPstructure.footPosition+this->ZMPstructure.desirexf+this->ZMPstructure.position,this->ZMPstructure.RFX,this->ZMPstructure.clearance));
        this->ZMPstructure.RFY=(yfunct(this->ZMPstructure.positiony,this->ZMPstructure.time,this->ZMPstructure.RFY,TsCart));

        this->ZMPstructure.goLeft=0;

    }
}
void DynamicWalkClass::zmpTrajectory(double Tc,double xfinal){
    //double this->ZMPstructure.DSRemainTimey=0;
    double SSRemainTimey=0;
    if (this->ZMPstructure.stepCount<1){
        this->ZMPstructure.offset=this->ZMPstructure.stepLength/2;}
    else{
        this->ZMPstructure.offset=this->ZMPstructure.stepLength/2;
    }
    //maximum step length
    double limit=1;
    //default state
    int walkingcase=5;
    //initial conditions
    double finalSpeed2=0;
    this->ZMPstructure.DSRemainTime=0;
    double finalSpeed=0;
    double positiony=0;
    std:vector<double> tempZMP(zmpref2.preview_window);
    //compute the saggital local distace of the COM respect to the stance foot
    double Xposition=(this->ZMPstructure.actualPos-this->ZMPstructure.footPosition);
//    if (relativeflag){
//        Xposition=relativepos[0];
//    }
//        else if(!relativeflag){
//        Xposition=relativepos[1];
//    }
    bool reactive=(pow(this->ZMPstructure.actualSpeed,2)>pow(this->ZMPstructure.footEdgex,2)*9.81/this->ZMPstructure.z_c) || (this->ZMPstructure.actualPos>this->ZMPstructure.footPosition+this->ZMPstructure.footEdgex*2);


    //if moving forward
    if  ((((Xposition<=0.0 && this->ZMPstructure.actualSpeed>((Xposition))/Tc) || ((Xposition)>0.0) ))&& this->ZMPstructure.actualSpeed>0.005 && (this->ZMPstructure.stop==3 || this->ZMPstructure.stop==0 || this->ZMPstructure.stop==2)  && this->ZMPstructure.stepCount>0
         || ( ( (this->ZMPstructure.stop==0 ) && reactive && this->ZMPstructure.xfinal<=this->ZMPstructure.zmpx[0]) || (this->ZMPstructure.stop==3 || this->ZMPstructure.stop==2)) || (this->ZMPstructure.walkingcase==1 && Xposition>-0.05 && !this->ZMPstructure.stop==1)
         ){
            walkingcase=1;
            if ((this->ZMPstructure.stop==0 || this->ZMPstructure.stop==-1 ) && reactive && this->ZMPstructure.xfinal<=this->ZMPstructure.zmpx[0]){
                this->ZMPstructure.stop=3;
                this->ZMPstructure.stepCount=1;
                if (this->ZMPstructure.goLeft==0 && this->ZMPstructure.goRight==0)
                    this->ZMPstructure.goLeft=1;
                    this->ZMPstructure.footy=this->ZMPstructure.RFY;
                this->ZMPstructure.footChange=0;
            }
      }

    else if ((this->ZMPstructure.xfinal>this->ZMPstructure.zmpx[0] && this->ZMPstructure.actualSpeed>=-0.1) &&this->ZMPstructure.stop!=-1 || this->ZMPstructure.stop==1){
           walkingcase=2;
   }
   else{
            walkingcase=5;
   }
   if (this->ZMPstructure.xfinal<xfinal){
       if (this->ZMPstructure.startwalk==0){
           for (int i=0;i<zmpref2.matrix_size;i++){
               for (int j=0;j<N2;j++){
                    zmpref2.zmpMx[i][j]=this->ZMPstructure.footPosition;
               }
           }
           this->ZMPstructure.stop=0;
             this->ZMPstructure.stepCount=0;
            this->ZMPstructure.goLeft=0;
            this->ZMPstructure.goRight=0;
            this->ZMPstructure.footChange=1;

            for (int i=0;i<zmpref2.matrix_size;i++){
                   for (int j=0;j<N2;j++){
                       if (j<3*this->ZMPstructure.SSPhaseSize){
                           zmpref2.zmpMy[i][j]=Poly3(this->ZMPstructure.actualPosY,0,this->ZMPstructure.footy,(3*this->ZMPstructure.SSPhaseSize-this->ZMPstructure.DSPhaseSize)*TsCart,j*TsCart);
                       }
                       else{
                           zmpref2.zmpMy[i][j]=this->ZMPstructure.footy;
                       }
                   }
               }
               SSRemainTimey=0;
               this->ZMPstructure.DSRemainTimey=0.5;
               double positiony2=halfHip/2;

               tempZMP=zmpref2.ZmpUpdatey(this->ZMPstructure.actualPosY*0,SSRemainTimey,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSRemainTimey,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLengthy,this->ZMPstructure.yTemp,positiony2);
               for ( int i=0;i<this->ZMPstructure.zmpLengthy;i++){
                   this->ZMPstructure.zmpy[i]=tempZMP[i];
               }
        }
      if (this->ZMPstructure.startwalk==(this->ZMPstructure.SSPhaseSize-10)){
          this->ZMPstructure.xfinal=xfinal;
           this->ZMPstructure.startwalk=0;
       }
       else{
           this->ZMPstructure.startwalk=this->ZMPstructure.startwalk+1;}

   }

    switch (walkingcase){
        case 1:
            //if the system was waking back a change of references is requiered due to the direction change
            if (this->ZMPstructure.goLeft==1 && this->ZMPstructure.desirexf<0){
                this->ZMPstructure.leftFoot=this->ZMPstructure.footPosition-this->ZMPstructure.position;
            }
            else if(this->ZMPstructure.goRight==1 && this->ZMPstructure.desirexf<0){
                this->ZMPstructure.rightFoot=this->ZMPstructure.footPosition-this->ZMPstructure.position;
            }

            //if the trajectory generation is already over the xfinal do a smaller step and stop the
            //dynamic of the motion (desired final position:stance foot, desire final speed:0)
            if (this->ZMPstructure.stop==3 || this->ZMPstructure.stop==2){
                this->ZMPstructure.desirexf=this->ZMPstructure.offset;
                this->ZMPstructure.zeroSpeed=0;
                this->ZMPstructure.stepLength1=0;

                this->ZMPstructure.finalSpeed=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,this->ZMPstructure.desirexf,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                if(this->ZMPstructure.time>0.1)
                    this->ZMPstructure.position=pendulumModel.newPos(this->ZMPstructure.zeroSpeed,this->ZMPstructure.finalSpeed,this->ZMPstructure.z_c);
                finalSpeed2=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.z_c);
                this->ZMPstructure.SStime=pendulumModel.arrivalTime(this->ZMPstructure.SStime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,finalSpeed2,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                this->ZMPstructure.stop=2;


            }
            //else use normal walking conditions
            else{
                this->ZMPstructure.desirexf=this->ZMPstructure.offset;
                this->ZMPstructure.zeroSpeed=this->ZMPstructure.zeroSpeed1;
                this->ZMPstructure.stepLength1=this->ZMPstructure.stepLength;

                this->ZMPstructure.finalSpeed=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                finalSpeed2=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.z_c);

                this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,this->ZMPstructure.desirexf,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                if(this->ZMPstructure.time>0.1)
                    this->ZMPstructure.position=pendulumModel.newPos(this->ZMPstructure.zeroSpeed,this->ZMPstructure.finalSpeed,this->ZMPstructure.z_c);
                this->ZMPstructure.SStime=pendulumModel.arrivalTime(this->ZMPstructure.SStime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);

            }

            if (this->ZMPstructure.position>limit){
                this->ZMPstructure.position=limit;
            }
            if (this->ZMPstructure.time>0){
                this->ZMPstructure.SSRemainTime=pendulumModel.arrivalTime(this->ZMPstructure.SSRemainTime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference*0,finalSpeed2,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                if (this->ZMPstructure.SSRemainTime<=0){
                    this->ZMPstructure.SSRemainTime=0;
                    this->ZMPstructure.DSRemainTime=this->ZMPstructure.time;
                }
                else if(this->ZMPstructure.DSRemainTime<=0){
                    this->ZMPstructure.DSRemainTime=0;
                }
            }
            else{
                this->ZMPstructure.DSRemainTime=0;
                this->ZMPstructure.SSRemainTime=0;
            }
            //generate ZMP reference
            tempZMP=zmpref2.ZmpUpdatex(this->ZMPstructure.footPosition,this->ZMPstructure.SSRemainTime,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLength1,this->ZMPstructure.xfinal,this->ZMPstructure.desirexf+this->ZMPstructure.position);
            for (int i=0;i<(this->ZMPstructure.zmpLength);i++){
                this->ZMPstructure.zmpx[i]=tempZMP[i];}

            //check if the step was complete based on the COM position
            if (Xposition>=this->ZMPstructure.desirexf && this->ZMPstructure.whichfoot==2){
                if(this->ZMPstructure.xfinal<this->ZMPstructure.zmpx[0] && this->ZMPstructure.stop!=2){
                    this->ZMPstructure.stop=3;}
                footReference();
                if (this->ZMPstructure.stop==2){

                     this->ZMPstructure.stop=1;

                     //this->ZMPstructure.SStime=0;
                }
            }
            this->steptime++;
            break;
        case 2:

             if (this->ZMPstructure.goLeft==1 && this->ZMPstructure.desirexf<0){
                this->ZMPstructure.leftFoot=this->ZMPstructure.footPosition+this->ZMPstructure.desirexf-this->ZMPstructure.position;
            }
            else if(this->ZMPstructure.goRight==1 && this->ZMPstructure.desirexf<0){
            this->ZMPstructure.rightFoot=this->ZMPstructure.footPosition+this->ZMPstructure.desirexf-this->ZMPstructure.position;
            }
            if (this->ZMPstructure.stop==1 ){
                    cout<<"Finish"<<endl;
                    this->ZMPstructure.desirexf=0;
                    this->ZMPstructure.zeroSpeed=0;
                    this->ZMPstructure.stepLength1=0;

                    this->ZMPstructure.finalSpeed=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                    this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,this->ZMPstructure.desirexf,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                    //if(this->ZMPstructure.time>0.1)
                        this->ZMPstructure.position=0;//pendulumModel.newPos(this->ZMPstructure.zeroSpeed,this->ZMPstructure.finalSpeed,this->ZMPstructure.z_c);
                    finalSpeed2=0;//pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                    //this->ZMPstructure.SStime=pendulumModel.arrivalTime(this->ZMPstructure.SStime,this->ZMPstructure.desirexf,finalSpeed2,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                    if (this->ZMPstructure.SStime<=0){

                        this->ZMPstructure.time=this->ZMPstructure.SSPhaseSize/2*TsCart;
                        this->ZMPstructure.SStime=this->ZMPstructure.SSPhaseSize/2*TsCart;
                        this->ZMPstructure.DSRemainTime=0;
                        this->ZMPstructure.SSRemainTime=this->ZMPstructure.SStime;
                    }
                    else{
                        this->ZMPstructure.time=this->ZMPstructure.SStime-TsCart;
                        this->ZMPstructure.SStime=this->ZMPstructure.SStime-TsCart;
                        this->ZMPstructure.DSRemainTime=0;
                        this->ZMPstructure.SSRemainTime=this->ZMPstructure.SSRemainTime-TsCart;

                    }
            }
            else{

                this->ZMPstructure.desirexf=this->ZMPstructure.offset;
                this->ZMPstructure.zeroSpeed=this->ZMPstructure.zeroSpeed1;
                this->ZMPstructure.stepLength1=this->ZMPstructure.stepLength;

                this->ZMPstructure.finalSpeed=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                finalSpeed2=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.z_c);

                this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,this->ZMPstructure.desirexf,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                if(this->ZMPstructure.time>0.1)
                    this->ZMPstructure.position=pendulumModel.newPos(this->ZMPstructure.zeroSpeed,this->ZMPstructure.finalSpeed,this->ZMPstructure.z_c);

                this->ZMPstructure.SStime=pendulumModel.arrivalTime(this->ZMPstructure.SStime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.finalSpeed,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);

                if (this->ZMPstructure.time>0){
                    this->ZMPstructure.SSRemainTime=pendulumModel.arrivalTime(this->ZMPstructure.SSRemainTime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,finalSpeed2,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);

                    //saturate time to the SSPhasesize
                    if  (this->ZMPstructure.SSRemainTime>this->ZMPstructure.SSPhaseSize*TsCart){
                        this->ZMPstructure.SSRemainTime=this->ZMPstructure.SSPhaseSize*TsCart;
                    }
                    //if SS time == 0 then the remaining time corresponds to the DS
                    if (this->ZMPstructure.SSRemainTime<=0){
                        this->ZMPstructure.SSRemainTime=0;
                        this->ZMPstructure.DSRemainTime=this->ZMPstructure.time;
                    }
                    else if(this->ZMPstructure.DSRemainTime<=0){
                        this->ZMPstructure.DSRemainTime=0;
                    }

                }
                else{
                    this->ZMPstructure.DSRemainTime=0;
                    this->ZMPstructure.SSRemainTime=this->ZMPstructure.SSPhaseSize*TsCart;

                }
            }
            //generate ZMP reference
            tempZMP=zmpref2.ZmpUpdatex(this->ZMPstructure.footPosition,this->ZMPstructure.SSRemainTime,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLength1,this->ZMPstructure.xfinal,this->ZMPstructure.desirexf+this->ZMPstructure.position+0.005);
            for (int i=0;i<(this->ZMPstructure.zmpLength);i++){
                this->ZMPstructure.zmpx[i]=tempZMP[i];
            }
            if (this->ZMPstructure.actualPos>=(this->ZMPstructure.footPosition) && this->ZMPstructure.stop==1){// && (this->ZMPstructure.LFZ<=0 && this->ZMPstructure.RFZ<=0)){
                //footReference(zmpStructure,LF,RF);
                //this->initStructure(&this->ZMPstructure,gcomx,gcomy,this->ZMPstructure.zmpyref);
                this->ZMPstructure.stop=-1;

            }
            if (Xposition>=this->ZMPstructure.desirexf && this->ZMPstructure.whichfoot==2){
                if (this->ZMPstructure.actualPos>=(this->ZMPstructure.footPosition) && this->ZMPstructure.stop==1){// && (this->ZMPstructure.LFZ<=0 && this->ZMPstructure.RFZ<=0)){
                    //footReference(zmpStructure,LF,RF);
                    //this->initStructure(&this->ZMPstructure,gcomx,gcomy,this->ZMPstructure.zmpyref);
                    this->ZMPstructure.stop=-1;

                }
                footReference();
            }
            break;

        case 5:
            if (this->ZMPstructure.stop!=-1){
                this->ZMPstructure.zeroSpeed=0;

                this->ZMPstructure.finalSpeed=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,0,this->ZMPstructure.z_c);
                this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,0,this->ZMPstructure.finalSpeed,(Xposition),this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                this->ZMPstructure.position=0;//newPos(this->ZMPstructure.zeroSpeed,finalSpeed,this->ZMPstructure.z_c);
                this->ZMPstructure.stepLength=0;
                finalSpeed2=pendulumModel.finalVel(Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,this->ZMPstructure.z_c);
                if (this->ZMPstructure.LFZ==0 && this->ZMPstructure.RFZ==0){
                   this->ZMPstructure.SStime=pendulumModel.arrivalTime(this->ZMPstructure.SStime,this->ZMPstructure.desirexf-this->ZMPstructure.DSReference,finalSpeed2,Xposition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                }
                else{
                    this->ZMPstructure.SStime=this->ZMPstructure.time;
                }
                if  (this->ZMPstructure.SSRemainTime>this->ZMPstructure.SSPhaseSize*TsCart || (this->ZMPstructure.SSRemainTime==0 && this->ZMPstructure.DSRemainTime==0)){
                    this->ZMPstructure.SSRemainTime=this->ZMPstructure.SSPhaseSize*TsCart;
                }
                if (this->ZMPstructure.SSRemainTime<=0){
                    this->ZMPstructure.DSRemainTime=this->ZMPstructure.time;
                }
                else{
                    this->ZMPstructure.DSRemainTime=0;
                }
                tempZMP=zmpref2.ZmpUpdatex(this->ZMPstructure.footPosition,this->ZMPstructure.SSRemainTime,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLength1,this->ZMPstructure.xfinal,this->ZMPstructure.desirexf+this->ZMPstructure.position);

            }
            else {
                   this->ZMPstructure.desirexf=0;
                    this->ZMPstructure.zeroSpeed=0;
                    this->ZMPstructure.stepLength1=0;
                    this->ZMPstructure.finalSpeed=pendulumModel.finalVel(this->ZMPstructure.actualPos-this->ZMPstructure.footPosition,this->ZMPstructure.actualSpeed,this->ZMPstructure.desirexf,this->ZMPstructure.z_c);
                    this->ZMPstructure.time=pendulumModel.arrivalTime(this->ZMPstructure.time,this->ZMPstructure.desirexf,this->ZMPstructure.finalSpeed,this->ZMPstructure.actualPos-this->ZMPstructure.footPosition,this->ZMPstructure.actualSpeed,this->ZMPstructure.z_c);
                    this->ZMPstructure.position=0;//newPos(0,finalSpeed,this->ZMPstructure.z_c);
                    this->ZMPstructure.SStime=this->ZMPstructure.time;
                    this->ZMPstructure.DSRemainTime=0;
                    this->ZMPstructure.SSRemainTime=0.01;
                    tempZMP=zmpref2.ZmpUpdatex(this->ZMPstructure.footPosition,this->ZMPstructure.SSRemainTime,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLength1,this->ZMPstructure.footPosition,Xposition);
//                    if(!reactive && this->ZMPstructure.actualPos<(this->ZMPstructure.footPosition+0.03) && this->ZMPstructure.actualSpeed<0.05){
//                       this->ZMPstructure.stop=0;
//                       this->ZMPstructure.stepCount=0;}

            }
             for (int i=0;i<this->ZMPstructure.zmpLength;i++){
                this->ZMPstructure.zmpx[i]=tempZMP[i];
            }
            break;

    }
    this->ZMPstructure.walkingcase=walkingcase;
   // this->ZMPstructure.yTemp=0;

         double speedinY=0;
        double limity=0.3;
        double positiony2=0,landingY=0;
        this->ZMPstructure.stepLengthy= this->ZMPstructure.zmpyref*2;
        double posLeft=0, posRight=0;

        double leftSpeed=0, rightSpeed=0;
        double landLeft=0, landRight=0;
        double posLeft2=0, posRight2=0;
        //calculate the time in saggital plane for the n+1 step
        double positiony3;
        double tao=Filtertime.applyFilter(pendulumModel.timeTo(-this->ZMPstructure.position,this->ZMPstructure.finalSpeed,0,this->ZMPstructure.zeroSpeed, this->ZMPstructure.z_c))+steptime/2;
        if (tao<0.6){
            tao=0.6;

        }
        double footy1=this->ZMPstructure.footy;
        if (this->ZMPstructure.goLeft && this->ZMPstructure.footChange==0){
            if (this->ZMPstructure.SSRemainTime==Tprev){
            SSRemainTimey=0;
                if (this->ZMPstructure.DSRemainTimey<=0){
                    this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSPhaseSize*TsCart;
                }
                else {
                    this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSRemainTimey-TsCart;
                }
            }
            else{
                this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSRemainTime;
                SSRemainTimey=this->ZMPstructure.SSRemainTime;
            }
            //evaluate speed of COMy in frontal plane given the timing in saggital plane for the landing
            speedinY=pendulumModel.newVelY((this->ZMPstructure.actualPosY-this->ZMPstructure.footy),this->ZMPstructure.actualSpeedy,SSRemainTimey,this->ZMPstructure.z_c);
            leftSpeed=FilterLd.applyFilter(speedinY);

            //evaluate final COMy position in frontal plane given the timing in saggital plane for the landing
            landingY=FilterlandyL.applyFilter(pendulumModel.newPosY(this->ZMPstructure.actualPosY-this->ZMPstructure.footy,this->ZMPstructure.actualSpeedy,SSRemainTimey,this->ZMPstructure.z_c));
            // calculate the foot landing position respect to the stand foot

            landRight=FilterlandyR.applyFilter(-HALF_HIP_WIDTH/2);
            posRight2=FilterLandR.applyFilter(-HALF_HIP_WIDTH/2);
            FilterR.applyFilter(-2*this->ZMPstructure.zmpyref/2);

            if(this->ZMPstructure.time>0.05 && this->ZMPstructure.time<0.7){
                landLeft=(landingY);//FilterL.applyFilter(pendulumModel.landingPos(this->ZMPstructure.actualPosY-this->ZMPstructure.footy,this->ZMPstructure.actualSpeedy,speedinY,this->ZMPstructure.z_c));
                posLeft2=FilterLandL.applyFilter(-pendulumModel.finalPosY(leftSpeed,tao));
                positiony=FilterL.applyFilter(posLeft2+landLeft);
            }
            else if(this->ZMPstructure.time>0.7){

                landLeft=FilterlandyL.applyFilter(HALF_HIP_WIDTH/2);
                posLeft2=FilterLandL.applyFilter(HALF_HIP_WIDTH/2);
                FilterL.applyFilter(2*this->ZMPstructure.zmpyref/2);
            }
            else{
                landLeft=FilterlandyL.y[0];
                posLeft2=FilterLandL.y[0];
                positiony=FilterL.y[0];
            }


            if ((walkingcase==2 && this->ZMPstructure.LFX<this->ZMPstructure.footPosition+this->ZMPstructure.footEdgex) || this->ZMPstructure.walkingcase==5 || this->ZMPstructure.stop==1){
                positiony=this->ZMPstructure.footy+2*this->ZMPstructure.zmpyref;
                positiony2=this->ZMPstructure.footy;
            }
            else{

                positiony=posLeft2+landLeft;
                if (positiony>limity){
                    positiony=this->ZMPstructure.footy+limity;
                    positiony2=positiony;
                }
               else  if (positiony<=4*this->ZMPstructure.zmpyref/2){
                    positiony=this->ZMPstructure.footy+4*this->ZMPstructure.zmpyref/2;
                    positiony2=this->ZMPstructure.footy+4*this->ZMPstructure.zmpyref/2;
                }
                else{
                    positiony=  this->ZMPstructure.footy+positiony;
                    positiony2= positiony;
                    }
                }
            positiony3=(positiony2);
//            positiony2=this->ZMPstructure.yTemp+this->ZMPstructure.zmpyref;
//            positiony=positiony2;
        }
        else if (this->ZMPstructure.goRight && this->ZMPstructure.footChange==0){
            if (this->ZMPstructure.SSRemainTime==Tprev){
                SSRemainTimey=0;
                if (this->ZMPstructure.DSRemainTimey<=0){
                    this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSPhaseSize*TsCart;
                }
                else {
                    this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSRemainTimey-TsCart;
                }
            }
            else{
                this->ZMPstructure.DSRemainTimey=this->ZMPstructure.DSRemainTime;
                SSRemainTimey=this->ZMPstructure.SSRemainTime;
            }
            speedinY=pendulumModel.newVelY(this->ZMPstructure.actualPosY-this->ZMPstructure.footy,this->ZMPstructure.actualSpeedy,SSRemainTimey,this->ZMPstructure.z_c);
            rightSpeed=FilterRd.applyFilter(speedinY);
            landingY=FilterlandyR.applyFilter(pendulumModel.newPosY(this->ZMPstructure.actualPosY-this->ZMPstructure.footy,this->ZMPstructure.actualSpeedy,SSRemainTimey,this->ZMPstructure.z_c));


            landLeft=FilterlandyL.applyFilter(HALF_HIP_WIDTH/2);
            posLeft2=FilterLandL.applyFilter(HALF_HIP_WIDTH/2);
            FilterL.applyFilter(2*this->ZMPstructure.zmpyref/2);

            if(this->ZMPstructure.SStime>0.05 && this->ZMPstructure.time<0.7){
                landRight=(landingY);//-FilterR.applyFilter(pendulumModel.landingPos(this->ZMPstructure.actualPosY-this->ZMPstructure.footy,this->ZMPstructure.actualSpeedy,rightSpeed,this->ZMPstructure.z_c));
                posRight2=FilterLandR.applyFilter(-pendulumModel.finalPosY(rightSpeed,tao));
                positiony=FilterR.applyFilter(posRight2+landRight);

            }
            else if(this->ZMPstructure.time>0.7){
                landRight=FilterlandyR.applyFilter(-HALF_HIP_WIDTH/2);
                posRight2=FilterLandR.applyFilter(-HALF_HIP_WIDTH/2);
                FilterR.applyFilter(-2*this->ZMPstructure.zmpyref/2);
            }
            else{
                landRight=FilterlandyR.y[0];
                posRight2=FilterLandR.y[0];
                positiony=FilterR.y[0];
            }


            if ((walkingcase==2 && this->ZMPstructure.RFX<this->ZMPstructure.footPosition+this->ZMPstructure.footEdgex ) || this->ZMPstructure.walkingcase==5 || this->ZMPstructure.stop==1){
                positiony=this->ZMPstructure.footy-2*this->ZMPstructure.zmpyref;
                positiony2=-this->ZMPstructure.footy;
            }
            else{
                positiony=posRight2+landRight;
                if (positiony<-limity){
                    positiony=this->ZMPstructure.footy-limity;
                    positiony2=positiony;
                }
                else if (positiony>=-4*this->ZMPstructure.zmpyref/2){
                    positiony=this->ZMPstructure.footy-4*this->ZMPstructure.zmpyref/2;
                    positiony2=this->ZMPstructure.footy-4*this->ZMPstructure.zmpyref/2;
                }
                else{
                    positiony= this->ZMPstructure.footy+positiony;
                    positiony2=  positiony;
                }
            }
            positiony3=(positiony2);
//            positiony2=this->ZMPstructure.yTemp-this->ZMPstructure.zmpyref;
//            positiony=positiony2;
        }

        else{

            this->ZMPstructure.DSRemainTimey=0;
            SSRemainTimey=0;
            positiony=this->ZMPstructure.positiony;
            positiony2=positiony;
            positiony3=(positiony2);
        }
        if (this->ZMPstructure.stop==-1){
            tempZMP=zmpref2.ZmpUpdatey(this->ZMPstructure.yTemp,SSRemainTimey,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSRemainTimey,this->ZMPstructure.DSPhaseSize,TsCart,0,this->ZMPstructure.yTemp,this->ZMPstructure.yTemp);
            for (int i=0;i<zmpref2.matrix_size;i++){
              for (int j=0;j<N2;j++){
                   zmpref2.zmpMy[i][j]=this->ZMPstructure.yTemp;
               }
            }


        }
        else if (this->ZMPstructure.stop==1){
          for (int i=0;i<zmpref2.matrix_size;i++){
            for (int j=0;j<N2;j++){
                 zmpref2.zmpMy[i][j]=this->ZMPstructure.yTemp;
             }
          }
          tempZMP=zmpref2.ZmpUpdatey(footy1,SSRemainTimey,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSRemainTimey,this->ZMPstructure.DSPhaseSize,TsCart,0,this->ZMPstructure.yTemp,this->ZMPstructure.yTemp);
       }
        else{
                tempZMP=zmpref2.ZmpUpdatey(footy1,SSRemainTimey,this->ZMPstructure.SSPhaseSize,this->ZMPstructure.DSRemainTimey,this->ZMPstructure.DSPhaseSize,TsCart,this->ZMPstructure.stepLengthy,this->ZMPstructure.yTemp,positiony2);
    }
    for ( int i=0;i<this->ZMPstructure.zmpLengthy;i++){
        this->ZMPstructure.zmpy[i]=tempZMP[i];
    }
    in.open("updatedpos.txt",ios::app);
        in<<positiony3<<"   "<<posLeft2+landLeft<<"    "<<posRight2+landRight<<"\n";;
    in.close();
    this->ZMPstructure.positiony=positiony;
}
