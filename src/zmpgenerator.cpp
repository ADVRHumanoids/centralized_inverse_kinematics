/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Juan Alecandro Castano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "zmpgenerator.h"
/**
 * @brief zmpgenerator::zmpgenerator
 * default constructur, initialize the system variable
 * in preview windows =400
 * and the matrix for the filter contains 2 rows
 */
zmpgenerator::zmpgenerator(){
    this->matrix_size=2;
    this->preview_window=400;
    this->zmpMx.resize(this->matrix_size);
    for (int i=0;i<this->matrix_size;i++)
        this->zmpMx[i].resize(this->preview_window);
    this->zmpMy.resize(this->matrix_size);
    for (int i=0;i<this->matrix_size;i++)
        this->zmpMy[i].resize(this->preview_window);

}
/**
 * @brief zmpgenerator::zmpgenerator
 * constructur, initialize the system variable using the
 * input info
 * @param window  preview window (default 400)
 * @param mSize   number of rows in the filter matrix (default 2)
 */
zmpgenerator::zmpgenerator(int window, int mSize){
    this->matrix_size=window;
    this->preview_window=mSize;
    this->zmpMx.resize(this->matrix_size);
    for (int i=0;i<this->matrix_size;i++)
        this->zmpMx[i].resize(this->preview_window);
    this->zmpMy.resize(this->matrix_size);
    for (int i=0;i<this->matrix_size;i++)
        this->zmpMy[i].resize(this->preview_window);

}
zmpgenerator::~zmpgenerator(){}
/**
 * @brief zmpgenerator::Poly3
 * third order polinomium
 * @param s0 initial position
 * @param t0 initial time
 * @param sf final position
 * @param tf final time
 * @param time present time
 * @return polinomium value at present time
 */
double zmpgenerator::Poly3(double s0,double t0,double sf,double tf,double time){
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
/**
 * @brief zmpgenerator::FilterZmp
 * This function applies a simple avarege filter to the ZMP reference
 * usingthe information contain in the Matrices of the system
 * @param wichmatrix internal variable, Does not affect the user, informs if it is using
 *        the zmp in x axis or y axis
 * @param zmp vector of 1xpreview_window size to be feel in the process
 * @return the updated ZMP reference in a vector type
 */
std::vector<double> zmpgenerator::FilterZmp(std::string wichmatrix,std::vector<double> zmp){

    std::vector<double> zmpreturn(preview_window);
    if (wichmatrix=="xmatrix"){

        for (int i=0;i<matrix_size-1;i++){
            zmpMx[i]=zmpMx[i+1];
        }

        for (int j=0;j<preview_window;j++){
                this->zmpMx[matrix_size-1]=zmp;
            }
        for (int j=0;j<preview_window;j++){
            double temp1=0;
            for (int i=0; i<matrix_size;i++){
                temp1+=this->zmpMx[i][j];
            }
            this->zmpMx[(matrix_size-1)][j]=temp1/matrix_size;
        }
        zmpreturn=this->zmpMx[matrix_size-1];
    }
    else if(wichmatrix=="ymatrix"){
        for (int i=0;i<matrix_size-1;i++){
            this->zmpMy[i]=this->zmpMy[(i+1)];

        }
        this->zmpMy[(matrix_size-1)]=zmp;
        for (int j=0;j<preview_window;j++){
            double temp1=0;
            for (int i=0; i<matrix_size;i++){
                temp1+=this->zmpMy[i][j];
            }
            this->zmpMy[(matrix_size-1)][j]=temp1/matrix_size;
        }
        zmpreturn=this->zmpMy[matrix_size-1];
     }
    return zmpreturn;
}
/**
 * @brief zmpgenerator::addOffset This function allows to add an offset
 *          To a vector
 * @param vector vector to consider
 * @param offset offset to be use
 * @return  The vector with the desired offset
 */
std::vector<double> zmpgenerator::addOffset(std::vector<double> vector, double offset){
    std::vector<double> newvector;
    for(int i=0;i<vector.size();i++){
        newvector.push_back(vector[i]+offset);
    }
    return newvector;


}
/**
 * @brief zmpgenerator::ZmpUpdatex
 *                      Generates the zmp trajectory for the  x axis
 * @param footPosition  actual support foot's position
 * @param SSRemainTime  Remaining time of the single support phase
 * @param SSPhaseSize   single support phase nominal size
 * @param DSPhaseSize   double support phase nominal size
 * @param T             sample time
 * @param stepLength    lenght of the zmp
 *                      (probably remove in a future version)
 * @param xfinal        final expected position of the complete walk
 * @param position      expected landing position for the swing foot
 * @return              Vector containing the updated ZMP reference
 */
std::vector<double> zmpgenerator::ZmpUpdatex(double footPosition,double SSRemainTime,int SSPhaseSize,
            int DSPhaseSize,double T,double stepLength,double xfinal,double position){

    double stepLength2=0;
    zmp.clear();//zmp.resize(this->preview_window,0);


    std::vector<double> softDS(DSPhaseSize,0);
    std::vector<double> DS1(DSPhaseSize,0);
    std::vector<double> temporalvector2;
    std::vector<double> softzmp2(SSPhaseSize,0);
    std::vector<double> zmp2;

    if      (footPosition>xfinal){       stepLength2=-fabs(stepLength);}
    else if (footPosition<xfinal){  stepLength2=fabs(stepLength);}
    else{                           stepLength2=fabs(stepLength);}

    for (int i=0;i<DSPhaseSize; i++){
        softDS[i]=Poly3(0,0,stepLength2,DSPhaseSize*T,i*T);
        DS1[i]=Poly3(0,0,position,DSPhaseSize*T,i*T);
    }

    softzmp2.insert(softzmp2.end(),softDS.begin(),softDS.end());

    while(softzmp2.size()<this->preview_window){
        temporalvector2=this->addOffset(softzmp2,softzmp2[softzmp2.size()-1]);
        softzmp2.insert(softzmp2.end(),temporalvector2.begin(),temporalvector2.end());
    }
    softzmp2.resize(this->preview_window);

    if (SSRemainTime>0){
        int Nsamples=int(SSRemainTime/T+0.5);
        if (Nsamples<=preview_window){
             zmp2.resize(Nsamples,footPosition);
             if (Nsamples+DSPhaseSize<=preview_window){
                 temporalvector2=addOffset(DS1,footPosition);
                 zmp2.insert(zmp2.end(),temporalvector2.begin(),temporalvector2.end());
                 softzmp2.resize(this->preview_window-Nsamples-DSPhaseSize);
                 temporalvector2=addOffset(softzmp2,zmp2[zmp2.size()-1]);
                 zmp2.insert(zmp2.end(),temporalvector2.begin(),temporalvector2.end());
            }
            else {
                zmp2.resize(this->preview_window);
                for (int i=Nsamples;i<preview_window;i++){
                    zmp2[i]=softDS[i-Nsamples]+zmp2[Nsamples-1];
                }
            }
        }
        else {
            zmp2.resize(this->preview_window,footPosition);
        }
    }
    else{
        zmp2.resize(this->preview_window,0);
        for (int i=0; i<preview_window-1; i++){
            zmp2[i]=this->zmpMx[this->matrix_size-1][i+1];
        }
        zmp2[preview_window-1]=zmp2[this->preview_window-2];
    }

    return this->FilterZmp("xmatrix",zmp2);


}
/**
 * @brief zmpgenerator::ZmpUpdatey
 *                      Generates the zmp trajectory for the  x axis
 * @param footPosition  actual support foot's position
 * @param SSRemainTime  Remaining time of the single support phase
 * @param SSPhaseSize   single support phase nominal size
 * @param DSRemainTime  Remaining time of the Double support phase
 * @param DSPhaseSize   double support phase nominal size
 * @param T             sample time
 * @param stepLength    lenght of the zmp
 *                      (probably remove in a future version)
 * @param ydes          Position 0 on the y axis
 * @param position      Landing position for the swing foot in the y axis
 * @return
 */
std::vector<double> zmpgenerator::ZmpUpdatey(double footPosition,double SSRemainTime,int SSPhaseSize,double DSRemainTime,int DSPhaseSize,
            double T,double stepLength,double ydes,double position){

    int mSize=matrix_size;

    float stepLength2=0;

    std::vector<double> softzmpNeg;
    std::vector<double> softzmp2;
    std::vector<double> softzmp3;
    std::vector<double> zmp2;
    std::vector<double> temporalvector2;

    std::vector<double> softDS(DSPhaseSize,0);
    std::vector<double> softDS2(DSPhaseSize,0);
        std::vector<double> softDS1(DSPhaseSize);
    int sizeDS1=DSPhaseSize;

    std::vector<double> temp;

    if (footPosition<ydes ){
        stepLength2=stepLength;//0.0726*2
    }
    else if (footPosition>ydes ){
        stepLength2=-stepLength;//0.0726*2
    }
    else{
        stepLength2=0;
    }

    for (int i=0;i<DSPhaseSize;i++){
        softDS2[i]=Poly3(0,0,-stepLength2,DSPhaseSize*T,i*T);
    }
    for (int i=0;i<DSPhaseSize;i++){
        softDS1[i]=Poly3(footPosition,0,position,DSPhaseSize*T,i*T);
    }


    softzmp2.resize(SSPhaseSize,0);
    softzmp2.insert(softzmp2.end(),softDS2.begin(),softDS2.end());

    softzmp3=softzmp2;
    for(int i=0;i<softzmp2.size();i++)
        softzmpNeg.push_back(-softzmp2[i]);

    int m=1;
    while(softzmp3.size()<this->preview_window){
        if (m%2==0){
            temporalvector2=this->addOffset(softzmp2,softzmp3[softzmp3.size()-1]);
            softzmp3.insert(softzmp3.end(),temporalvector2.begin(),temporalvector2.end());
        }
        else{
            temporalvector2=this->addOffset(softzmpNeg,softzmp3[softzmp3.size()-1]);
            softzmp3.insert(softzmp3.end(),temporalvector2.begin(),temporalvector2.end());
        }
        m=m+1;
}
    if (SSRemainTime>0){
        int Nsamples=int(SSRemainTime/T+0.5);
        if (Nsamples<=preview_window){

                zmp2.resize(Nsamples,footPosition);
                zmp2.insert(zmp2.end(),softDS1.begin(),softDS1.end());
                temporalvector2=addOffset(softzmp3,zmp2[zmp2.size()-1]);
                zmp2.insert(zmp2.end(),temporalvector2.begin(),temporalvector2.end());
                zmp2.resize(this->preview_window);
        }
        else{ zmp2.resize(Nsamples,footPosition);}
    }
    else if (DSRemainTime>0){
        int Nsamples=(DSRemainTime/T+0.5);

        double auxtime=DSPhaseSize*T;
        if ((auxtime-DSRemainTime)<0){

            temp=softDS1;
            temp.resize(DSPhaseSize+SSPhaseSize,temp[DSPhaseSize-1]);
            Nsamples=DSPhaseSize;//+SSPhaseSize;
        }
        else{
            temp.resize(Nsamples);
            for (int i=(DSPhaseSize-Nsamples);i<DSPhaseSize;i++){
                  temp[i-(DSPhaseSize-Nsamples)]=Poly3(footPosition,0,position,DSPhaseSize*T,i*T);
            }
            temp.resize(Nsamples+SSPhaseSize,temp[Nsamples-1]);
            Nsamples=Nsamples;//+SSPhaseSize;
        }
        if (Nsamples<=preview_window){
            for (int i=0;i<Nsamples;i++){ zmp2.push_back(temp[i]);}

            temporalvector2=addOffset(softzmp3,zmp2[zmp2.size()-1]);
            temporalvector2.resize(this->preview_window-zmp2.size());
            zmp2.insert(zmp2.end(),temporalvector2.begin(),temporalvector2.end());

        }
        else{
            for (int i=0;i<preview_window;i++){
                zmp2.push_back(temp[i]);
            }
        }
    }
    else{
        for (int i=0; i<preview_window-1; i++){
            zmp2.push_back(this->zmpMy[mSize-1][i+1]);
        }
        zmp2[preview_window-1]=zmp2[preview_window-2];
    }
    zmp2.resize(this->preview_window);
    return this->FilterZmp("ymatrix",zmp2);
}
