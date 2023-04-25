//
// Created by boxingwang on 23-4-17.
//

#include "WBC_srb.h"
#include <iostream>
#include "iomanip"
using namespace Eigen;

WBC_srb::WBC_srb():wbc_srb_QP(8,12) {
    model_A.setZero();
    model_A.block<3,3>(0,0)=Eigen::Matrix<double,3,3>::Identity();
    model_A.block<3,3>(0,4)=Eigen::Matrix<double,3,3>::Identity();
    model_A.block<3,1>(3,3)<<0,0,1;
    model_A.block<3,1>(3,7)<<0,0,1;

    model_bd.setZero();
    QP_Wc.setOnes();
    QP_Wp.setOnes();
    QP_S.setOnes();
    QP_alpha=0;
    QP_beta=0;

    g_vec<<0,0,-9.8;
    m=14;
    Ig<<1e-3,0,0,
        0,1e-3,0,
        0,0,1e-3;
    M_c.setZero();
    M_c(0,0)=1;
    M_c(1,0)=-1;
    M_c(2,1)=1;
    M_c(3,1)=-1;
    M_c(4,2)=1;
    M_c(5,3)=1;

    for (int i=0;i<12;i++)
    {
        qp_lbA[i]=0;
        qp_ubA[i]=0;
    }
    qp_lbA[4]=20;
    qp_ubA[4]=400;
    qp_lbA[5]=-15;
    qp_ubA[5]=15;

    qp_lbA[10]=20;
    qp_ubA[10]=400;
    qp_lbA[11]=-15;
    qp_ubA[11]=15;

    qp_ubA[0]=qpOASES::INFTY;
    qp_ubA[1]=qpOASES::INFTY;
    qp_ubA[2]=qpOASES::INFTY;
    qp_ubA[3]=qpOASES::INFTY;
    qp_ubA[6]=qpOASES::INFTY;
    qp_ubA[7]=qpOASES::INFTY;
    qp_ubA[8]=qpOASES::INFTY;
    qp_ubA[9]=qpOASES::INFTY;

    uOld.setZero();
    uNow.setZero();

    qpOASES::Options options;
//    options.setToReliable();
    options.printLevel=qpOASES::PL_NONE;
    wbc_srb_QP.setOptions(options);
};

void WBC_srb::set_state(double *xCoM, double *vCoM,double *pe, double *eul, double *omegaW) {
    for (int i=0;i<3;i++)
    {xCoM_cur(i)=xCoM[i]; vCoM_cur(i)=vCoM[i];}
    for (int i=0;i<6;i++)
        pe_cur(i)=pe[i];
    R_cur= Euler2Rot(eul[0],eul[1],eul[2]);
    w_cur<<omegaW[0],omegaW[1],omegaW[2];
}

// right first
void WBC_srb::setLegState(double *legInd) {
    legInStance[0]=legInd[0];
    legInStance[1]=legInd[1];
    qp_lbA[4]=0;
    qp_ubA[4]=0;
    qp_lbA[5]=0;
    qp_ubA[5]=0;

    qp_lbA[10]=0;
    qp_ubA[10]=0;
    qp_lbA[11]=0;
    qp_ubA[11]=0;

    if (legInStance[0]>0.5)
    {
        qp_lbA[4]=20;
        qp_ubA[4]=400;
        qp_lbA[5]=-15;
        qp_ubA[5]=15;
    }

    if (legInStance[1]>0.5)
    {
        qp_lbA[10]=20;
        qp_ubA[10]=400;
        qp_lbA[11]=-15;
        qp_ubA[11]=15;
    }
}

void WBC_srb::setModelPara(double mIn, Eigen::Matrix<double, 3, 3> &Iin,double miuIn) {
    m=mIn;
    Ig=Iin;
    miu=miuIn;
    M_c.block<4,1>(0,2)<<miu,miu,miu,miu;
}

void WBC_srb::get_ddX_ddw(double *xd, double *dx_d, double *Euld, double *w_d) {
    Vector3d xd_vec, dxd_vec;
    xd_vec<<xd[0],xd[1],xd[2];
    dxd_vec<<dx_d[0],dx_d[1],dx_d[2];
    ddx_d=K_xp.asDiagonal()*(xd_vec-xCoM_cur)+K_xd.asDiagonal()*(dxd_vec-vCoM_cur);
    Matrix3d Rd;
    Rd= Euler2Rot(Euld[0],Euld[1],Euld[2]);
    Vector3d theta_delta,wd_vec;
    theta_delta= getWfromR(Rd*R_cur.transpose());
    wd_vec<<w_d[0],w_d[1],w_d[2];
    ddw_d=K_wp.asDiagonal()*theta_delta+K_wd.asDiagonal()*(wd_vec-w_cur);
}

void WBC_srb::runQP() {

    Matrix<double,3,1> tmp;
    model_A.block<3,3>(3,0)= crossMatrix(pe_cur.block<3,1>(0,0)-xCoM_cur);
    model_A.block<3,3>(3,4)= crossMatrix(pe_cur.block<3,1>(3,0)-xCoM_cur);
    model_bd.block<3,1>(0,0)=m*(ddx_d-g_vec);
    model_bd.block<3,1>(3,0)=R_cur*Ig*R_cur.transpose()*ddw_d;

//    std::cout<<model_A<<std::endl;


    Matrix<double,8,8> H_tmp;
    Matrix<double,1,8> g_tmp;

    H_tmp=model_A.transpose()*QP_S.asDiagonal().toDenseMatrix()*model_A+QP_alpha*QP_Wc.asDiagonal().toDenseMatrix()+
            QP_beta*QP_Wp.asDiagonal().toDenseMatrix();
    g_tmp=-model_bd.transpose()*QP_S.asDiagonal().toDenseMatrix()*model_A-
            QP_beta*uOld.transpose()*QP_Wp.asDiagonal().toDenseMatrix();

//    std::cout<<model_A<<std::endl;
//    std::cout<<H_tmp<<std::endl;

    Matrix<double,12,8> A_tmp;
    A_tmp.setZero();
    A_tmp.block<6,4>(0,0)=M_c;
    A_tmp.block<6,4>(6,4)=M_c;


    copy_Eigen_to_real_t(qp_H,H_tmp,8,8);
    copy_Eigen_to_real_t(qp_g,g_tmp,1,8);
    copy_Eigen_to_real_t(qp_A,A_tmp,12,8);

    for(int i=0;i<8;i++)
        xOpt_iniGuess[i]=0;
    if (legInStance[0]>0.5)
        xOpt_iniGuess[2]=140;
    if (legInStance[1]>0.5)
        xOpt_iniGuess[6]=140;
    qpOASES::returnValue res;
//    res=wbc_srb_QP.init(qp_H,qp_g,qp_A,NULL,NULL,qp_lbA,qp_ubA,nWSR,&cpu_time);
    nWSR=100;
    cpu_time=0.001;
    res=wbc_srb_QP.init(qp_H,qp_g,qp_A,NULL,NULL,qp_lbA,qp_ubA,nWSR,&cpu_time,xOpt_iniGuess);
    qpStatus=qpOASES::getSimpleStatus(res);
    last_nWSR=nWSR;
    last_cpuTime=cpu_time;

    //std::cout<<qpStatus<<std::endl;

//    if (res==qpOASES::SUCCESSFUL_RETURN)
//        std::cout<<"successful_return"<<std::endl;
//    else if (res==qpOASES::RET_MAX_NWSR_REACHED)
//        std::cout<<"max_nwsr"<<std::endl;
//    else if (res==qpOASES::RET_INIT_FAILED)
//        std::cout<<"init_failed"<<std::endl;
//    else
//        std::cout<<qpOASES::getSimpleStatus(res)<<std::endl;

    qpOASES::real_t xOpt[8];
    wbc_srb_QP.getPrimalSolution(xOpt);

    uNow<<xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],
            xOpt[5],xOpt[6],xOpt[7];
    uOld=uNow;
    wbc_srb_QP.reset();

    //-------- for debugging --------------
//    Matrix<double,12,1> BoundRes;
//
//    BoundRes=A_tmp*uNow;
//    Matrix<double,6,1> bAct;
//    bAct=model_A*uNow;
//    std::cout<<"bAct ";
//    for (int i=0;i<6;i++)
//    {std::cout.width(10);
//        std::cout<<bAct(i)<<"\t";}
//    std::cout<<std::endl;
//    std::cout<<"bDes ";
//    for (int i=0;i<6;i++)
//    {std::cout.width(10);
//        std::cout<<model_bd(i)<<"\t";}
//    std::cout<<std::endl;
//
//    std::cout<<"uOld ";
//    for (int i=0;i<8;i++)
//    {std::cout.width(10);
//        std::cout<<uOld(i)<<"\t";}
//    std::cout<<std::endl;
//    std::cout<<"uNow ";
//    for (int i=0;i<8;i++)
//    {std::cout.width(10);
//        std::cout<<uNow(i)<<"\t";}
//    std::cout<<std::endl;
//
//    std::cout.precision(3);
//    std::cout.width(10);
//    std::cout<<"lowBound ";
//    for (int i=0;i<12;i++)
//    {std::cout.width(10);
//        std::cout<<qp_lbA[i]<<"\t";}
//    std::cout<<std::endl;
//    std::cout.width(10);
//    std::cout<<"RelBound ";
//    for (int i=0;i<12;i++)
//    {std::cout.width(10);
//        std::cout<<BoundRes(i)<<"\t";}
//    std::cout<<std::endl;
//    std::cout.width(10);
//    std::cout<<"uppBound ";
//    for (int i=0;i<12;i++)
//    {std::cout.width(10);
//        std::cout<<qp_ubA[i]<<"\t";}
//    std::cout<<std::endl;
}

Matrix<double, 3, 1> WBC_srb::getWfromR(Matrix<double, 3, 3> R) {
    Matrix<double,3,1> w;
    double pi=3.1415926;
    w.setZero();
    if (R.isIdentity(1e-5))
    {
        return w;
    }
    else if (R.isDiagonal(1e-5))
    {
        w<<pi/2*(R(0,0)+1),pi/2*(R(1,1)+1),pi/2*(R(2,2)+1);
        return w;
    }
    Vector3d l;
    l<<R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1);
    double theta;
    theta= atan2(l.norm(),R(0,0)+R(1,1)+R(2,2)+1);
    w=theta*l/l.norm();
    return w;
}

Matrix<double, 3, 3> WBC_srb::Euler2Rot(double roll, double pitch, double yaw) {
    Matrix3d Rx, Ry, Rz;
    Rx<< 1,0,0,
            0, cos(roll),-sin(roll),
            0,sin(roll),cos(roll);
    Ry<< cos(pitch),0,sin(pitch),
            0,1,0,
            -sin(pitch),0,cos(pitch);
    Rz<<cos(yaw),-sin(yaw),0,
            sin(yaw),cos(yaw),0,
            0,0,1;
    return Rz*Ry*Rx;
}

void WBC_srb::copy_Eigen_to_real_t(qpOASES::real_t* target, MatrixXd source, int nRows, int nCols) {
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    // real_t is stored by rows, same to C array
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count] = source(i, j);
            count++;
        }
    }
}

Matrix<double,3,3> WBC_srb::crossMatrix(Eigen::Vector3d omega) {
    Matrix<double,3,3> R;
    R.setZero();
    R(0, 1) = -omega(2);
    R(0, 2) = omega(1);
    R(1, 0) = omega(2);
    R(1, 2) = -omega(0);
    R(2, 0) = -omega(1);
    R(2, 1) = omega(0);

    return R;
}