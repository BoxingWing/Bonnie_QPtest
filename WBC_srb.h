//
// Created by boxingwang on 23-4-17.
//

#ifndef BONNIE_QPTEST_WBC_SRB_H
#define BONNIE_QPTEST_WBC_SRB_H

#include "qpOASES.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

class WBC_srb {

private:
    const double dt=0.001;
    qpOASES::QProblem wbc_srb_QP;

    Eigen::Matrix<double,3,3> R_cur;
    Eigen::Matrix<double,3,3> R_cur_z;
    Eigen::Vector3d pCoM_cur, vCoM_cur, w_cur;
    Eigen::Matrix<double,6,1> pe_cur;
    double m,miu;
    Eigen::Matrix3d Ig,IgInv;
    Eigen::Vector3d g_vec;
    Eigen::Matrix<double,6,4> M_c;
    const Eigen::Matrix<double,3,1> pCoM_Off_L{0.0455025,0.000187389,-0.0576864};
    Eigen::Matrix<double,3,1> pCoM_Off_W;

    Eigen::Matrix<double,3,1> pCoM_pred;
    Eigen::Matrix<double,6,1> pe_pred;
    Eigen::Matrix<double,3,3> R_pred;

    Eigen::Matrix<double,3,3> RodForm(Eigen::Vector3d omegaIn);

    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //       lb<=x<=ub
    qpOASES::real_t qp_H[8*8];
    qpOASES::real_t qp_A[12*8];
    qpOASES::real_t qp_g[8];
    qpOASES::real_t qp_lbA[12];
    qpOASES::real_t qp_ubA[12];
    qpOASES::int_t nWSR=100;
    qpOASES::real_t cpu_time=0.1;
    qpOASES::real_t xOpt_iniGuess[8];

public:
    WBC_srb();


    double x_d[3],x_cur[3],dx_d[3],dx_cur[3];
    int legInStance[2]; // right first
    Eigen::Matrix<double,3,1> K_xp, K_xd, K_wp, K_wd;
    Eigen::Matrix<double,3,1> ddx_d,ddw_d;
    Eigen::Matrix<double,8,1> uOld, uNow, uOut;
    Eigen::Matrix<double,3,1> ddx_d_qpRes,ddw_d_qpRes;
    Eigen::Matrix<double,6,1> pe_Body_pred, pe_Body_Old, pe_Body_delta, pe_Body_Accumu;
    Eigen::Vector3d xd_vec, dxd_vec;

    Eigen::Matrix<double,6,8> model_A;
    Eigen::Matrix<double,6,1> model_bd;
    Eigen::Matrix<double,6,1> QP_S;
    Eigen::Matrix<double,8,1> QP_Wc, QP_Wp;
    Eigen::Matrix<double,4,8> QP_M_cs, QP_M_cf;
    Eigen::Matrix3d Rd;
    double QP_alpha, QP_beta;

    int qpStatus{0};
    int last_nWSR{0};
    double last_cpuTime{0};

    void setModelPara(double mIn, Eigen::Matrix<double,3,3> &Iin,double miuIn);
    void set_state(double* xCoM, double*  vCoM, double* pe, double* eul, double* omegaW, bool isOmegaW);
    void setLegState(double* legInd); // first one for right leg
    void get_ddX_ddw(double *xd, double *dx_d, double *Euld, double *w_d);
    void runQP(bool EN);
    Eigen::Matrix<double,3,1> getWfromR(Eigen::Matrix<double,3,3> R);
    Eigen::Matrix<double,3,3> Euler2Rot(double roll, double pitch, double yaw);
    void copy_Eigen_to_real_t(qpOASES::real_t* target, Eigen::MatrixXd source, int nRows, int nCols);
    Eigen::Matrix<double,3,3> crossMatrix(Eigen::Vector3d omega);
};

#endif //BONNIE_QPTEST_WBC_SRB_H
