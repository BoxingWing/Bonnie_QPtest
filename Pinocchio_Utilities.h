//
// Created by boxingwang on 23-3-27.
//

#ifndef BONNIE_JACTEST_PINOCCHIO_UTILITIES_H
#define BONNIE_JACTEST_PINOCCHIO_UTILITIES_H

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/math/quaternion.hpp"
#include "Eigen/Geometry"

class Pinocchio_Utilities {
public:
    pinocchio::Model model_Bonnie_Static, model_Bonnie_Dynamic;
    Eigen::VectorXd q_S, q_D;
    Eigen::VectorXd dq_S, dq_D;
    Eigen::Matrix<double,4,5> J_L,J_R;
    Eigen::Matrix<double,4,5> J_L_float,J_R_float;
    Eigen::Matrix<double,3,3> Ig;
    Eigen::Matrix<double,3,1> pe_L,pe_R;
    Eigen::Matrix<double,20,1> Gq; // generalized gravity term, including fx, fy, fz, taux, tauy, tauz, tauQ1-tauQ14
    Eigen::Matrix<double,14,1> qB_urdf;
    Eigen::Matrix<double,21,1> qB_urdf_float;
    Eigen::Matrix<double,3,1> pCoM;
    Eigen::Matrix<double,3,1> pBaseLink;    // position of the baselink
    Eigen::Matrix<double,4,1> BaseQuat;    // quaternion of the baselink
    double totalMass{0};

    Pinocchio_Utilities(std::string urdfName);

    // NOTE: qr, ql, qPas_r, qPas_l should be direct values get from the robot
    void setJointAngle(double* qr, double *ql, double *qPas_r, double *qPas_l);
    void computeIg();
    void computeJac();
    void computeJac_float(double *eul);
    void computeG();
    double M8016_I2T(double Id);
    double M10015_I2T(double Id);
    double M8016_T2I(double Td);
    double M10015_T2I(double Td);
    double sgn(double in);

    Eigen::Matrix<double,3,3> eul2Rot(double roll, double pitch, double yaw);
};


#endif //BONNIE_JACTEST_PINOCCHIO_UTILITIES_H
