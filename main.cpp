#include <iostream>
#include "WBC_srb.h"
#include "quill/Quill.h"
#include "FileOperator.h"
#include "Pinocchio_Utilities.h"
#include "LPFilter_ava.h"
FileOperator fileRW("../stateData_QP.txt");

Pinocchio_Utilities pinLib("../BonnieURDF_latest.urdf");

int main()
{
//    Eigen::Quaternion<double> quat;
//    Eigen::Matrix<double,3,3> Rtmp;
//    Rtmp<<0,0.707107,0.707107,
//          0,0.707107,-0.707107,
//          -1,0,0;
//    quat=Rtmp;
//
//    std::cout<<quat.x()<<std::endl;
//    std::cout<<quat.y()<<std::endl;
//    std::cout<<quat.z()<<std::endl;
//    std::cout<<quat.w()<<std::endl;
//    std::cout<<Rtmp.determinant()<<std::endl;

    std::vector<double> tmpValue;
    std::string tmpStr;
    // Get a pointer to the default logger
    quill::Handler *file_handler = quill::file_handler("../OutputData.txt", "w");
    file_handler->set_pattern(QUILL_STRING("%(message)"));
    quill::Logger *dl = quill::create_logger("logger", file_handler);
    quill::start();

    WBC_srb wbc_Controller;
    Eigen::Matrix<double,3,3> Ig;
    Ig<<0.523524,3.19194e-5,-0.0279089,
        3.19194e-5,0.393347,-0.000262413,
        -0.0279089,-0.000262413,0.21856;
    Ig=Ig/13*14;

    double xCoM[3],vCoM[3],pe[6],eul[3],omegaL[3],legInd[2],xd[3],Euld[3],dx_d[3], w_d[3];
    double legIndPhase[2];
    double qr[5],ql[5],qPas_r[2],qPas_l[2];
    double xCoMOff[3];

    xCoM[0]=0;xCoM[0]=0;xCoM[2]=0.5;
    vCoM[0]=0;vCoM[1]=0;vCoM[2]=0;
    pe[0]=0;pe[1]=-0.0823955;pe[2]=0;pe[3]=0;pe[4]=0.0824522;pe[5]=0;
    eul[0]=-0.1/3.1415*180;eul[1]=0;eul[2]=0;
    omegaL[0]=0;omegaL[1]=0;omegaL[2]=0;
    legInd[0]=1;legInd[1]=0;
    xd[0]=0;xd[1]=0;xd[2]=0.6;
    Euld[0]=0;Euld[1]=0;Euld[2]=0;
    dx_d[0]=0;dx_d[1]=0;dx_d[2]=0;
    w_d[0]=0;w_d[1]=0;w_d[2]=0;
    legIndPhase[0]=1;legIndPhase[1]=1;
    double yaw0;

    wbc_Controller.QP_S<<1,2,10,450,450,100; // 1,2,10,450,450,100;
    wbc_Controller.QP_Wp<<10,10,10,10,10,10,10,10;
    wbc_Controller.QP_Wc<<3,3,3,3,3,3,3,3;
    wbc_Controller.QP_alpha=0.1;
    wbc_Controller.QP_beta=0.01;
    wbc_Controller.K_xp<<75,75,40;
    wbc_Controller.K_xd<<10,10,5;
    wbc_Controller.K_wp<<300,300,0;
    wbc_Controller.K_wd<<30,30,0;

    wbc_Controller.setModelPara(14,Ig,0.5);

    LPFilter_ava wx,wy,wz,eulx,euly,eulz;

    for (int i = 0; i < fileRW.getTotalLine(); i++) {
        fileRW.getNewLine();
        fileRW.getNumsInLine();
        xCoM[0] = fileRW.values[0];
        xCoM[1] = fileRW.values[1];
        xCoM[2] = fileRW.values[2];
        vCoM[0] = fileRW.values[3];
        vCoM[1] = fileRW.values[4];
        vCoM[2] = fileRW.values[5];
        pe[0] = fileRW.values[6];
        pe[1] = fileRW.values[7];
        pe[2] = fileRW.values[8];
        pe[3] = fileRW.values[9];
        pe[4] = fileRW.values[10];
        pe[5] = fileRW.values[11];
        legInd[0]=fileRW.values[12];
        legInd[1]=fileRW.values[13];
        legIndPhase[0]=fileRW.values[14];
        legIndPhase[1]=fileRW.values[15];
        eul[0]=fileRW.values[16];
        eul[1]=fileRW.values[17];
        eul[2]=fileRW.values[18];
        omegaL[0]=fileRW.values[19];
        omegaL[1]=fileRW.values[20];
        omegaL[2]=fileRW.values[21];
        qr[0]=fileRW.values[22];
        qr[1]=fileRW.values[23];
        qr[2]=fileRW.values[24];
        qr[3]=fileRW.values[25];
        qr[4]=fileRW.values[26];
        qPas_r[0]=fileRW.values[27];
        qPas_r[1]=fileRW.values[28];
        ql[0]=fileRW.values[29];
        ql[1]=fileRW.values[30];
        ql[2]=fileRW.values[31];
        ql[3]=fileRW.values[32];
        ql[4]=fileRW.values[33];
        qPas_l[0]=fileRW.values[34];
        qPas_l[1]=fileRW.values[35];
        yaw0=fileRW.values[36];

        eul[2]=eul[2]-yaw0;

        omegaL[0]=wx.run(omegaL[0]);
        omegaL[1]=wy.run(omegaL[1]);
        omegaL[2]=wz.run(omegaL[2]);
        eul[0]=eulx.run(eul[0]);
        eul[1]=euly.run(eul[1]);
        eul[2]=eulz.run(eul[2]);

        wbc_Controller.set_state(xCoM, vCoM, pe, eul, omegaL, false);
        wbc_Controller.setLegState(legIndPhase);
        wbc_Controller.get_ddX_ddw(xd, dx_d, Euld, w_d);
        if (i>312)
            wbc_Controller.runQP(true);

        pinLib.setJointAngle(qr,ql,qPas_r,qPas_l);
//        pinLib.computeJac();

        pinLib.computeJac_float(eul);
        xCoMOff[0]=pinLib.pCoM(0);
        xCoMOff[1]=pinLib.pCoM(1);
        xCoMOff[2]=pinLib.pCoM(2);

        Eigen::Matrix<double,4,1> WrenchR, WrenchL;
        WrenchR<<-wbc_Controller.uOut.block<4,1>(0,0);
        WrenchL<<-wbc_Controller.uOut.block<4,1>(4,0);

        Eigen::Matrix<double,5,1> tauR,tauL;
        tauR=pinLib.J_R_float.transpose()*WrenchR;
        tauL=pinLib.J_L_float.transpose()*WrenchL;

        Eigen::Matrix<double,5,1> IcmdR,IcmdL;
        IcmdR<<pinLib.M10015_T2I(tauR(0)),pinLib.M8016_T2I(tauR(1)),pinLib.M8016_T2I(tauR(2)),pinLib.M10015_T2I(tauR(3)),0;
        IcmdL<<pinLib.M10015_T2I(tauL(0)),pinLib.M8016_T2I(tauL(1)),pinLib.M8016_T2I(tauL(2)),pinLib.M10015_T2I(tauL(3)),0;

        // output data
        tmpValue.clear();

        tmpValue.push_back(wbc_Controller.uOut(0));
        tmpValue.push_back(wbc_Controller.uOut(1));
        tmpValue.push_back(wbc_Controller.uOut(2));
        tmpValue.push_back(wbc_Controller.uOut(3));
        tmpValue.push_back(wbc_Controller.uOut(4));
        tmpValue.push_back(wbc_Controller.uOut(5));
        tmpValue.push_back(wbc_Controller.uOut(6));
        tmpValue.push_back(wbc_Controller.uOut(7));
        tmpValue.push_back(legInd[0]);
        tmpValue.push_back(legInd[1]);
        tmpValue.push_back(wbc_Controller.last_nWSR);
        tmpValue.push_back(wbc_Controller.last_cpuTime);
        tmpValue.push_back(tauR(0));
        tmpValue.push_back(tauR(1));
        tmpValue.push_back(tauR(2));
        tmpValue.push_back(tauR(3));
        tmpValue.push_back(tauR(4));
        tmpValue.push_back(tauL(0));
        tmpValue.push_back(tauL(1));
        tmpValue.push_back(tauL(2));
        tmpValue.push_back(tauL(3));
        tmpValue.push_back(tauL(4));
        tmpValue.push_back(IcmdR(0));
        tmpValue.push_back(IcmdR(1));
        tmpValue.push_back(IcmdR(2));
        tmpValue.push_back(IcmdR(3));
        tmpValue.push_back(IcmdR(4));
        tmpValue.push_back(IcmdL(0));
        tmpValue.push_back(IcmdL(1));
        tmpValue.push_back(IcmdL(2));
        tmpValue.push_back(IcmdL(3));
        tmpValue.push_back(IcmdL(4));
        tmpValue.push_back(xCoMOff[0]);
        tmpValue.push_back(xCoMOff[1]);
        tmpValue.push_back(xCoMOff[2]);
        tmpValue.push_back(wbc_Controller.ddx_d(0));
        tmpValue.push_back(wbc_Controller.ddx_d(1));
        tmpValue.push_back(wbc_Controller.ddx_d(2));
        tmpValue.push_back(wbc_Controller.ddx_d_qpRes(0));
        tmpValue.push_back(wbc_Controller.ddx_d_qpRes(1));
        tmpValue.push_back(wbc_Controller.ddx_d_qpRes(2));
        tmpValue.push_back(wbc_Controller.ddw_d(0));
        tmpValue.push_back(wbc_Controller.ddw_d(1));
        tmpValue.push_back(wbc_Controller.ddw_d(2));
        tmpValue.push_back(wbc_Controller.ddw_d_qpRes(0));
        tmpValue.push_back(wbc_Controller.ddw_d_qpRes(1));
        tmpValue.push_back(wbc_Controller.ddw_d_qpRes(2));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(0));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(1));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(2));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(3));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(4));
        tmpValue.push_back(wbc_Controller.pe_Body_Old(5));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(0));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(1));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(2));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(3));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(4));
        tmpValue.push_back(wbc_Controller.pe_Body_delta(5));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(0));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(1));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(2));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(3));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(4));
        tmpValue.push_back(wbc_Controller.pe_Body_Accumu(5));
        tmpValue.push_back(omegaL[0]);
        tmpValue.push_back(omegaL[1]);
        tmpValue.push_back(omegaL[2]);
        tmpValue.push_back(eul[0]);
        tmpValue.push_back(eul[1]);
        tmpValue.push_back(eul[2]);

        tmpStr = fmt::format("{:.5f}", fmt::join(tmpValue, " "));
        LOG_INFO(dl, "{}", tmpStr);
    }
    std::cout<<wbc_Controller.ddx_d.transpose()<<std::endl;
    std::cout<<wbc_Controller.model_A<<std::endl;
    std::cout<<wbc_Controller.model_bd<<std::endl;
    std::cout<<wbc_Controller.uNow.transpose()<<std::endl;

    return 0;
}
