#include <iostream>
#include "WBC_srb.h"
#include "quill/Quill.h"
#include "FileOperator.h"

FileOperator fileRW("../StateData.txt");

int main()
{
    std::vector<double> tmpValue;
    std::string tmpStr;
    // Get a pointer to the default logger
    quill::Handler *file_handler = quill::file_handler("../OutputData.txt", "w");
    file_handler->set_pattern(QUILL_STRING("%(message)"));
    quill::Logger *dl = quill::create_logger("logger", file_handler);
    quill::start();

    WBC_srb wbc_Controller;
    Eigen::Matrix<double,3,3> Ig;
    Ig<<0.334168,-9.95633e-5,-0.0238023,
        -9.95633e-5,0.225661,-0.000164969,
        -0.0238023,-0.000164969,0.13379;
    double xCoM[3],vCoM[3],pe[6],eul[3],omegaW[3],legInd[2],xd[3],Euld[3],dx_d[3], w_d[3];
    double legIndPhase[2];
    xCoM[0]=0;xCoM[0]=0;xCoM[2]=0.5;
    vCoM[0]=0;vCoM[1]=0;vCoM[2]=0;
    pe[0]=0;pe[1]=-0.0823955;pe[2]=0;pe[3]=0;pe[4]=0.0824522;pe[5]=0;
    eul[0]=-0.1/3.1415*180;eul[1]=0;eul[2]=0;
    omegaW[0]=0;omegaW[1]=0;omegaW[2]=0;
    legInd[0]=1;legInd[1]=0;
    xd[0]=0;xd[1]=0;xd[2]=0.6;
    Euld[0]=0;Euld[1]=0;Euld[2]=0;
    dx_d[0]=0;dx_d[1]=0;dx_d[2]=0;
    w_d[0]=0;w_d[1]=0;w_d[2]=0;
    legIndPhase[0]=1;legIndPhase[1]=1;

    wbc_Controller.QP_S<<20,20,50,450,450,450;
    wbc_Controller.QP_Wp<<10,10,4,10,10,10,4,10;
    wbc_Controller.QP_Wc<<3,3,3,3,3,3,3,3;
    wbc_Controller.QP_alpha=0.001;
    wbc_Controller.QP_beta=0.1;
    wbc_Controller.K_xp<<150,150,150;
    wbc_Controller.K_wd<<25,25,25;
    wbc_Controller.K_wp<<200,200,200;
    wbc_Controller.K_wd<<30,30,30;

//    xCoM[0]=0;xCoM[0]=0;xCoM[2]=0.59;
//    wbc_Controller.setModelPara(14,Ig,0.5);
//    wbc_Controller.set_state(xCoM, vCoM, pe, eul, omegaW);
//    wbc_Controller.setLegState(legIndPhase);
//    wbc_Controller.get_ddX_ddw(xd, dx_d, Euld, w_d);
//    wbc_Controller.runQP();
//
//    xCoM[0]=0;xCoM[0]=0;xCoM[2]=0.599;
//    wbc_Controller.set_state(xCoM, vCoM, pe, eul, omegaW);
//    wbc_Controller.setLegState(legIndPhase);
//    wbc_Controller.get_ddX_ddw(xd, dx_d, Euld, w_d);
//    wbc_Controller.runQP();

    int count=0;
    wbc_Controller.setModelPara(14,Ig,0.5);
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
        omegaW[0]=fileRW.values[19];
        omegaW[1]=fileRW.values[20];
        omegaW[2]=fileRW.values[21];

        wbc_Controller.set_state(xCoM, vCoM, pe, eul, omegaW);
        wbc_Controller.setLegState(legIndPhase);
        wbc_Controller.get_ddX_ddw(xd, dx_d, Euld, w_d);
        if (i*0.001>2.1)
            wbc_Controller.runQP();


        // output data
        tmpValue.clear();

        tmpValue.push_back(wbc_Controller.uNow(0));
        tmpValue.push_back(wbc_Controller.uNow(1));
        tmpValue.push_back(wbc_Controller.uNow(2));
        tmpValue.push_back(wbc_Controller.uNow(3));
        tmpValue.push_back(wbc_Controller.uNow(4));
        tmpValue.push_back(wbc_Controller.uNow(5));
        tmpValue.push_back(wbc_Controller.uNow(6));
        tmpValue.push_back(wbc_Controller.uNow(7));
        tmpValue.push_back(legInd[0]);
        tmpValue.push_back(legInd[1]);
        tmpValue.push_back(wbc_Controller.last_nWSR);
        tmpValue.push_back(wbc_Controller.last_cpuTime);

        tmpStr = fmt::format("{:.5f}", fmt::join(tmpValue, " "));
        LOG_INFO(dl, "{}", tmpStr);
    }
//    std::cout<<wbc_Controller.ddx_d.transpose()<<std::endl;
//    std::cout<<wbc_Controller.model_A<<std::endl;
//    std::cout<<wbc_Controller.model_bd<<std::endl;
//    std::cout<<wbc_Controller.uNow.transpose()<<std::endl;

    return 0;
}


//int main( )
//{
//    USING_NAMESPACE_QPOASES
//
//    /* Setup data of first QP. */
//    real_t H[2*2] = {2.0, -2, -2, 4 };
//    real_t A[2*2] = {2, 1.0, 1.0,-4 };
//    real_t g[2] = { -4, 0 };
//    real_t lb[2] = { 0.0, 0.0 };
//    //real_t ub[2] = { 20.0, 20.0 };
//    //real_t lbA[2] = { -100.0 ,-100};
//    real_t ubA[2] = { 6.0 ,0};
//
//    /* Setting up QProblem object. */
//    QProblem example( 2,2 );
//
//    Options options;
//    example.setOptions( options );
//
//    /* Solve first QP. */
//    int_t nWSR = 10;
//    example.init( H,g,A,lb,NULL,NULL,ubA, nWSR);//没有约束，或者约束边界值不存在，则可以使用NULL
//
//    /* Get and print solution of first QP. */
//    real_t xOpt[2];
//    real_t yOpt[2+2];
//    example.getPrimalSolution( xOpt );
//    example.getDualSolution( yOpt );
//    printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e , %e];  objVal = %e\n\n",
//            xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],yOpt[3],example.getObjVal() );
//
//    example.printOptions();
//    example.printProperties();
//    //getGlobalMessageHandler()->listAllMessages();
//    return 0;
//}
