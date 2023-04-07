#include <iostream>
#include <qpOASES.hpp>

int main( )
{
    USING_NAMESPACE_QPOASES

    /* Setup data of first QP. */
    real_t H[2*2] = {2.0, -2, -2, 4 };
    real_t A[2*2] = {2, 1.0, 1.0,-4 };
    real_t g[2] = { -4, 0 };
    real_t lb[2] = { 0.0, 0.0 };
    //real_t ub[2] = { 20.0, 20.0 };
    //real_t lbA[2] = { -100.0 ,-100};
    real_t ubA[2] = { 6.0 ,0};

    /* Setting up QProblem object. */
    QProblem example( 2,2 );

    Options options;
    example.setOptions( options );

    /* Solve first QP. */
    int_t nWSR = 10;
    example.init( H,g,A,lb,NULL,NULL,ubA, nWSR);//没有约束，或者约束边界值不存在，则可以使用NULL

    /* Get and print solution of first QP. */
    real_t xOpt[2];
    real_t yOpt[2+2];
    example.getPrimalSolution( xOpt );
    example.getDualSolution( yOpt );
    printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e , %e];  objVal = %e\n\n",
            xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],yOpt[3],example.getObjVal() );

    example.printOptions();
    example.printProperties();
    //getGlobalMessageHandler()->listAllMessages();
    return 0;
}
