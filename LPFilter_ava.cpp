//
// Created by boxingwang on 23-7-9.
//

#include "LPFilter_ava.h"

double LPFilter_ava::run(double dataIn) {
    dataRec[2]=dataRec[1];
    dataRec[1]=dataRec[0];
    dataRec[0]=dataIn;
    return (dataRec[0]+dataRec[1]+dataRec[2])/3.0 ;
}

void LPFilter_ava::resetData() {
    for (int i=0;i<3;i++)
        dataRec[i]=0;
}
