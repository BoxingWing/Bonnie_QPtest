//
// Created by boxingwang on 23-7-9.
//

#ifndef BONNIE_QPTEST_LPFILTER_AVA_H
#define BONNIE_QPTEST_LPFILTER_AVA_H


class LPFilter_ava {
private:
    double dataRec[3]={0};
public:
    void resetData();
    double run(double dataIn);

};


#endif //BONNIE_QPTEST_LPFILTER_AVA_H
