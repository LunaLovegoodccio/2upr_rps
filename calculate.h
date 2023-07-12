#ifndef CALCULATE_H
#define CALCULATE_H

#include "Global.h"

const double ANGLE_TO_STEPS_RATIO = 16;  // 角度到脉冲数的比例  1.8°对应32个脉冲  0.9°对应16个脉冲
const int Legth_TO_STEPS_RATIO = 3200;   // 长度到脉冲数的比例
const double ANGLE_TO_LEGTH = 0.01;  //角度到长度  转180度走过1mm
const double runtime = 300;  //运行1mm需要300ms  v=10mm/s


class Calculate
{
public:
    Calculate();
    ~Calculate();
    QVector<int> calculate_N(double z,double a,double b);
    QVector<int> calculate_Z(double d1,double d2,double d3);
    bool calculateWorkspace();


private:
    int max;
    int m_okCount;   //接收ok的次
    int* Speed;
    int* Direction;
    int* Pulse;
    int* Accelerations = new int[3]{0, 0, 0};
    //Rotation* r;

};

#endif // CALCULATE_H
