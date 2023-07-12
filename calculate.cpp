#include "calculate.h"
#include <cmath>
#include <vector>
#include <iostream>

Calculate::Calculate()
{
    Pulse = new int[3];
    Speed = new int[3];
    Direction = new int[3];

}
Calculate::~Calculate()
{
    delete[] Pulse;
    delete[] Speed;
    delete[] Direction;
}

QVector<int> Calculate::calculate_Z(double d1,double d2,double d3)
{
    delta_q1 = d1 - prev_q1;
    delta_q2 = d2 - prev_q2;
    delta_q3 = d3 - prev_q3;
    if(delta_q1 > 0) Direction[0]=1; else Direction[0]=0;
    if(delta_q1 > 0) Direction[1]=1; else Direction[1]=0;
    if(delta_q1 > 0) Direction[2]=1; else Direction[2]=0;
    qDebug()<<"delta_q1="<<delta_q1;
    qDebug()<<"delta_q2="<<delta_q2;
    qDebug()<<"delta_q3="<<delta_q3;
    // 根据三个Δq计算步进电机应给的脉冲数
    Pulse[0] = abs(delta_q1*Legth_TO_STEPS_RATIO);   //double  *   int
    Pulse[1] = abs(delta_q2*Legth_TO_STEPS_RATIO);
    Pulse[2] = abs(delta_q3*Legth_TO_STEPS_RATIO);
   // angle1 = step_pulses1/ANGLE_TO_STEPS_RATIO;
   // angle2 = step_pulses2/ANGLE_TO_STEPS_RATIO;
   // angle3 = step_pulses3/ANGLE_TO_STEPS_RATIO;

    // 更新prev_q值
    prev_q1 = d1;
    prev_q2 = d2;
    prev_q3 = d3;

    // 细分为0.9度单位
    max = findMaximum(delta_q1,delta_q2,delta_q3);
    qDebug()<<"max"<<max;
    //double angleMax = static_cast<double>(max * ANGLE_TO_LEGTH);   //电机转角最大
    int minMax = abs(max * runtime);   //时间最大
    qDebug()<<"minMax="<<minMax;

    if(minMax!=0){
        v1=abs(delta_q1)*60000/minMax;
        v2=abs(delta_q2)*60000/minMax;
        v3=abs(delta_q3)*60000/minMax;
    }
    if (v1 != 0) {
        Speed[0] = v1;
    } else {
        Speed[0] = 10;
    }
    if (v2 != 0) {
        Speed[1] = v2;
    } else {
        Speed[1] = 10;
    }
    if (v3 != 0) {
        Speed[2] = v3;
    } else {
        Speed[2] = 10;
    }

    QVector<int> protocolData;

    for (int i = 0; i < 3; i++) {
        int direction = *(Direction + i);
        int speed = *(Speed + i);
        int acceleration = *(Accelerations + i);
        int pulse = *(Pulse + i);

        protocolData.append(direction);
        protocolData.append(speed);
        protocolData.append(acceleration);
        protocolData.append(pulse);
    }
    return protocolData;

}

//b----theta  a---psi
QVector<int> Calculate::calculate_N(double z,double a,double b)
{

    /*
    a = (a * std::acos(-1)) / 180;
    b = (b * std::acos(-1)) / 180;
    q1 = sqrt(pow(z * tan(b) - l_b * sin(b) * sin(a), 2) +
                 pow(z - l_b * sin(a) * cos(b), 2) +
                 pow(l_b * cos(a) - l_a, 2));
    q2 = sqrt(pow(z * tan(b) + l_b * sin(b) * sin(a), 2) +
                 pow(z + l_b * sin(a) * cos(b), 2) +
                 pow(l_b * cos(b) - l_a, 2));
    q3 = sqrt(pow(z * tan(b) + l_b * cos(b) - a, 2) +
                 pow(z - l_b * sin(b),2));
    */

    // 运动学逆解
    double phi = 0;   //绕z轴角度
    double x = z * std::tan(b * con);
    double y = 0;

    // 动平台上各关节坐标  在动坐标系下
    Eigen::Vector3d vb0[3];
    vb0[0] << 0, -array_b[0], 0;
    vb0[1] << 0, array_b[1], 0;
    vb0[2] << array_b[2], 0, 0;

    // 静平台上各关节坐标
    Eigen::Vector3d va[3];
    va[0] << 0, -array_a[0], 0;
    va[1] << 0, array_a[1], 0;
    va[2] << array_a[2], 0, 0;


    //动系-静系的旋转变化矩阵
    Eigen::Matrix3d R = Rotation("z", phi) * Rotation("y", b) * Rotation("x", a);
    //Eigen::Matrix3d R_inv = R.inverse();

    Eigen::Vector3d vb[3];
    for (int k = 0; k < 3; ++k) {
        vb[k] = R * vb0[k];
    }
    Eigen::Vector3d vc = R * Eigen::Vector3d(0, 1, 0);
    Eigen::Vector3d vd(1, 0, 0);
    Eigen::Vector3d vp(x, y, z);


    // 计算杆长 d 和转动副转角 R
    for (int k = 0; k < 2; ++k) {
        double qi = (vp + vb[k] - va[k]).norm();
        Eigen::Vector3d vw = (vp + vb[k] - va[k]) / qi;
        double Ui = std::acos(va[k].dot(vw) / array_a[k]);
        double UUi = std::acos(vd.dot(vw));
        double ri = std::acos(vb[k].dot(vw) / array_b[k]);
        if(k==0)
        {
            q1=qi;
            U1=Ui / con;
            U12=UUi / con;
            R1=ri / con;
        }
        else if(k==1)
        {
            q2=qi;
            U2=Ui / con;
            U22=UUi /con;
            R2=ri / con;
        }
    }
    {
        int k = 2;
        q3 = (vp + vb[k] - va[k]).norm();
        Eigen::Vector3d vw = (vp + vb[k] - va[k]) / q3;
        S3 = (std::acos(vb[k].dot(vw) / array_b[k]))/con;
        S32 = (std::acos(vc.dot(vw))) / con;
        R3 = std::acos(va[k].dot(vw) / array_a[k]) / con;
    }

    qDebug()<<"q1="<<q1;
    qDebug()<<"q2="<<q2;
    qDebug()<<"q3="<<q3;
/*
    qDebug()<<"R1="<<R1;
    qDebug()<<"R2="<<R2;
    qDebug()<<"R3="<<R3;
    qDebug()<<"U1="<<U1;
    qDebug()<<"U2="<<U2;
    qDebug()<<"U12="<<U12;
    qDebug()<<"U22="<<U22;
    qDebug()<<"S3="<<S3;
    qDebug()<<"S32="<<S32;
*/
    delta_q1 = q1 - prev_q1;
    delta_q2 = q2 - prev_q2;
    delta_q3 = q3 - prev_q3;
    if(delta_q1 > 0) Direction[0]=1; else Direction[0]=0;
    if(delta_q2 > 0) Direction[1]=1; else Direction[1]=0;
    if(delta_q3 > 0) Direction[2]=1; else Direction[2]=0;
    /*
    qDebug()<<"delta_q1="<<delta_q1;
    qDebug()<<"delta_q2="<<delta_q2;
    qDebug()<<"delta_q3="<<delta_q3;
    qDebug()<<"Direction="<<Direction[0]<<Direction[1]<<Direction[2];
    */
    // 根据三个Δq计算步进电机应给的脉冲数
    Pulse[0] = abs(delta_q1*Legth_TO_STEPS_RATIO);   //double  *   int
    Pulse[1] = abs(delta_q2*Legth_TO_STEPS_RATIO);
    Pulse[2] = abs(delta_q3*Legth_TO_STEPS_RATIO);
   // angle1 = step_pulses1/ANGLE_TO_STEPS_RATIO;
   // angle2 = step_pulses2/ANGLE_TO_STEPS_RATIO;
   // angle3 = step_pulses3/ANGLE_TO_STEPS_RATIO;

    // 更新prev_q值
    prev_q1 = q1;
    prev_q2 = q2;
    prev_q3 = q3;

    // 细分为0.9度单位
    max = findMaximum(delta_q1,delta_q2,delta_q3);
    qDebug()<<"max"<<max;
    //double angleMax = static_cast<double>(max * ANGLE_TO_LEGTH);   //电机转角最大
    int minMax = abs(max * runtime);   //时间最大
    qDebug()<<"minMax="<<minMax;

    if(minMax!=0){
        qDebug()<<"hao";
        v1=abs(delta_q1) * 60000 / minMax;
        v2=abs(delta_q2) * 60000 / minMax;
        v3=abs(delta_q3) * 60000 / minMax;
    }
    if (v1 != 0) {
        Speed[0] = v1;
    } else {
        Speed[0] = 10;
    }

    if (v2 != 0) {
        Speed[1] = v2;
    } else {
        Speed[1] = 10;
    }

    if (v3 != 0) {
        Speed[2] = v3;
    } else {
        Speed[2] = 10;
    }


    if(ToolUsing)
    {
        //vq = vq + e_tool * R * offset; // 刀尖点坐标公式

        //Eigen::Vector3d vq = Eigen::Vector3d(x1, y1, z1) - e_tool * R * Eigen::Vector3d(0, 0, 1);
        /*
        double tool_z_translation = e_tool;  // 定义刀尖坐标系相对于动态坐标系在 z 轴上平移的向量
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  // 定义平移矩阵 T
        T(2, 3) = tool_z_translation;

        // 构建从静态坐标系到刀尖坐标系的坐标转换矩阵
        Eigen::Matrix4d Transformation = Eigen::Matrix4d::Identity();
        Transformation.block(0, 0, 3, 3) = R_inv;
        Transformation(0, 3) = -x;
        Transformation(1, 3) = -y;
        Transformation(2, 3) = -z;

        // 将静态坐标系下的点转换为刀尖坐标系原点
        Eigen::Vector4d vq = T * Transformation * vq0;

        //vq = vq0 + e_tool * R * offset; // 刀尖点坐标公式

*/

    }

    vq = Eigen::Vector3d(x, y, z) + e_tool * R * Eigen::Vector3d(0, 0, 1);  //实时刀尖点

    qDebug() << "vq:" << vq.x() << vq.y() << vq.z();


    /*
    QString protocolString = "0 ";

    for (int i = 0; i < 3; i++) {
        QString protocolEntry = QString("%1 %2 %3 %4 ").arg(Direction[i]).arg(Speed[i]).arg(Accelerations[i]).arg(Pulse[i]);
        protocolString += protocolEntry;
    }

    QByteArray protocolData = protocolString.toUtf8();
    qDebug()<<protocolData;
    return protocolData;  */

    QVector<int> protocolData;

    for (int i = 0; i < 3; i++) {
        int direction = *(Direction + i);
        int speed = *(Speed + i);
        int acceleration = *(Accelerations + i);
        int pulse = *(Pulse + i);

        protocolData.append(direction);
        protocolData.append(speed);
        protocolData.append(acceleration);
        protocolData.append(pulse);
    }
    return protocolData;

}


bool Calculate::calculateWorkspace(){
    if (R1 > R1_min && R1 < R1_max && R2 > R2_min && R2 < R2_max && R3 > R3_min && R3 < R3_max &&
        U1 > U1_min && U1 < U1_max && U12 > U12_min && U12 < U12_max && U2 > U2_min && U2 < U2_max &&
        U22 > U22_min && U22 < U22_max && S3 > S3_min && S3 < S3_max && S32 > S32_min && S32 < S32_max &&
        q1 <= qmax12 && q2 <= qmax12 && q3 <= qmax3 && q1 > qmin12 && q2 > qmin12 && q3 > qmin3)
    {

        return true;
        qDebug()<<"在工作空间";
    }
    else
    {
        const QByteArray data = "3";
        serial.write(data);
        qDebug()<<"不在工作空间";
        return false;
    }

}





