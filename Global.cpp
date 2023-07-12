#include "Global.h"

QSerialPort serial;
Eigen::Vector3d vq;
double e_tool = 50;
double q1=0;
double q2=0;
double q3=0;
int v1=0;
int v2=0;
int v3=0;
int direction1=0;
int direction2=0;
int direction3=0;
double angle1=0.00;
double angle2=0.00;
double angle3=0.00;
double prev_q1=330.000;
double prev_q2=330.000;
double prev_q3=330.000;
double delta_q1=0;
double delta_q2=0;
double delta_q3=0;

double m_z=323.54;    //零点m_z值
double m_a=0.00;
double m_b=0.00;

double l_b=75;    //动平台半径
double l_a=140;   //静平台半径
double l_d=50;  //动z轴

double array_a[3]={l_a,l_a,l_a};
double array_b[3]={l_b,l_b,l_b};

double con = M_PI / 180;

// 主动支链长度范围
double qmin12 = 140.0;
double qmax12 = 340.0;   //295
double qmin3 = 140.0;
double qmax3 = 400.0;

//转动副转角
double R1=101.36;
double R2=101.36;
double R3=101.36;
//U副/S副转角
double U1=101.36;
double U2=101.36;
double U12=90;
double U22=90;
double S3=101.36;
double S32=90;

int R1_min=60;
int R1_max=150;
int R2_min=60;
int R2_max=150;
int R3_min=50;
int R3_max=130;

int U1_min=52;
int U1_max=128;
int U12_min=45;
int U12_max=135;
int U2_min=52;
int U2_max=128;
int U22_min=45;
int U22_max=135;
int S3_min=60;
int S3_max=150;
int S32_min=58;
int S32_max=122;

double TrajectoryPlanning = 250;

bool ToolUsing = false;
double angle = 0.0;
double angular_speed = 30;  // 每次增加的角度
double radius = 25; //圆形半径

double findMaximum(double a, double b, double c) {
    double maximum = a;
    if (b > maximum) {
        maximum = b;
    }
    if (c > maximum) {
        maximum = c;
    }

    return maximum;
}

int findMax(int a, int b, int c) {
    return std::max(std::max(a, b), c);
}

Eigen::Matrix3d Rotation(const std::string& axis, double angle) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double phi = angle * M_PI / 180.0;

    if (axis == "x") {
        R(1, 1) = std::cos(phi);
        R(1, 2) = -std::sin(phi);
        R(2, 1) = std::sin(phi);
        R(2, 2) = std::cos(phi);
    } else if (axis == "y") {
        R(0, 0) = std::cos(phi);
        R(0, 2) = std::sin(phi);
        R(2, 0) = -std::sin(phi);
        R(2, 2) = std::cos(phi);
    } else if (axis == "z") {
        R(0, 0) = std::cos(phi);
        R(0, 1) = -std::sin(phi);
        R(1, 0) = std::sin(phi);
        R(1, 1) = std::cos(phi);
    }

    return R;
}

int getLCM(int a, int b, int c) {
    // 计算两个数的最大公约数
    int gcd_ab = 0;
    for (int i = 1; i <= a && i <= b; ++i) {
        if (a % i == 0 && b % i == 0) {
            gcd_ab = i;
        }
    }

    int lcm_ab = (a * b) / gcd_ab;

    int gcd_abc = 0;
    for (int i = 1; i <= lcm_ab && i <= c; ++i) {
        if (lcm_ab % i == 0 && c % i == 0) {
            gcd_abc = i;
        }
    }

    int lcm_abc = (lcm_ab * c) / gcd_abc;

    return lcm_abc;
}




