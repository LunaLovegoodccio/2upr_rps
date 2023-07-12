#ifndef GLOBAL_H
#define GLOBAL_H

#include <cmath>
#include <QMainWindow>
#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <QTime>
#include <QLineEdit>
#include <QThread>
#include <QDebug>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <Eigen/Eigen/Dense>
#include <QIODevice>

extern double findMaximum(double a, double b, double c);
extern int findMax(int a, int b, int c);
extern Eigen::Matrix3d Rotation(const std::string& axis, double angle);
extern int getLCM(int a, int b, int c);
extern QSerialPort serial;
extern Eigen::Vector3d vq;
extern double e_tool;
extern double q1;
extern double q2;
extern double q3;
extern int v1;
extern int v2;
extern int v3;
extern int direction1;
extern int direction2;
extern int direction3;
extern double angle1;
extern double angle2;
extern double angle3;
extern double prev_q1;
extern double prev_q2;
extern double prev_q3;
extern double delta_q1;
extern double delta_q2;
extern double delta_q3;

extern double m_z;
extern double m_a;   //绕 y 轴的旋转角度（单位为度）theta
extern double m_b;   //绕 x 轴的旋转角度（单位为度）psi


extern double l_b;
extern double l_a;
extern double l_d;

extern double array_a[3];
extern double array_b[3];

extern double con;

// 主动支链长度范围
extern double qmin12;
extern double qmax12;
extern double qmin3;
extern double qmax3;

//转动副转角
extern double R1;
extern double R2;
extern double R3;
//U副/S副转角
extern double U1;
extern double U2;
extern double U12;
extern double U22;
extern double S3;
extern double S32;

extern int R1_min;
extern int R1_max;
extern int R2_min;
extern int R2_max;
extern int R3_min;
extern int R3_max;

extern int U1_min;
extern int U1_max;
extern int U12_min;
extern int U12_max;
extern int U2_min;
extern int U2_max;
extern int U22_min;
extern int U22_max;
extern int S3_min;
extern int S3_max;
extern int S32_min;
extern int S32_max;

extern bool ToolUsing;
extern double angle;
extern double angular_speed;
extern double TrajectoryPlanning;
extern double radius;

#endif // GLOBAL_H
