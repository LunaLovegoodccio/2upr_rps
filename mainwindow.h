#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "calculate.h"
#include "Global.h"

class Calculate;


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

const int pulsesPerIteration = 200;
class MainWindow : public QMainWindow{
    Q_OBJECT

signals:
    //void sendQuery(const QByteArray& query);
    void customReadyRead(const QString &motor1Angle, const QString &motor2Angle, const QString &motor3Angle);
    void globalVariableUpdated();

public:
    //QSerialPort *serial = new QSerialPort(this);


    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void system_init();
    void delay(int t);
    void displayReceivedData(const QByteArray& data);
    void SenddataToMCU(const QByteArray& data);
    void handleDataSent();
    void processResponse(const QByteArray& response);
    void sendCommand(const QByteArray& command);



public slots:
     //void handleDataSent();

private slots:
    //void receivedata();

    //QString handlePulseSent(const int* motorDirections, const int* motorSpeeds, const int* motorAccelerations, const int* motorPulses);
    //void handleReceiveAck(const QString& ack);

    void on_clear_clicked();
    void on_Send_clicked();
    void on_UnlockStall_clicked();
    void on_StackCheck_clicked();
    void on_Quit_Button_clicked();   
    void on_SEND1_clicked();
    void on_huiling_clicked();


    void on_diandong_z_jian_pressed();   //z-
    void on_fuwei_clicked();
    void on_diandongzjia_pressed();
    void on_diandong_Bjian_clicked();
    void on_diandong_Bjia_clicked();
    void on_diandong_Ajian_clicked();
    void on_diandongA_jia_clicked();

    void sendCommand();
    void readSerialData();
    void timerTimeout();
    void handleMotorPositionData(int motor1Angle, int motor2Angle, int motor3Angle);
    void updateLineEditValue();   //实时更新QLineEdit

    void on_pushButton_clicked();

    void on_pushButton_4_clicked();

private:
    QTimer *timer;
    QThread* workerThread;
    QString currentProtocolString;
    int pulseCount = 0; //脉冲数
    int ready = 0;//是否能够发送信息
    int max=0;//最大值
    int END=1;//当前指令运行结束
    int init=0;//是否在进行初始化
    int Jog_state=0;  //点动状态
    int Release=0;    //点动放开状态
    int ZeroState=0;  //回零标志位
    int m_sendDataRequested=0;
    int m_ackReceived=0;  //ok接收标志位
    bool continueSending = false; //持续发送标志位
    bool receiveSuccess = false;  //下位机执行结束标志位
    bool dataSent=false;  //点动标志位
    bool PressDown=false;  //按钮按下标志位
    bool Workspace=true;
    bool AlreadySending=false;
    bool TN=false;  //正逆解选择标志位
    QByteArray m_protocolData;
    Ui::MainWindow *ui;
    Calculate* c;

};

#endif // MAINWINDOW_H
