#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    system_init();
    c = new Calculate();
    qDebug()<<"ok";
    //connect(this, &MainWindow::dataSent, this, &MainWindow::handleDataSent, Qt::QueuedConnection);
    qDebug()<<"ok";
    //this->setMaximumSize(2800,2000);
    //this->setMinimumSize(200,150);
    this->setWindowTitle("机械原理");
    QFont f("仿宋",15);
    //qDebug() << "Timer current state: " << timer.isActive();
}


MainWindow::~MainWindow()
{

}

void MainWindow::delay(int t)
{
    QTime dieTime = QTime::currentTime().addMSecs(t);
    while(QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::system_init()
{
    serial.setPortName("COM13");
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setParity(QSerialPort::NoParity);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    ui->senddata_edit->setReadOnly(true);
    ui->recievedata->setReadOnly(1);
    //connect(this, &MainWindow::pulseSent, this, &MainWindow::handlePulseSent);
    //connect(ui->Quit_Button, &QPushButton::clicked, this, &MainWindow::on_Quit_Button_clicked);
    //connect(ui->diandong_z_jian, &QPushButton::pressed, this, &MainWindow::on_diandong_z_jian_pressed);
    //connect(ui->diandong_z_jian, &QPushButton::released, this, &MainWindow::on_diandong_z_jian_released);
    //connect(ui->UnlockStall, &QPushButton::clicked, this, &MainWindow::on_UnlockStall_clicked);
    //connect(ui->Send, &QPushButton::clicked, this, &MainWindow::on_Send_clicked);
    //connect(ui->StackCheck, &QPushButton::clicked, this, &MainWindow::on_StackCheck_clicked);
    connect(this, SIGNAL(customReadyRead(int, int ,int)),
            this, SLOT(handleMotorPositionData(int, int, int)));
    connect(this, SIGNAL(globalVariableUpdated()), this, SLOT(updateLineEditValue()));
    if (!serial.open(QIODevice::ReadWrite))
    {
        qDebug() << "Failed to open serial port";
        return;
    }
    if (serial.isOpen()) {
        timer = new QTimer(this);
        //connect(timer, &QTimer::timeout, this, &MainWindow::printHelloWorld);
        connect(timer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
        connect(timer, &QTimer::timeout, this, &MainWindow::updateLineEditValue);
        timer->start(1000); // 1000毫秒 = 1秒
    }

}

void MainWindow::timerTimeout()
{
    //sendCommand();
    readSerialData();
}

void  MainWindow::updateLineEditValue()
{
    ui->feedback_z->setText(QString::number(m_z));
    ui->feedback_a->setText(QString::number(m_a));
    ui->feedback_b->setText(QString::number(m_b));

}


void MainWindow::sendCommand()
{
    if (!serial.isOpen()) {
        //qDebug() << "Serial port is not open.";
        return;
    }
    QByteArray sendData;
    sendData.append("4");
    serial.write(sendData);
    //qDebug() << "Sent command to read motor positions.";
}

void MainWindow::readSerialData()
{
    // 读取串口数据
    QByteArray responseData = serial.readAll();

    if (responseData.size() >= 16 && responseData[0] == 'A' && responseData[1] == 'A') {
        QString responseString(responseData);

        QStringList motorAngles = responseString.split(' ', QString::SkipEmptyParts);
        if (motorAngles.size() >= 4) {
            QString motor1Angle = motorAngles[1].trimmed();
            QString motor2Angle = motorAngles[2].trimmed();
            QString motor3Angle = motorAngles[3].trimmed();

            // 发送自定义信号，将电机位置数据传递给槽函数进行处理
            emit customReadyRead(motor1Angle, motor2Angle, motor3Angle);

            qDebug() << "Received motor positions: "
                     << "Motor 1: " << motor1Angle << ", "
                     << "Motor 2: " << motor2Angle << ", "
                     << "Motor 3: " << motor3Angle;
        }
    }


}

void MainWindow::handleMotorPositionData(int motor1Angle, int motor2Angle, int motor3Angle)
{
    double q1_show = motor1Angle * ANGLE_TO_LEGTH;
    double q2_show = motor2Angle * ANGLE_TO_LEGTH;
    double q3_show = motor2Angle * ANGLE_TO_LEGTH;
    ui->QLineEdit_q1->setText(QString::number(q1_show));
    ui->QLineEdit_q2->setText(QString::number(q2_show));
    ui->QLineEdit_q3->setText(QString::number(q3_show));
    qDebug() << "Received motor position data: "
             << motor1Angle << ", " << motor2Angle << ", " << motor3Angle;
}



// Function to process received data from the microcontroller
void MainWindow::processResponse(const QByteArray& response) {
    if (response.startsWith("AA")) {
        QString motorAnglesString = response.mid(2);
        QStringList motorAngles = motorAnglesString.split(' ');
    }
    else {
        qDebug() << "Invalid response.";
    }
}


void MainWindow::displayReceivedData(const QByteArray& data){
    ui->senddata_edit->setPlainText(QString::fromUtf8(data));
}


void MainWindow::on_clear_clicked()
{
    ui->recievedata->clear();
}

void MainWindow::on_Send_clicked()
{
    QString command = ui->send_data->text();
    QByteArray data = command.toUtf8();
    serial.write(data);
    serial.flush();
    qDebug()<<"send success";
    ui->recievedata->setPlainText(QString::fromUtf8(data));

}


void MainWindow::on_UnlockStall_clicked()
{
    const QByteArray data = "3";
    serial.write(data);
    serial.flush();
    qDebug() << "Send success: " << data;
    ui->recievedata->setPlainText(QString::fromUtf8(data));

}


void MainWindow::on_StackCheck_clicked()
{
    const QByteArray data = "5";
    serial.write(data);
    qDebug()<<"5";
    ui->recievedata->setPlainText(QString::fromUtf8(data));
}

void MainWindow::on_Quit_Button_clicked()
{
    qApp->exit();
}


void MainWindow::on_fuwei_clicked()
{
    const QByteArray data = "2 ";
    serial.write(data);
    serial.flush();
    qDebug()<<"fasong";
    ui->recievedata->setPlainText(data);
}

void MainWindow::handleDataSent()
{
    qDebug() << "Entering handleDataSent()";
    QVector<int> Data;
    if(TN)
    {
        Data=c->calculate_Z(q1,q2,q3);
    }
    else
    {
        Data=c->calculate_N(m_z,m_a,m_b);
    }


    if (Data.size() < 12)
    {
        qDebug() << "Data向量的大小不正确";
        return;
    }
    Workspace=c->calculateWorkspace();
    if(Workspace)
    {
    }
    else
    {
        QMessageBox::information(nullptr, "Information", "Boundary Point!!!");
        return;
    }

    qDebug() << "在工作空间内";

    static int FirstPulseMax = 30000;
    qDebug()<<"data="<<Data;
    int totalPulses1 = Data[3];
    int totalPulses2 = Data[7];
    int totalPulses3 = Data[11];
    qDebug()<<"totalPusel1="<<totalPulses1;
    qDebug()<<"totalPusel2="<<totalPulses2;
    qDebug()<<"totalPuse3="<<totalPulses3;
    int MaxPulse = findMax(totalPulses1,totalPulses2,totalPulses3);
    int count = MaxPulse / FirstPulseMax;
    int remainder = MaxPulse % FirstPulseMax;
    if(remainder != 0)  //向上取整
    {
        count +=1;
    }

    if (count != 0) {
        int everyPulse1 = totalPulses1 / count;
        int everyPulse2 = totalPulses2 / count;
        int everyPulse3 = totalPulses3 / count;
        qDebug()<<"everyPulse1= "<<everyPulse1;
        Data[3]=everyPulse1;
        Data[7]=everyPulse2;
        Data[11]=everyPulse3;

        bool dataFirstSent=false;
        if (!continueSending) {
            if (!dataFirstSent) {
                const QByteArray data = "3";
                serial.write(data);
                qDebug() << "Data First sent to serial port";
                dataFirstSent = true;
                continueSending = true;
            }
        }
        while(continueSending){
            QByteArray receivedData;
            bool receiveSuccess = false;
            while(!receiveSuccess){
                if(serial.waitForReadyRead(50000))
                {
                    receivedData += serial.readAll();
                    ui->senddata_edit->setPlainText(receivedData);
                    if(receivedData.contains("0") || receivedData.contains("3"))
                    {
                        qDebug()<<"cici";
                        qDebug() <<"Recieve OK,MCU has already run";
                        receiveSuccess = true;
                        QString protocolString = "0 ";
                        for (int i = 0; i < 12; i++) {
                            QString protocolEntry = QString("%1 ").arg(Data[i]);
                            protocolString += protocolEntry;
                        }

                        QByteArray protocolData = protocolString.toUtf8();
                        qDebug()<<protocolData;
                        serial.write(protocolData);
                        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
                else
                {
                    qDebug() << "Timeout: Failed to receive OK from the lower machine";
                    continueSending = false;
                    break;
                }
            }

            count -= 1;
            if (count == 0) {
                AlreadySending = true;
                continueSending = false;   //在执行完成后，不再持续发送
                qDebug()<<"执行完毕!";
                break;
            }

            //if(!continueSending) break;
        }
    }
    qDebug()<<"count="<<count;


}


//z-
void MainWindow::on_diandong_z_jian_pressed()
{
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_z-=okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。

    }
}

//z+
void MainWindow::on_diandongzjia_pressed()
{
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_z+=okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。
    }
}

void MainWindow::on_SEND1_clicked()
{
    m_z = (ui->QLineEdit_z->text()).toDouble();
    m_a = (ui->QLineEdit_a->text()).toDouble();
    m_b = (ui->QLineEdit_b->text()).toDouble();
    handleDataSent();
}

void MainWindow::on_huiling_clicked()
{
    const QByteArray data = "2";
    serial.write(data);
    serial.flush();
    qDebug() << "Send success: " << data;
    ui->recievedata->setPlainText(QString::fromUtf8(data));
}


void MainWindow::on_diandong_Bjian_clicked()
{
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_b -= okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。

    }
}

void MainWindow::on_diandong_Bjia_clicked()
{
    qDebug() <<"test";
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_b += okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。

    }
}

void MainWindow::on_diandong_Ajian_clicked()
{
    qDebug() <<"test";
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_a -= okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。

    }
}

void MainWindow::on_diandongA_jia_clicked()
{
    int okCount=0;
    if (!PressDown) {
        ++okCount;
        m_a += okCount;
        handleDataSent();
        qDebug() <<"press have already work";
        okCount=0;
        PressDown=false;
        //break;   //按下按钮后仅处理一段消息并发送，而不继续在循环中处理更多的消息。

    }
}

void MainWindow::on_pushButton_clicked()
{
    int num_iterations = 360 / angular_speed;   //迭代次数
    m_a = 0;
    m_b = 0;
    m_z = TrajectoryPlanning;
    handleDataSent();
    ToolUsing = true;
    for (int i = 0; i <= num_iterations; i++) {
        qDebug()<<"test begin";
        angle = i * angular_speed;
        //angle = angle * 2 * M_PI / 360.0;
        qDebug()<<"angel="<<angle;
        double center_x = 0;
        double center_y = 0;

        double x1 = center_x + radius * std::cos(angle * con);   //静系下
        double y1 = center_y + radius * std::sin(angle * con);
        double z1 = TrajectoryPlanning;

        vq.x() = x1;
        vq.y() = y1;
        vq.z() = z1;
        double a1 = std::asin(-vq.y() / e_tool);
        m_a = a1 / con;   //弧度转角度
        double b1 = std::atan(vq.x() / vq.z());
        m_b = b1 / con;
        m_z = vq.z() - e_tool * std::cos(m_b * con) * std::cos(m_a * con);

        handleDataSent();

        qDebug()<<"m_a="<<m_a;
        qDebug()<<"m_b="<<m_b;
        qDebug()<<"m_z="<<m_z;
    }
    ToolUsing = false;
}


void MainWindow::on_pushButton_4_clicked()
{
    TN=true;
    q1 = (ui->QLineEdit_q1->text()).toDouble();
    q2 = (ui->QLineEdit_q2->text()).toDouble();
    q3 = (ui->QLineEdit_q3->text()).toDouble();
    handleDataSent();
    TN=false;
}
