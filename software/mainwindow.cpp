#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QUdpSocket>
#include <QNetworkDatagram>

QValueAxis *axisX = new QValueAxis;
QValueAxis *axisY = new QValueAxis;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    series = new QLineSeries();

    chart = new QChart();

    chart->legend()->hide();

    //chart->createDefaultAxes();

    chart->setAnimationOptions(QChart::SeriesAnimations);



    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->resize(400, 500);


    axisX->setRange(0, 255);
    axisX->setTickCount(10);
    axisX->setLabelFormat("%.1f");
    axisY->setMin(0);
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);



    socket = new QUdpSocket(this);
    socket->bind(QHostAddress("192.168.1.166"), 10000);

    connect(socket, SIGNAL(readyRead()), this, SLOT(readDatagram()));

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::readDatagram(void){
    QNetworkDatagram data;



    while(socket->hasPendingDatagrams()){
        data = socket->receiveDatagram();
    }

    series->clear();
    float max = 0;
    for(int i = 0; i < 1024 - 4; i += 4){
        series->append(i/4 - 1, *(float *)&data.data()[i]);
        max = *(float *)&data.data()[i] > max ? *(float *)&data.data()[i] : max;
    }

    if(max > 5000) axisY->setMax(max + 500);
    else axisY->setMax(5000);


    series->detachAxis(axisY);
    series->attachAxis(axisY);
    series->attachAxis(axisX);

    //series->append(3, *(float *)&data.data()[12]);
    //series->append(4, *(float *)&data.data()[16]);
    //series->append(5, *(float *)&data.data()[20]);
    chart->addSeries(series);



    chartView->show();
}


