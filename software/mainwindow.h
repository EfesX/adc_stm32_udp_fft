#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QtCharts>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT



public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();





private:
    Ui::MainWindow *ui;

    QUdpSocket *socket;
    QLineSeries *series;
    QChart *chart;
    QChartView *chartView;

public slots:
    void readDatagram(void);
};
#endif // MAINWINDOW_H
