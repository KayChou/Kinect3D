#ifndef WIDGET_H
#define WIDGET_H

#include <iostream>
#include "KinectCapture.h"
#include "calibration.h"
#include "RGBD_FIFO_Process.h"
#include "utils.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(FIFO** FIFO_RGBD_Acquisition, QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::Widget *ui;
    FIFO** FIFO_RGBD_Acquisition;

private:
    bool bStartFlag;
    bool bOpenFlag;
    bool bCalibrationFlag;

    QImage image;
};
#endif // WIDGET_H
