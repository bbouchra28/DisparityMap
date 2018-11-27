#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <QMainWindow>
#include <QFileDialog>
#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>

using namespace cv;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_left_clicked();

    void on_pushButton_right_clicked();

    void on_horizontalSlider_num_of_disparity_sliderMoved(int position);

    void on_horizontalSlider_num_of_disparity_valueChanged(int value);

    void on_horizontalSlider_block_size_valueChanged(int value);

    void on_horizontalSlider_pre_filter_cap_valueChanged(int value);

    void on_horizontalSlider_min_disparity_valueChanged(int value);

    void on_horizontalSlider_uniqueness_ratio_valueChanged(int value);

    void on_horizontalSlider_speckle_window_size_valueChanged(int value);

    void on_horizontalSlider_speckle_range_valueChanged(int value);

    void on_horizontalSlider_disp_12_max_diff_valueChanged(int value);

    void on_horizontalSlider_P1_valueChanged(int value);

    void on_horizontalSlider_P2_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    Mat left_image;
    Mat right_image;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 144, 3, 216, 864, 1, 63, 10, 100, 32);
    void compute_depth_map();
    void compute_disparity_map();
    void set_SADWindowSize();
    void set_num_of_disparity_slider_to_multiple_16(int position);
};

#endif // MAINWINDOW_H


