#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowTitle(tr("Depth map project"));

    ui->horizontalSlider_min_disparity->setValue(sgbm->getMinDisparity());
    ui->horizontalSlider_num_of_disparity->setValue(sgbm->getNumDisparities());
    ui->horizontalSlider_block_size->setValue(sgbm->getBlockSize());
    ui->horizontalSlider_P1->setValue(sgbm->getP1());
    ui->horizontalSlider_P2->setValue(sgbm->getP2());
    ui->horizontalSlider_disp_12_max_diff->setValue(sgbm->getDisp12MaxDiff());
    ui->horizontalSlider_pre_filter_cap->setValue(sgbm->getPreFilterCap());
    ui->horizontalSlider_uniqueness_ratio->setValue(sgbm->getUniquenessRatio());
    ui->horizontalSlider_speckle_window_size->setValue(sgbm->getSpeckleWindowSize());
    ui->horizontalSlider_speckle_range->setValue(sgbm->getSpeckleRange());


}

MainWindow::~MainWindow()
{
delete ui;
}

void MainWindow::on_pushButton_left_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, "Select left picture file", QDir::homePath());
    if (filename.isNull() || filename.isEmpty())
        return;

    QImage left_picture;
    left_picture.load(filename);

    QPixmap left_pixmap = QPixmap::fromImage(left_picture);
    int max_width  = std::min(ui->label_image_left->maximumWidth(), left_picture.width());
    int max_height = std::min(ui->label_image_left->maximumHeight(), left_picture.height());
    ui->label_image_left->setPixmap(left_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    set_SADWindowSize();

    std::string filename_s = filename.toUtf8().constData();

    cv::Mat mat = cv::imread(filename_s);
    cv::cvtColor(mat, mat, CV_BGR2GRAY);
    this->left_image = mat;

    compute_depth_map();
}

void MainWindow::on_pushButton_right_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, "Select right picture file", QDir::homePath());
    if (filename.isNull() || filename.isEmpty())
        return;

    QImage right_picture;
    right_picture.load(filename);

    QPixmap right_pixmap = QPixmap::fromImage(right_picture);
    int max_width  = std::min(ui->label_image_right->maximumWidth(), right_picture.width());
    int max_height = std::min(ui->label_image_right->maximumHeight(), right_picture.height());
    ui->label_image_right->setPixmap(right_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));

    set_SADWindowSize();

    std::string filename_s = filename.toUtf8().constData();

    cv::Mat mat = cv::imread(filename_s);
    cv::cvtColor(mat, mat, CV_BGR2GRAY);
    this->right_image = mat;

    compute_depth_map();
}

void MainWindow::compute_depth_map()
{
    if (this->left_image.empty() || this->right_image.empty())
        return;

    if (left_image.rows != right_image.rows || left_image.cols != right_image.cols)
    {
         ui->label_depth_map->setText("Can't compute depth map: left and right images should be the same size");
         return;
    }

    Mat disp;
    sgbm->compute(left_image, right_image, disp);
    cv::resize(disp, disp, Size(480, 360));
    cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

    QImage disparity_image(disp.data, disp.cols, disp.rows, QImage::Format_Grayscale8);
    QPixmap disparity_pixmap = QPixmap::fromImage(disparity_image);
    int max_width  = std::min(ui->label_depth_map->maximumWidth(),  disparity_image.width());
    int max_height = std::min(ui->label_depth_map->maximumHeight(), disparity_image.height());

    ui->label_depth_map->setPixmap(disparity_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
    ui->label_depth_map->setPixmap(disparity_pixmap);
}

void MainWindow::on_horizontalSlider_pre_filter_cap_valueChanged(int value)
{
    sgbm->setPreFilterCap(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_P1_valueChanged(int value)
{
    sgbm->setP2(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_P2_valueChanged(int value)
{
    sgbm->setP2(value);
    compute_depth_map();
}

void MainWindow::set_SADWindowSize()
{
    int value = 255;

    if (! left_image.empty())
        value = std::min(value, std::min(left_image.cols, left_image.rows));
    if (! right_image.empty())
        value = std::min(value, std::min(right_image.cols, right_image.rows));

    value = std::max(value, 5);

    ui->horizontalSlider_block_size->setMaximum(value);
}

void MainWindow::on_horizontalSlider_block_size_valueChanged(int value)
{
    if ((value % 2) == 0)
    {
        value -= 1;
        ui->horizontalSlider_block_size->setValue(value);
    }

    sgbm->setBlockSize(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_min_disparity_valueChanged(int value)
{
    sgbm->setMinDisparity(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_num_of_disparity_sliderMoved(int value)
{
    set_num_of_disparity_slider_to_multiple_16(value);
}

void MainWindow::on_horizontalSlider_num_of_disparity_valueChanged(int value)
{
    set_num_of_disparity_slider_to_multiple_16(value);
}

void MainWindow::set_num_of_disparity_slider_to_multiple_16(int value)
{
    if ((value % 16) != 0)
    {
        value -= (value % 16);
        ui->horizontalSlider_num_of_disparity->setValue(value);
    }

    sgbm->setNumDisparities(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_uniqueness_ratio_valueChanged(int value)
{
    sgbm->setUniquenessRatio(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_speckle_window_size_valueChanged(int value)
{
    sgbm->setSpeckleWindowSize(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_speckle_range_valueChanged(int value)
{
    sgbm->setSpeckleRange(value);
    compute_depth_map();
}

void MainWindow::on_horizontalSlider_disp_12_max_diff_valueChanged(int value)
{
    sgbm->setDisp12MaxDiff(value);
    compute_depth_map();
}

