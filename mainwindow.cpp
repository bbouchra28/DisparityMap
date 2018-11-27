#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <vector>
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

    compute_disparity_map();
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

    compute_disparity_map();
}

void MainWindow::compute_disparity_map()
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
    //cv::resize(disp, disp, Size(480, 360));
    cv::normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
    float QMatrix[16] = {1, 0, 0, -1141.452,
                         0, 1, 0, -927.052,
                         0, 0, 0, 2329.558,
                         0, 0, 1/270, (1141.452-1241.731)/270};
    Mat Q = Mat(4, 4, CV_32F, QMatrix);
    Mat depth;
    reprojectImageTo3D(disp, depth, Q);
    imshow("Depth", depth);
    imshow("Disp", disp);
    //Point3d p(depth);
    cv::FileStorage file("depth.txt", cv::FileStorage::WRITE);
    // Write to file!
    //file << "Depth" << p;
    std::cout << "rows = " << depth.rows << " Cols = " << depth.cols << std::endl;
    std::cout << "first pixel = " << depth.at<Point3f>(0,0) << std::endl;

    std::vector<cv::Point3f> _points;
    std::vector<std::vector<int>> _faces;
    for (int i = 0; i < depth.rows; ++i)
    {
        for (int j = 0; j < depth.cols; ++j)
       _points.push_back(Point3f(depth.at<Point3f>(i,j)));
    }


    // create cubic for each point.
    std::vector<cv::Point3f> p_tmp;
        for (auto &p : _points)
        {
            p_tmp.push_back(p);

            p_tmp.push_back(p);
            p_tmp.back().z += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().z += 0.1;
            p_tmp.back().y += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().y += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().x += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().x += 0.1;
            p_tmp.back().z += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().x += 0.1;
            p_tmp.back().y += 0.1;
            p_tmp.back().z += 0.1;
            p_tmp.push_back(p);
            p_tmp.back().x += 0.1;
            p_tmp.back().y += 0.1;
        }

        _points = p_tmp;

        //===

        int indx = 1;
        size_t size = _points.size();
        for(auto &p: _points)
        {
            int elp = indx % 8;
            if(indx > 7 && elp == 0 && indx < size-8)
            {
                _faces.push_back(std::vector<int>{4, indx, indx+1 ,indx+2, indx+3});
                _faces.push_back(std::vector<int>{4, indx+7, indx+6 ,indx+5, indx+4});
                _faces.push_back(std::vector<int>{4, indx, indx+4 ,indx+5, indx+1});
                _faces.push_back(std::vector<int>{4, indx+1, indx+5 ,indx+6, indx+2});
                _faces.push_back(std::vector<int>{4, indx+2, indx+6 ,indx+7, indx+3});
                _faces.push_back(std::vector<int>{4, indx+3, indx+7 ,indx+4, indx+0});
            }
            indx++;
        }

        std::ofstream filestream;
        filestream.open("output.ply",std::ios::out);


        int p_size = (int)(_points.size());
        int f_size = (int)(_faces.size());

        // MARK: Header writing
        filestream << "ply" << std::endl <<
                   "format " << "ascii" << " 1.0" << std::endl <<
                   "comment file " << std::endl <<
                   "element vertex " << p_size << std::endl <<
                   "property float x" << std::endl <<
                   "property float y" << std::endl <<
                   "property float z" << std::endl <<
                   "element face " << f_size <<std::endl <<
                   "property list uchar int vertex_indices" <<std::endl <<
                   "end_header" << std::endl;


        for(auto &p : _points) // Loop through all elements
        {
            filestream.precision(4);
            filestream << std::fixed << p.x << " " << p.y << " " << p.z << std::endl;
        }
        for(auto &f : _faces)
        {
            size_t f_s = f.size();
            for(int i=0; i<f_s; i++)
            {
                auto _f = f[i];
                filestream << std::setprecision(2) << _f << (i==(f_s-1)?"":" ");
            }
            filestream << std::endl;
        }

        filestream.close();

    //std::cout << "Depth = "<< std::endl << " "  << depth << std::endl << std::endl;

    /*QImage disparity_image(disp.data, disp.cols, disp.rows, QImage::Format_Grayscale8);
    QPixmap disparity_pixmap = QPixmap::fromImage(disparity_image);
    int max_width  = std::min(ui->label_depth_map->maximumWidth(),  disparity_image.width());
    int max_height = std::min(ui->label_depth_map->maximumHeight(), disparity_image.height());

    ui->label_depth_map->setPixmap(disparity_pixmap.scaled(max_width, max_height, Qt::KeepAspectRatio));
    ui->label_depth_map->setPixmap(disparity_pixmap);*/

}

void MainWindow::compute_depth_map()
{
    /*Mat leftImagePoint, rightImagePoint;
    findChessboardCorners(left_image, cvSize(left_image.cols,left_image.rows), leftImagePoint);
    findChessboardCorners(right_image, cvSize(right_image.cols,right_image.rows), rightImagePoint);
    Mat left_cam = (Mat_<double>(3,3) << 2329.558, 0, 1141.452, 0, 2329.558, 927.052, 0, 0, 1);
    Mat right_cam = (Mat_<double>(3,3) << 2329.558, 0, 1241.731, 0, 2329.558, 927.052, 0, 0, 1);
    std::vector<std::vector<Point3f> > objectPoints(2);
    const float squareSize = 1.f;
    objectPoints[0] = Create3DChessboardCorners(cvSize(left_image.cols,left_image.rows), squareSize);
    objectPoints[1] = Create3DChessboardCorners(cvSize(right_image.cols,right_image.rows), squareSize);
    Mat leftDistortionCoefficient, rightDistortionCoefficient;
    Mat rotationMatrix;
    Mat translationVector;
    Mat essentialMatrix;
    Mat fundamentalMatrix;
    Mat R1, R2, P1, P2, Q;
    stereoCalibrate(objectPoints, leftImagePoint, rightImagePoint,
                      left_cam, leftDistortionCoefficient,
                      right_cam, rightDistortionCoefficient,
                      imageSize, rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
                      TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                      CV_CALIB_FIX_ASPECT_RATIO +
                      CV_CALIB_ZERO_TANGENT_DIST +
                      CV_CALIB_SAME_FOCAL_LENGTH +
                      CV_CALIB_RATIONAL_MODEL +
                      CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    stereoRectify(left_cam, leftDistortionCoefficient, right_cam, rightDistortionCoefficient,
                  cvSize(left_image.cols,left_image.rows), rotationMatrix, translationMatrix,
                  R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, (0,0));*/

}

void MainWindow::on_horizontalSlider_pre_filter_cap_valueChanged(int value)
{
    sgbm->setPreFilterCap(value);
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_P1_valueChanged(int value)
{
    sgbm->setP2(value);
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_P2_valueChanged(int value)
{
    sgbm->setP2(value);
    compute_disparity_map();
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
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_min_disparity_valueChanged(int value)
{
    sgbm->setMinDisparity(value);
    compute_disparity_map();
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
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_uniqueness_ratio_valueChanged(int value)
{
    sgbm->setUniquenessRatio(value);
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_speckle_window_size_valueChanged(int value)
{
    sgbm->setSpeckleWindowSize(value);
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_speckle_range_valueChanged(int value)
{
    sgbm->setSpeckleRange(value);
    compute_disparity_map();
}

void MainWindow::on_horizontalSlider_disp_12_max_diff_valueChanged(int value)
{
    sgbm->setDisp12MaxDiff(value);
    compute_disparity_map();
}

