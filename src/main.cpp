#include <iostream>
#include <string>

// libelas
#include "../Thirdparty/libelas/src/elas.h"

// Opencv
#include <opencv2/opencv.hpp>

// PCL
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

int main(int argc, char** argv) {

    string file_1 = "/home/yiluo/SFM/workspace/SemiDense/bin/l00000046.png";
    string file_2 = "/home/yiluo/SFM/workspace/SemiDense/bin/r00000046.png";

    cv::Mat ImageLeftRGB = cv::imread(file_1);

    cv::Mat ImageLeft = cv::imread(file_1, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat ImageRight = cv::imread(file_2, CV_LOAD_IMAGE_GRAYSCALE);
    //    cv::imshow("left", ImageRight);
    //    cv::waitKey(0);

    int32_t width = ImageLeft.cols;
    int32_t height = ImageRight.rows;

    // allocate memory for disparity images
    const int32_t dims[3] = {width, height, width}; // bytes per line = width
    float D1_data[width * height];
    float D2_data[width * height];

    // process
    Elas::parameters param;
    param.postprocess_only_left = true;
    Elas elas(param);
    elas.process((uchar*) ImageLeft.data, (uchar*) ImageRight.data, D1_data, D2_data, dims);

    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 0;
    for (int32_t i = 0; i < width * height; i++) {
        if (D1_data[i] > disp_max) disp_max = D1_data[i];
        if (D2_data[i] > disp_max) disp_max = D2_data[i];
    }

    // Save disparity
    string output_1 = "/home/yiluo/SFM/workspace/SemiDense/bin/l00000046.png";
    string output_2 = "/home/yiluo/SFM/workspace/SemiDense/bin/r00000046.png";
    cv::Mat ImageD1(height, width, CV_8U);
    cv::Mat ImageD2(height, width, CV_8U);

    uchar* ImageD1_data = (uchar*) ImageD1.data;
    uchar* ImageD2_data = (uchar*) ImageD2.data;
    for (int32_t i = 0; i < width * height; i++) {
        ImageD1_data[i] = (uint8_t) max(255.0 * D1_data[i] / disp_max, 0.0);
        ImageD2_data[i] = (uint8_t) max(255.0 * D2_data[i] / disp_max, 0.0);
    }

    cv::imwrite(output_1, ImageD1);
    cv::imwrite(output_2, ImageD2);

    /// Generate grad_x and grad_y
    cv::Mat grad, grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;
    //
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    
    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    cv::Sobel(ImageLeft, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    cv::Sobel(ImageLeft, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    /// Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    // Create 3D PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Camera
    float fx = 718.856;
    float fy = 718.856;
    float cx = 607.1928;
    float cy = 185.2157;
    float bf = 386.1448;
    
    int gradThreshold = 20;

    // Create points
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++) {
            float u = j;
            float v = i;

            float disparity = (double) D1_data[i * width + j];
            if (disparity < 0) continue;
            if (grad.at<uchar>(i,j) < gradThreshold) continue;

            float Z = bf * fx / disparity;
            float X = (u - cx) * Z / fx;
            float Y = (v - cy) * Z / fy;
            cv::Vec3b pointRGB = ImageLeftRGB.at<cv::Vec3b>(v, u);
            pcl::PointXYZRGB point;
            point.x = X;
            point.y = Y;
            point.z = Z;
            point.b = pointRGB[0];
            point.g = pointRGB[1];
            point.r = pointRGB[2];
            if (Z > 0 && Z < 100000)
                point_cloud_ptr->points.push_back(point);

        }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }


    return 0;
}
