/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "DepthMap.h"

DepthMap::DepthMap(cv::Mat& _ImageLeftRGB, cv::Mat& _ImageRightRGB, cv::Mat& _Twc, float _fx, float _fy, float _cx, float _cy) {

    ImageLeftRGB = _ImageLeftRGB.clone();
    cv::cvtColor(_ImageLeftRGB, ImageLeftGray, CV_RGB2GRAY);
    cv::cvtColor(_ImageRightRGB, ImageRightGray, CV_RGB2GRAY);

    width = ImageLeftRGB.cols;
    height = ImageLeftRGB.rows;

    Disparity = cv::Mat(height, width, CV_32F);
    currentDepthMap = new DepthHypo[width*height];

    Twc = _Twc.clone();

    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;

    ComputeDepthFromStereoPair();
}

DepthMap::~DepthMap() {
    delete currentDepthMap;
}

void DepthMap::ComputeDepthFromStereoPair() {
    // allocate memory for disparity images
    const int32_t dims[3] = {width, height, width}; // bytes per line = width
    float D1_data[width * height];
    float D2_data[width * height];

    // process
    Elas::parameters param;
    param.postprocess_only_left = true;
    Elas elas(param);
    elas.process((uchar*) ImageLeftGray.data, (uchar*) ImageRightGray.data, D1_data, D2_data, dims);

    // Store and create depth hypo
    // Only left image are consider
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++)
        {
            int index = i*width + j;
            if(D1_data[index] > 0) {
                // Set 
                currentDepthMap[index].depth = bf/(D1_data[index]*fx);
                currentDepthMap[index].var = 1.0; // should proportional to the matching error and gradient val
            }
        }
    }
    
    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 0;
    for (int32_t i = 0; i < width * height; i++) {
        if (D1_data[i] > disp_max) disp_max = D1_data[i];
        if (D2_data[i] > disp_max) disp_max = D2_data[i];
    }

    // Save disparity
    cv::Mat ImageD1(height, width, CV_8U);
    cv::Mat ImageD2(height, width, CV_8U);

    uchar* ImageD1_data = (uchar*) ImageD1.data;
    uchar* ImageD2_data = (uchar*) ImageD2.data;
    for (int32_t i = 0; i < width * height; i++) {
        ImageD1_data[i] = (uint8_t) max(255.0 * D1_data[i] / disp_max, 0.0);
        ImageD2_data[i] = (uint8_t) max(255.0 * D2_data[i] / disp_max, 0.0);
    }
    
    cv::imshow("left disparity", ImageD1);
    cv::imshow("right disparity", ImageD1);
    cv::waitKey(0);
}