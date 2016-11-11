/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DepthMap.h
 * Author: yiluo
 *
 * Created on November 10, 2016, 6:25 PM
 */

#ifndef DEPTHMAP_H
#define DEPTHMAP_H

// Opencv
#include <opencv2/opencv.hpp>

// libelas
#include "../Thirdparty/libelas/src/elas.h"

// General
#include <iostream>
#include <string>

// System
#include "DepthHypo.h"

using namespace std;

class DepthMap {
public:
    
    DepthMap(cv::Mat& _ImageLeftRGB, cv::Mat& _ImageRightRGB, cv::Mat& _Twc, float _fx, float _fy, float _cx, float _cy);
    ~DepthMap();
    void ComputeDepthFromStereoPair();
    
    DepthHypo* currentDepthMap;
    
    cv::Mat ImageLeftGray;
    cv::Mat ImageRightGray;
    cv::Mat ImageLeftRGB;
    cv::Mat Disparity;
    
    cv::Mat Twc;

    float fx, fy, cx, cy;
    int width, height;
};


#endif /* DEPTHMAP_H */

