//
//  Camera.cpp
//  linfslam
//
//  Created by Alvaro Parra on 5/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#include "Camera.hpp"

using namespace linf;

void Camera::init(CameraParameters intrinsics, const cv::Mat im)
{
    m_camera_parameters = std::move(intrinsics);
    
    const CameraParameters::Dist_type &dist_coef = m_camera_parameters.dist_coef();
    
    // uncalibrated case
    if(dist_coef(0) != 0.0)
    {
        cv::Mat mat(4,2,CV_64F);
        mat.at<double>(0,0) = 0.0;        mat.at<double>(0,1) = 0.0;
        mat.at<double>(1,0) = im.cols;    mat.at<double>(1,1) = 0.0;
        mat.at<double>(2,0) = 0.0;        mat.at<double>(2,1) = im.rows;
        mat.at<double>(3,0) = im.cols;    mat.at<double>(3,1) = im.rows;
        
        const CameraParameters::Intrinsic_type &K = m_camera_parameters.intrinsic();
        
        // Undistort corners
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K, dist_coef, cv::Mat(), K);
        mat = mat.reshape(1);
        
        m_minX = std::min(mat.at<double>(0,0), mat.at<double>(2,0));
        m_maxX = std::max(mat.at<double>(1,0), mat.at<double>(3,0));
        m_minY = std::min(mat.at<double>(0,1), mat.at<double>(1,1));
        m_maxY = std::max(mat.at<double>(2,1), mat.at<double>(3,1));
    }
    else
    {
        m_minX = 0.0f;
        m_maxX = im.cols;
        m_minY = 0.0f;
        m_maxY = im.rows;
    }
    
    m_gridCellWidthInv = (FRAME_GRID_COLS)/(m_maxX-m_minX);
    m_gridCellHeightInv = (FRAME_GRID_ROWS)/(m_maxY-m_minY);
}
