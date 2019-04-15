//
//  Pose.hpp
//  linfslam
//
//  Created by Alvaro Parra on 5/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef Pose_hpp
#define Pose_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace linf
{

class Pose
{
    
public:
    typedef cv::Matx33d Mat3;
    typedef cv::Vec3d Vec3;
    typedef cv::Vec4d Vec4;
    
    Pose(): m_R(Mat3::eye()), m_t(cv::Vec3d(0,0,0)) {}
    
    Pose(Mat3 R, Vec3 t): m_R(R), m_t(t) {}
    
    void setR(Mat3 R) { m_R = std::move(R); }
    
    void setT(Vec3 t) { m_t = std::move(t); }
    
    const Mat3 &R() const { return m_R; }
    
    const Vec3 &t() const { return m_t; }
    
private:
    
    Mat3 m_R;
    Vec3 m_t;
};

}

#endif /* Pose_hpp */
