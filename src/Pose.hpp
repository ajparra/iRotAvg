/**
 * This file is part of iRotAvg.
 *
 * Created by Alvaro Parra on 19/3/19.
 * Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
 * dot edu dot au> (The University of Adelaide)
 * For more information see <https://github.com/ajparra/iRotAvg>
 *
 * This work was supported by Maptek (http://maptek.com) and the
 * ARC Grant DP160103490.
 *
 * iRotAvg is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * iRotAvg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with iRotAvg. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Pose_hpp
#define Pose_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace irotavg
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
