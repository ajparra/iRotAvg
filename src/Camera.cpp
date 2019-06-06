/**
 * This file is part of IRA.
 *
 * Created by Alvaro Parra on 19/3/19.
 * Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
 * dot edu dot au> (The University of Adelaide)
 * For more information see <https://github.com/ajparra/IRA>
 *
 * This work was supported by Maptek (http://maptek.com) and the
 * ARC Grant DP160103490.
 *
 * IRA is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * IRA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with IRA. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Camera.hpp"

using namespace ira;

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
