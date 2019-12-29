/**
 * This file is part of iRotAvg.
 *
 * Created by Alvaro Parra on 19/3/19.
 * Copyright © 2019 Alvaro Parra <alvaro dot parrabustos at adelaide
 * dot edu dot au> (The University of Adelaide)
 * For more information see <https://github.com/ajparra/iRotAvg>
 *
 * This work was supported by Maptek (http://maptek.com) and the
 * ARC Linkage Project LP140100946.
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

#include "Frame.hpp"
#include "Camera.hpp"
#include "Converter.hpp"
#include <ctime>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/opencv.hpp>


using namespace irotavg;

std::string type2str(int type) {
    std::string r;
    
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    
    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    
    r += "C";
    r += (chans+'0');
    
    return r;
}


void Frame::findFeatures()
{
    cv::Mat image = getImage();
    cv::imshow("current", image);

    if (USE_ORB)
    {
        // use ORBextractor from ORBSLAM2
        (m_orb_extractor)(image,cv::Mat(),m_keypoints,m_descriptors);

//        // ignore keypoints on the top of the image
//        std::vector<cv::KeyPoint> keypoints;
//        keypoints.reserve(m_keypoints.size());
//        std::vector<bool> map(m_keypoints.size());
//        int i = 0;
//        for (auto &kp: m_keypoints)
//        {
//            if (kp.pt.y>300) // && kp.pt.y<900)
//            {
//                map[i]=true;
//                keypoints.push_back(kp);
//            }
//            i++;
//        }
//        cv::Mat descriptors = cv::Mat::zeros(0, 32, CV_8UC1);
//        for(i=0; i<m_descriptors.rows; i++)
//        {
//            if (map[i])
//            {
//                descriptors.push_back(m_descriptors.row(i));
//            }
//        }
//        m_keypoints = std::move(keypoints);
//        m_descriptors = descriptors.clone();
    }
    else
    {
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();  // have to use Ptr and create()
        sift->detectAndCompute(image, cv::Mat(), m_keypoints, m_descriptors);
    }
}


void Frame::undistortKeypoints()
{
    const CameraParameters &cam_pars = Camera::instance().cameraParameters();
    const CameraParameters::Dist_type &dist_coef = cam_pars.dist_coef();
    
    if(dist_coef(0)==0.0)
    {
        m_undistorted_keypoints = m_keypoints;
        return;
    }
    
    // Fill matrix with points
    const int n = (int)m_keypoints.size();
    cv::Mat mat(n,2,CV_64F);
    for(int i=0; i<n; i++)
    {
        mat.at<double>(i,0) = m_keypoints[i].pt.x;
        mat.at<double>(i,1) = m_keypoints[i].pt.y;
    }
    
    // undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat,mat,cam_pars.intrinsic(),dist_coef,cv::Mat(),
                        cam_pars.intrinsic());
    mat = mat.reshape(1);
    
    // Fill undistorted keypoint vector
    //m_undistorted_keypoints.resize(n);
    m_undistorted_keypoints.reserve(n);
    for(int i=0; i<n; i++)
    {
        cv::KeyPoint kp = m_keypoints[i];
        kp.pt.x = mat.at<double>(i,0);
        kp.pt.y = mat.at<double>(i,1);
        //m_undistorted_keypoints[i] = kp;
        m_undistorted_keypoints.push_back(kp);
    }
}


cv::Mat Frame::getImage()
{
    cv::Mat image;
    
    image = cv::imread(this->m_path, cv::IMREAD_GRAYSCALE);   // Read the file
    
    if( !image.data )                              // Check for invalid input
    {
        class FrameException: public std::exception
        {
            virtual const char* what() const throw()
            {
                return "Could not read frame";
            }
        } frameException;
        throw frameException;
    }
    return image;
}



bool Frame::posInGrid(const cv::KeyPoint &kp, int &x, int &y)
{
    const Camera &cam = Camera::instance();
    x = round((kp.pt.x-cam.minX())*cam.gridCellWidthInv());
    y = round((kp.pt.y-cam.minY())*cam.gridCellHeightInv());
    
//    std::cout<< "FRAME_GRID_COLS " << FRAME_GRID_COLS <<std::endl;
//    std::cout<< "FRAME_GRID_ROWS " << FRAME_GRID_ROWS <<std::endl;

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(x<0 || x>=FRAME_GRID_COLS || y<0 || y>=FRAME_GRID_ROWS)
        return false;
    
    return true;
}

void Frame::assignFeaturesToGrid()
{
    const int n = (int)m_undistorted_keypoints.size();
    
    int reserveSize = 0.5f*n/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    
    for(int i=0; i<FRAME_GRID_COLS; i++)
        for (int j=0; j<FRAME_GRID_ROWS; j++)
            m_grid[i][j].reserve(reserveSize);
    
    for(int i=0; i<n; i++)
    {
        const cv::KeyPoint &kp = m_undistorted_keypoints[i];//mvKeysUn[i];
        
        int x, y;
        if(posInGrid(kp,x,y))
            m_grid[x][y].push_back(i);
    }
}


std::vector<int> Frame::getFeaturesInArea(const double x, const double y, const double r,
                                   const int minLevel, const int maxLevel) const
{
    const int n = (int)m_keypoints.size();
    std::vector<int> indices;
    indices.reserve(n);
    
    Camera &cam = Camera::instance();
    
    const int minCellX = std::max(0,(int)floor((x-cam.minX()-r)*cam.gridCellWidthInv()));
    if(minCellX>=FRAME_GRID_COLS)
        return indices;
    
    const int maxCellX = std::min((int)FRAME_GRID_COLS-1,(int)ceil((x-cam.minX()+r)*cam.gridCellWidthInv()));
    if(maxCellX<0)
        return indices;
    
    const int minCellY = std::max(0,(int)floor((y-cam.minY()-r)*cam.gridCellHeightInv()));
    if(minCellY>=FRAME_GRID_ROWS)
        return indices;
    
    const int maxCellY = std::min((int)FRAME_GRID_ROWS-1,(int)ceil((y-cam.minY()+r)*cam.gridCellHeightInv()));
    if(maxCellY<0)
        return indices;
    
    const bool checkLevels = (minLevel>0) || (maxLevel>=0);
    
    for(int ix = minCellX; ix<=maxCellX; ix++)
    {
        for(int iy = minCellY; iy<=maxCellY; iy++)
        {
            const auto cell = m_grid[ix][iy];
            if(cell.empty())
                continue;
            
            //for(int kpIdx: cell)
            for(int j=0, jend=(int)cell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = m_undistorted_keypoints[cell[j]];  //  mvKeysUn[cell[j]] undistorted keys
                if(checkLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }
                
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;
                
                if(fabs(distx)<r && fabs(disty)<r)
                    indices.push_back(cell[j]);
            }
        }
    }
    
    return indices;

}


void Frame::computeBoW()
{
    if(m_bow.empty() || m_bow_features.empty())
    {
        ORBVocabulary &orb_vocab = ORBVocabulary::instance();
        auto &vocab = orb_vocab.vocabulary();
        
        std::vector<cv::Mat> descriptors = Converter::descriptorsMatToVector(m_descriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        vocab.transform(descriptors, m_bow, m_bow_features, ORB_VOCAB_LEVELS);
    }
}

const double Frame::getTimeStamp() const
{
    return m_time_stamp;
}
