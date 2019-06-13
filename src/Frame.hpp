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

#ifndef Frame_hpp
#define Frame_hpp

#include <stdio.h>
#include "opencv2/opencv.hpp"

#include <iostream>
#include <exception>
#include "ORBExtractor.hpp"
#include "Camera.hpp"

// BoW
#include "BowVector.h"
#include "FeatureVector.h"

#include "ORBVocabulary.hpp"


//---------------------------------------------------
namespace irotavg
{

class Frame
{
#define USE_ORF_FLAG true
    
public:
    
    Frame(int id, const std::string path, ORB_SLAM2::ORBextractor &orbExtractor) :
    USE_ORB(USE_ORF_FLAG),   //
    m_id(id),
    m_path(path),
    m_orb_extractor(orbExtractor)
    {
        findFeatures();
        undistortKeypoints();
        assignFeaturesToGrid();
        computeBoW();
    }
    
    std::vector<cv::KeyPoint> &keypoints() { return m_keypoints; }
    std::vector<cv::KeyPoint> &undistortedKeypoints() { return m_undistorted_keypoints; }
    
    cv::Mat &descriptors() { return m_descriptors; }
    
    int id() const { return m_id; }
    
    cv::Mat getImage();
    
    
    bool posInGrid(const cv::KeyPoint &kp, int &x, int &y);
    
    void assignFeaturesToGrid();
    
    std::vector<int> getFeaturesInArea(double x, double y, double r,
                                       int minLevel=-1, int maxLevel=-1) const;
    
    
    const DBoW2::FeatureVector &bow_features() const { return m_bow_features; }
    
    const DBoW2::BowVector &bow() const { return m_bow; }
    
    
private:
    void findFeatures();
    
    void undistortKeypoints();
    
    void computeBoW();
    
    const bool USE_ORB;
    
    const int m_id;
    const std::string m_path;
    ORB_SLAM2::ORBextractor &m_orb_extractor;
    //static cv::Ptr<cv::FeatureDetector> detector (cv::ORB::create() );
    
    std::vector<cv::KeyPoint> m_keypoints;
    std::vector<cv::KeyPoint> m_undistorted_keypoints;
    cv::Mat m_descriptors;
    
    // Keypoints are assigned to cells in a grid
    std::vector<int> m_grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    
    // -----------------------------------------------------------
    // BoW members
    DBoW2::BowVector m_bow;
    DBoW2::FeatureVector m_bow_features;
};
    
}

#endif /* Frame_hpp */
