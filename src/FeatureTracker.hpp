//
//  FeatureTracker.hpp
//  linfslam
//
//  Created by Alvaro Parra on 29/11/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef FeatureTracker_hpp
#define FeatureTracker_hpp

#include <stdio.h>
#include <vector>

#include "Frame.hpp"
#include "Pose.hpp"
#include "View.hpp"

#include "l1_irls.hpp"
//----------------------------------------------

namespace linf
{
class FeatureTracker
{
    
public:
    
    FeatureTracker(std::vector<float> scale_sigma_squares):
    m_scale_sigma_squares(scale_sigma_squares)
    {}
    
    void processFrame(Frame &frame);
    void processFrameSift(Frame &frame);
    
    void saveViewGraph(const std::string &filename) const;
    void savePoses(const std::string &filename) const;
    
    // refine rotations by using rotation averaging over connections in
    // the last winSize views.
    void rotAvg(const int winSize);
    
    void fixPose(int idx, Pose &pose);
    
    bool isPoseFixed(int idx) const;
    int countFixedPoses() const;
    
private:
    
    bool checkDistEpipolarLine(const cv::KeyPoint &kp1,
                               const cv::KeyPoint &kp2,
                               const cv::Mat &F12) const;
    
    int findORBMatches(Frame &f1, Frame &f2, cv::Mat F12, 
                       std::vector<std::pair<int,int> > &vMatchedPairs) const;
    
    Pose findInitialPose(View &v1, View &v2,
                         cv::Mat &E, FeatureMatches &matches, int min_matches) ;
    
    //TODO: define as const
    bool findPose(View &v1, View &v2, View &pivot_view,
                  std::vector<int> pivot2current_map, Pose &pose,
                  cv::Mat &E, FeatureMatches &matches);
    
    //  iteratively refine pose by alternating between
    //  finding matches with E
    //  and solving F from matches
    int refinePose (Frame &f1, Frame &f2, Pose &pose, cv::Mat &E, FeatureMatches &matches) const;

    std::vector<View*> m_views;
    std::vector<float> m_scale_sigma_squares;
    
    std::vector<bool> m_fixed_mask; //mask for fixed views
    
};

}
#endif /* FeatureTracker_hpp */
