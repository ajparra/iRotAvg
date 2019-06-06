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

#ifndef ViewGraph_hpp
#define ViewGraph_hpp

#include <stdio.h>
#include <vector>

#include "Frame.hpp"
#include "Pose.hpp"
#include "View.hpp"

#include "l1_irls.hpp"
//----------------------------------------------

namespace ira
{
class ViewGraph
{
    
public:
    
    ViewGraph(std::vector<float> scale_sigma_squares):
    m_scale_sigma_squares(scale_sigma_squares)
    {}
    
    bool processFrame(Frame &frame);
    
    View &currentView()
    {
        return *m_views.back();
    }
    
    void saveViewGraph(const std::string &filename) const;
    
    void savePoses(const std::string &filename) const;
    
    // refine rotations by using rotation averaging
    void rotAvg(const int winSize);
    
    void fixPose(int idx, Pose &pose);
    
    bool isPoseFixed(int idx) const;
    
    int countFixedPoses() const;
    
    
    Pose findInitialPose(View &v1, View &v2,
                         cv::Mat &E, FeatureMatches &matches, int min_matches);
    
    
    //  iteratively refine pose by alternating between
    //  finding matches with E
    //  and solving F from matches
    int refinePose (Frame &f1, Frame &f2, Pose &pose, cv::Mat &E, FeatureMatches &matches) const;

    // -----------------------------------------------------------------------------------
    // ---- loop functions... maybe move to a different class
    // -----------------------------------------------------------------------------------
    
    bool detectLoopCandidates(View &view, std::vector<View*> &candidates);
    
    typedef std::pair<std::set<View*>, int> ConsistentGroup;
    
    bool checkLoopConsistency(const std::vector<View*> &loop_candidates,
                              std::vector<View*> &consistent_candidates,
                              std::vector<ConsistentGroup> &consistent_groups,
                              std::vector<ConsistentGroup> &prev_consistent_groups,
                              const int covisibility_consistency_th=3);
    
    int findORBMatchesByBoW(Frame &f1, Frame &f2,
                            std::vector<std::pair<int,int> > &matches,
                            const double nnratio) const;
    
    Pose findRelativePose(Frame &f1, Frame &f2,
                          FeatureMatches &matches,
                          int &n_epi_inlrs, cv::Mat &mask,
                          cv::Mat &E, double th=1.0) const;
    
    void filterMatches(FeatureMatches &matches, const cv::Mat &inlrs_mask, int n_epi_inlrs) const;
    
    double local_rad() const { return m_local_rad; }
    
private:
    
    bool checkDistEpipolarLine(const cv::KeyPoint &kp1,
                               const cv::KeyPoint &kp2,
                               const cv::Mat &F12) const;
    
    int findORBMatches(Frame &f1, Frame &f2, cv::Mat F12, 
                       std::vector<std::pair<int,int> > &matches) const;
    
    //TODO: set input arguments as const
    bool findPose(View &v1, View &v2, View &pivot_view,
                  const std::vector<int> pivot2current_map, Pose &pose,
                  cv::Mat &E, FeatureMatches &matches);
    
    std::vector<View*> m_views;
    std::vector<float> m_scale_sigma_squares;
    
    std::vector<bool> m_fixed_mask; //mask for fixed views
    
    double m_local_rad = 85;
    
};

}
#endif /* ViewGraph_hpp */
