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


#include "ViewGraph.hpp"
#include "ViewDatabase.hpp"

#include <ctime>

#define HISTO_LENGTH 30
#define TH_LOW 50

#define PLOT true

using namespace irotavg;


bool ViewGraph::checkDistEpipolarLine(const cv::KeyPoint &kp1,
                                           const cv::KeyPoint &kp2,
                                           const cv::Mat &F12) const
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    //TODO: use matrix operations
    // vector coefs = kp1.pt.x.transpose() * F12
    
    const double a = kp1.pt.x*F12.at<double>(0,0) + kp1.pt.y*F12.at<double>(1,0) + F12.at<double>(2,0);
    const double b = kp1.pt.x*F12.at<double>(0,1) + kp1.pt.y*F12.at<double>(1,1) + F12.at<double>(2,1);
    const double c = kp1.pt.x*F12.at<double>(0,2) + kp1.pt.y*F12.at<double>(1,2) + F12.at<double>(2,2);
    
    const float num = a*kp2.pt.x + b*kp2.pt.y + c;
    
    const float den = a*a + b*b;
    
    if(den==0)
        return false;
    
    const float dsqr = num*num/den;
    return dsqr<3.84*m_scale_sigma_squares[kp2.octave];
}


void computeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0, max2=0, max3=0;
    
    for(int i=0; i<L; i++)
    {
        const int s = (int)histo[i].size();
        if(s>max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if(s>max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }
    
    if(max2 < 0.1f*(float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if(max3 < 0.1f*(float)max1)
    {
        ind3 = -1;
    }
}


int descriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    
    int dist=0;
    
    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    
    return dist;
}


int ViewGraph::findORBMatchesByBoW(Frame &f1, Frame &f2,
                                        std::vector<std::pair<int,int> > &matched_pairs,
                                        const double nnratio) const
{
    bool check_orientation = true;
    
    const auto &keypoints1 = f1.undistortedKeypoints();
    const auto &keypoints2 = f2.undistortedKeypoints();
    const auto &descriptors1 = f1.descriptors();
    const auto &descriptors2 = f2.descriptors();
    const auto &bow_features1 = f1.bow_features();
    const auto &bow_features2 = f2.bow_features();
    
    const int n1 = (int)keypoints1.size();
    //const int n2 = (int)keypoints2.size();
    
    //const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
    
    int n_matches = 0;
    
    //vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));
    std::vector<int> matches12(n1,-1);
    //std::vector<bool> matched2(n2,false);

    //const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
    
    
    std::vector<int> rotHist[HISTO_LENGTH];
    
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;
        
    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    
    
    DBoW2::FeatureVector::const_iterator it1 = bow_features1.begin();
    DBoW2::FeatureVector::const_iterator it2 = bow_features2.begin();
    DBoW2::FeatureVector::const_iterator end1 = bow_features1.end();
    DBoW2::FeatureVector::const_iterator end2 = bow_features2.end();
    
    while(it1!=end1 && it2!=end2)
    {
        if(it1->first == it2->first)
        {
            //const vector<unsigned int> vIndicesKF = KFit->second;
            //const vector<unsigned int> vIndicesF = Fit->second;
            
            //for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            for(size_t i1=0, iend1=it1->second.size(); i1<iend1; i1++)
            {
                //const unsigned int realIdxKF = vIndicesKF[iKF];
                const int idx1 = it1->second[i1];
                const cv::KeyPoint &kp1 = keypoints1[idx1];
                const cv::Mat &d1 = descriptors1.row(idx1);
                
//                MapPoint* pMP = vpMapPointsKF[realIdxKF];
//                if(!pMP)
//                    continue;
                //const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);
                
                int best_dist1 = 256;
                int best_idx2  = -1 ;
                int best_dist2 = 256;
                
                //for(size_t iF=0; iF<vIndicesF.size(); iF++)
                for(size_t i2=0, iend2=it2->second.size(); i2<iend2; i2++)
                {
                    //const unsigned int realIdxF = vIndicesF[iF];
                    const int idx2 = it2->second[i2];

                    //TODO: check this condition!!!
//                    if(vpMapPointMatches[idx2])
//                        continue;
                    if (matches12[idx2]>-1)
                    {
                        continue;
                    }
                    
                    const cv::Mat &d2 = descriptors2.row(idx2);
                    const int dist = descriptorDistance(d1, d2);
                    
                    
                    if(dist < best_dist1)
                    {
                        best_dist2 = best_dist1;
                        best_dist1 = dist;
                        best_idx2 = idx2;
                    }
                    else if(dist < best_dist2)
                    {
                        best_dist2 = dist;
                    }
                }
                
                if(best_dist1 <= TH_LOW)
                {
                    const cv::KeyPoint &kp2 = keypoints2[best_idx2];
                    
                    if( best_dist1 < nnratio*best_dist2 )
                    {
                        //vpMapPointMatches[bestIdxF]=pMP;
                        matches12[idx1] = best_idx2;
                        //const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];
                        
                        if(check_orientation)
                        {
                            double rot = kp1.angle - kp2.angle;
                            if(rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot*factor);
                            if(bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(best_idx2);
                        }
                        n_matches++;
                    }
                }
            }
            
            it1++;
            it2++;
        }
        else if(it1->first < it2->first)
        {
            it1 = bow_features1.lower_bound(it2->first);
        }
        else
        {
            it2 = bow_features2.lower_bound(it1->first);
        }
    }
    
    
    if(check_orientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        
        computeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                matches12[rotHist[i][j]]=-1;
                n_matches--;
            }
        }
    }
    
    matched_pairs.clear();
    matched_pairs.reserve(n_matches);
    
    for(size_t i=0, iend=matches12.size(); i<iend; i++)
    {
        if(matches12[i]<0)
            continue;
        matched_pairs.push_back(make_pair(i,matches12[i]));
    }
    
    return n_matches;
}


int ViewGraph::findORBMatches(Frame &f1, Frame &f2,
                                   cv::Mat F12,
                                   std::vector<std::pair<int,int> > &matched_pairs) const
{
    bool check_orientation = true;
    
    const auto &keypoints1 = f1.undistortedKeypoints();
    const auto &keypoints2 = f2.undistortedKeypoints();
    const auto &descriptors1 = f1.descriptors();
    const auto &descriptors2 = f2.descriptors();
    const auto &bow_features1 = f1.bow_features();
    const auto &bow_features2 = f2.bow_features();
    
    const int n1 = (int)keypoints1.size();
    //const int n2 = (int)keypoints2.size();
    
    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    
    int n_matches = 0;
    std::vector<int> matches12(n1,-1);
    
    std::vector<int> rotHist[HISTO_LENGTH];
    for(int i=0; i<HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    
    const float factor = 1.0f/HISTO_LENGTH;
    
    DBoW2::FeatureVector::const_iterator it1 = bow_features1.begin();
    DBoW2::FeatureVector::const_iterator it2 = bow_features2.begin();
    DBoW2::FeatureVector::const_iterator end1 = bow_features1.end();
    DBoW2::FeatureVector::const_iterator end2 = bow_features2.end();

    while(it1!=end1 && it2!=end2)
    {
        if(it1->first == it2->first)
        {
            for(size_t i1=0, iend1=it1->second.size(); i1<iend1; i1++)
            {
                const int idx1 = it1->second[i1];
                const cv::KeyPoint &kp1 = keypoints1[idx1];
                const cv::Mat &d1 = descriptors1.row(idx1);
                
                int bestDist = TH_LOW;
                int bestIdx2 = -1;
                
                for(size_t i2=0, iend2=it2->second.size(); i2<iend2; i2++)
                {
                    const int idx2 = it2->second[i2];
                    
                    // If we have already matched or there is a MapPoint skip
                    //TODO: check this condition!!!
//                    if(vbMatched2[idx2] || pMP2)
//                        continue;
                    if (matches12[idx2]>-1)
                    {
                        continue;
                    }
                    
                    const cv::Mat &d2 = descriptors2.row(idx2); // I think & really make any difference here...
                    const int dist = descriptorDistance(d1,d2);
                    
                    if(dist > TH_LOW || dist > bestDist)
                        continue;
                    
                    const cv::KeyPoint &kp2 = keypoints2[idx2];
                    
                    //if(checkDistEpipolarLine(kp1,kp2,F12))
                    if(checkDistEpipolarLine(kp2,kp1,F12))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                
                if(bestIdx2 >= 0)
                {
                    const cv::KeyPoint &kp2 = keypoints2[bestIdx2];
                    matches12[idx1] = bestIdx2;
                    n_matches++;
                    
                    if(check_orientation)
                    {
                        double rot = kp1.angle - kp2.angle;
                        if(rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot*factor);
                        if(bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }
            it1++;
            it2++;
        }
        else if(it1->first < it2->first)
        {
            it1 = bow_features1.lower_bound(it2->first);
        }
        else
        {
            it2 = bow_features2.lower_bound(it1->first);
        }
    }
    
    if(check_orientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        
        computeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);
        
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                matches12[rotHist[i][j]]=-1;
                n_matches--;
            }
        }
    }
    
    matched_pairs.clear();
    matched_pairs.reserve(n_matches);
    
    for(size_t i=0, iend=matches12.size(); i<iend; i++)
    {
        if(matches12[i]<0)
            continue;
        matched_pairs.push_back(make_pair(i,matches12[i]));
    }
    
    return n_matches;
}


int findORBMatchesLocally(Frame &f1, Frame &f2,
                          const std::vector<cv::Point2f> &guess_matching_pts,
                          std::vector<int> &matches12,
                          const int window_size, const double nnratio)
{
    bool const check_orientation = false;
    int n_matches = 0;
    
    auto &points1 = f1.undistortedKeypoints();
    auto &points2 = f2.undistortedKeypoints();
    
    auto &descriptors1 = f1.descriptors();
    auto &descriptors2 = f2.descriptors();
    
    const size_t n1 = points1.size();
    const size_t n2 = points2.size();
    
    assert(n1==guess_matching_pts.size());
    
    matches12 = std::vector<int>(n1,-1);
    
    std::vector<int> rot_hist[HISTO_LENGTH];
    for(int i=0; i<HISTO_LENGTH; i++)
        rot_hist[i].reserve(500);
    
    const float factor = 1.0f/HISTO_LENGTH;
    
    std::vector<int> matched_distances(n2,INT_MAX);
    std::vector<int> matches21(n2,-1);
    
    for(int i1=0; i1<n1; i1++)
    {
        const cv::KeyPoint &kp1 = points1[i1];
        
        int level = kp1.octave;
        int min_level = MAX(0,level-2);
        int max_level = MIN(level+2,7);
        
        std::vector<int> indices2 = f2.getFeaturesInArea(guess_matching_pts[i1].x,
                                                         guess_matching_pts[i1].y,
                                                         window_size,
                                                         min_level,max_level);
        if(indices2.empty())
            continue;
        
        cv::Mat d1 = descriptors1.row(i1);
        
        int best_dist = INT_MAX;
        int best_dist2 = INT_MAX;
        int best_idx2 = -1;
        
        for (int i2: indices2)
        {
            cv::Mat d2 = descriptors2.row(i2);
            int dist = descriptorDistance(d1,d2);
            
            if (matched_distances[i2]<=dist)
                continue;
            
            if (dist < best_dist)
            {
                best_dist2 = best_dist;
                best_dist = dist;
                best_idx2 = i2;
            }
            else if (dist < best_dist2)
            {
                best_dist2 = dist;
            }
        }
        
        if (best_dist <= TH_LOW)
        {
            if (best_dist < (float)best_dist2*nnratio)
            {
                if (matches21[best_idx2] >= 0) // (i,j) //j has been assigned
                {
                    matches12[matches21[best_idx2]] = -1;
                    n_matches--;
                }
                
                matches12[i1] = best_idx2;
                matches21[best_idx2] = i1;
                matched_distances[best_idx2] = best_dist;
                n_matches++;
                
                if(check_orientation)
                {
                    float rot = points1[i1].angle - points2[best_idx2].angle;
                    if(rot<0.0)
                        rot += 360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rot_hist[bin].push_back(i1);
                }
            }
        }
    }
    
    if(check_orientation)
    {
        int ind1 = -1, ind2 = -1, ind3 = -1;
        
        computeThreeMaxima(rot_hist,HISTO_LENGTH,ind1,ind2,ind3);
        
        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rot_hist[i].size(); j<jend; j++)
            {
                int idx1 = rot_hist[i][j];
                if(matches12[idx1]>=0)
                {
                    matches12[idx1]=-1;
                    n_matches--;
                }
            }
        }
    }
    
    //    //Update prev matched
    //    for(size_t i1=0, iend1=matches12.size(); i1<iend1; i1++)
    //        if(matches12[i1]>=0)
    //            prev_matched[i1]=keyPoints2[matches12[i1]].pt;
    
    return n_matches;
}




int findCurr2PrevLocalMatches(Frame &curr, Frame &prev,
                              std::vector<int> &target, const int rad = 100)
{
    const double nnratio = .9; //1.0  ; //.8 .9;

    auto &curr_points = curr.undistortedKeypoints();
    const int n_curr_points = (int)curr_points.size();
    std::vector<cv::Point2f> guess_matching_points;
    guess_matching_points.reserve(n_curr_points);

    for(int i=0; i<n_curr_points; i++)
    {
        guess_matching_points.push_back(curr_points[i].pt);
    }

    // change to current_to_pivot_map
    //std::vector<int> target; //-1 if could not find any match or untracked

    int found_matches = findORBMatchesLocally(curr, prev, guess_matching_points,
                                               target, rad, nnratio);

    return found_matches;
}



Pose ViewGraph::findRelativePose(Frame &f1, Frame &f2,
                                      FeatureMatches &matches,
                                      int &n_cheirality, cv::Mat &mask, cv::Mat &E,
                                      double th) const
{
    //https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    assert(matches.size()>4);
    
    const auto &kps1 = f1.undistortedKeypoints();
    const auto &kps2 = f2.undistortedKeypoints();
    
    std::vector<cv::Point2d> points1, points2;
    const int n_matches = (int)matches.size();
    points1.reserve(n_matches);
    points2.reserve(n_matches);
    
    for (auto &match: matches)
    {
        assert(match.queryIdx>=0 && match.trainIdx>=0);
        points1.push_back( kps1[match.queryIdx].pt );
        points2.push_back( kps2[match.trainIdx].pt );
    }
    
    const Camera &cam = Camera::instance();
    const CameraParameters &cam_pars = cam.cameraParameters();
    
    const double focal = cam_pars.f();
    const cv::Point2d pp = cam_pars.pp();
    
    E = cv::findEssentialMat(points1, points2,
                             focal, pp, cv::RANSAC, 0.999, th, mask);
    
    int n_rsc_inlrs = 0;
    const uchar* mask_ptr = mask.ptr<uchar>(0);
    for (int i=0; i<mask.rows; i++)
        n_rsc_inlrs += mask_ptr[i];
    
    if (n_rsc_inlrs>6) // is 5 the min num of pts ?
    {
        cv::Mat R, t;
        //int n_cheirality;
        //cv::Mat mask_pose = mask.clone();
        n_cheirality = cv::recoverPose(E, points1, points2, R, t, focal, pp, mask);
        return Pose(R,t);
    }
    else
    {
        n_cheirality = 0;
        return Pose();
    }
}


void plotMatches(Frame &prev_frame, Frame &curr_frame, FeatureMatches &matches)
{
    auto im1 = prev_frame.getImage();
    auto im2 = curr_frame.getImage();
    auto kps1 = prev_frame.keypoints();
    auto kps2 = curr_frame.keypoints();
    
    cv::Mat im_matches;
    cv::drawMatches(im1, kps1, im2, kps2, matches, im_matches);
    double s=1.0f;
    cv::Size size(s*im_matches.cols,s*im_matches.rows);
    resize(im_matches,im_matches,size);
    cv::imshow("matches after ransac", im_matches);
    cv::waitKey(1);
}


void ViewGraph::filterMatches(FeatureMatches &matches, const cv::Mat &inlrs_mask, int n_epi_inlrs) const
{
    if (n_epi_inlrs==0)
    {
        matches.clear();
        return;
    }
    
    FeatureMatches tmp_matches;
    tmp_matches.reserve(n_epi_inlrs);

    const uchar* inlrs_mask_ptr = inlrs_mask.ptr<uchar>(0);
    for(int i = 0; i <matches.size(); i++)
    {
        if (inlrs_mask_ptr[i])
        {
            tmp_matches.push_back( std::move(matches[i]) );
        }
    }
    matches = std::move(tmp_matches); // no not use epi_matches from here...
    assert(matches.size()==n_epi_inlrs);
}


FeatureMatches findSIFTMatches(Frame &prev_frame, Frame &curr_frame)
{
    cv::FlannBasedMatcher matcher;
    FeatureMatches matches;
    matcher.match(prev_frame.descriptors(), curr_frame.descriptors(), matches);
    
    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < prev_frame.descriptors().rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    FeatureMatches good_matches;
    for( int i = 0; i < prev_frame.descriptors().rows; i++ )
    {
        //if( matches[i].distance <= std::max(3*min_dist, 100.0) )
        if( matches[i].distance <= std::max(3*min_dist, 80.0) )
        {
            good_matches.push_back(matches[i]);
        }
    }
    return good_matches;
}


int ViewGraph::refinePose(Frame &f1, Frame &f2, Pose &pose, cv::Mat &E_best, FeatureMatches &matches) const
{
    // TODO pass min_matches as parameter
    const int min_matches = 100;
    if (matches.size()<5)
    {
        return (int)matches.size();
    }
    
    const int max_iters = 10;
    const Camera &cam = Camera::instance();
    const cv::Mat K = cv::Mat(cam.cameraParameters().intrinsic());
    const cv::Mat K_inv = K.inv();
    const cv::Mat K_inv_t = K_inv.t();
    
    int best_inlrs = (int)matches.size();
    cv::Mat E = E_best;
    
    std::vector< std::pair<int,int> > matched_pairs;
    cv::Mat inlrs_mask;
    
    int inlrs;
    int iters = 1;
    do
    {
        cv::Mat F = K_inv_t*E_best*K_inv;
        
        matched_pairs.clear();
        if (findORBMatches(f1, f2, F, matched_pairs) <.75*min_matches)
        {
            break;
        }
    
        FeatureMatches curr_matches;
        //curr_matches.clear();
        for (auto &match_pair: matched_pairs)
            curr_matches.push_back( cv::DMatch(match_pair.first, match_pair.second, 0) );
        
        inlrs_mask = cv::Mat();
        Pose curr_pose = findRelativePose(f1, f2, curr_matches, inlrs, inlrs_mask, E);
        
        if (inlrs > best_inlrs )
        {
            best_inlrs = inlrs;
            E_best = E;
            matches = std::move(curr_matches);
            pose = std::move(curr_pose);
            filterMatches(matches, inlrs_mask, inlrs); //move out of the while
        }
        else
        {
            break;
        }
    }
    while (iters++<max_iters);
    
    //std::cout<< "refinePose iterations " << iters <<std::endl;    
    return (int)matches.size();
}


bool ViewGraph::findPose(View &v1, View &v2, View &pivot,
                              const std::vector<int> pivot2v2, Pose &pose,
                              cv::Mat &E_out, FeatureMatches &matches)
{
    matches.clear();
    if (!v1.isConnectedTo(pivot))
    {
        return false;
    }
    
    matches = v1.getFeatureMatches(pivot);
    
    // extend the matches from pivot to v2 view : (v1, pivot) --> (v1, v2)
    FeatureMatches tmp_matches;
    tmp_matches.reserve(matches.size());
    for (auto match : matches) // prev --> pivot : (match.queryIdx, match.trainIdx)
    {
        if (pivot2v2[match.trainIdx] >= 0)
        {
            match.trainIdx = pivot2v2[match.trainIdx];
            assert(match.trainIdx>=0);
            tmp_matches.push_back(match);
        }
    }
    matches = std::move(tmp_matches);

    if (matches.size()>5)
    {
        cv::Mat inlrs_mask;
        int n_epi_inlrs;
        pose = findRelativePose(v1.frame(), v2.frame(), matches, n_epi_inlrs, inlrs_mask, E_out);
        
        filterMatches(matches, inlrs_mask, n_epi_inlrs);
        return true;
    }
    else
    {
        return false;
    }
}


Pose ViewGraph::findInitialPose(View &v1, View &v2,
                                     cv::Mat &E, FeatureMatches &matches, int min_matches)
{
    Pose pose;
    int n_epi_inlrs = 0;
    std::vector<int> curr2prev_map; //-1 if could not find any match or untracked
    int iters = 0;
    
    auto &f1 = v1.frame();
    auto &f2 = v2.frame();
    // obtain patch size based on the point distance!
    
    double rad = 2.0*m_local_rad;
    do
    {
        int n_curr2prev_matches = 0;
        
        curr2prev_map.clear();
        n_curr2prev_matches = findCurr2PrevLocalMatches(f2, f1,
                                                        curr2prev_map, rad);
        // find and update mean rad
        std::vector<double> dists;
        dists.reserve(f2.undistortedKeypoints().size());
        for (int k=0;k<f2.undistortedKeypoints().size(); k++)
        {
            const auto &idx = curr2prev_map[k];
            if (idx>=0)
            {
                const auto &p2 = f2.undistortedKeypoints()[k];
                const auto &p1 = f1.undistortedKeypoints()[idx];
                double d = cv::norm(p1.pt-p2.pt);
                dists.push_back(d);
            }
        }
        
        double mean_dist = std::accumulate( dists.begin(), dists.end(), 0.0)/dists.size();
        m_local_rad = mean_dist; //.5*(m_local_rad + mean_dist);
        
        matches.clear();
        for (int curr_idx=0; curr_idx<curr2prev_map.size(); curr_idx++)
        {
            const int &prev_idx = curr2prev_map[curr_idx]; // in prev
            if (prev_idx != -1)
            {
                matches.push_back(cv::DMatch(prev_idx, curr_idx, 0));
            }
        }
        
        if (matches.size()<=4)
        {
            m_local_rad = 1;
            std::cout << "insufficient matches... " << std::endl;
            break;
        }
        
        cv::Mat inlrs_mask;
        pose = findRelativePose(f1, f2, matches, n_epi_inlrs, inlrs_mask, E);
        
        if (n_epi_inlrs > 2*min_matches)
        {
            filterMatches(matches, inlrs_mask, n_epi_inlrs);
            break;
        }
        else
        {
            rad *= 1.25;
        }
    }
    while(iters++<5);
    
//    std::cout<<"   local search iters  =  " << iters<<std::endl;
//    std::cout<<"   local search rad    =  " << m_local_rad<<std::endl;
    
    return pose;
}


//TODO: return the candidate view
bool ViewGraph::detectLoopCandidates(View &view, std::vector<View*> &candidates)
{
    ORBVocabulary &orb_vocab = ORBVocabulary::instance();
    auto &vocab = orb_vocab.vocabulary();
    
    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
//    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const auto &bow1 = view.frame().bow();
    const View::Connections &connections = view.connections();
    
    float min_score = 1;
    float score;
    for (const auto &c: connections)
    {
        View *v2 = c.first;

        const auto &bow2 = v2->frame().bow();

        score = vocab.score(bow1, bow2);
        if(score < min_score)
            min_score = score;
    }

    // Query the database imposing the minimum score
    auto &db = ViewDatabase::instance();
    candidates = db.detectLoopCandidates(view, min_score);

    // If there are no loop candidates, just add new keyframe and return false
    if(candidates.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
}



bool ViewGraph::checkLoopConsistency(const std::vector<View*> &loop_candidates,
                          std::vector<View*> &consistent_candidates,
                          std::vector<ConsistentGroup> &consistent_groups,
                          std::vector<ConsistentGroup> &prev_consistent_groups,
                          const int covisibility_consistency_th )
{
    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    
    consistent_candidates.clear(); //  mvpEnoughConsistentCandidates.clear();
    //std::vector<ConsistentGroup> consistent_groups;   // vCurrentConsistentGroups
    consistent_groups.clear();
    
    std::vector<bool> prev_consistent_group_flag(prev_consistent_groups.size(),false);  //vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    
    
    for (View *candidate : loop_candidates) //for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        
        // --- Create candidate group -----------
        std::set<View*> candidate_group;
        const auto &connections = candidate->connections();
        for(auto pair: connections)
        {
            View *connected_view = pair.first;
            candidate_group.insert(connected_view);
        }
        candidate_group.insert(candidate);
        
        
        // --- compare candidate grou against prevoius consistent groups -----------
        
        bool enough_consistent = false;
        bool consistent_for_some_group = false;
        
        for(size_t g=0, iendG=prev_consistent_groups.size(); g<iendG; g++) //for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            // find if candidate_group is consistent with any previous consistent group
            std::set<View*> prev_group = prev_consistent_groups[g].first;
            bool consistent = false;
            for (View *candidate: candidate_group)
            {
                if( prev_group.count(candidate) )
                {
                    consistent = true;
                    consistent_for_some_group = true;
                    break;
                }
            }
            
            if(consistent)
            {
                int previous_consistency = prev_consistent_groups[g].second;
                int current_consistency = previous_consistency + 1;
                
                if( !prev_consistent_group_flag[g] )
                {
                    consistent_groups.push_back(std::make_pair(candidate_group,
                                                               current_consistency));
                    prev_consistent_group_flag[g] = true; //this avoid to include the same group more than once
                }
                
                if(current_consistency >= covisibility_consistency_th && !enough_consistent)
                {
                    consistent_candidates.push_back(candidate);
                    enough_consistent = true; //this avoid to insert the same candidate more than once
                }
            }
        }
        
        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!consistent_for_some_group)
        {
            consistent_groups.push_back(std::make_pair(candidate_group,0));
        }
    }
    
    //        // Update Covisibility Consistent Groups
    //        mvConsistentGroups = vCurrentConsistentGroups;
    //        // Add Current Keyframe to database
    //        mpKeyFrameDB->add(mpCurrentKF);
    
    return !consistent_candidates.empty();    
}

bool ViewGraph::processFrame(Frame &frame,
                             const int win_size,
                             const int min_matches)
{
    const int skip = 0;
    
    // Create View
    View *curr_view = new View(frame);
    Frame &curr_frame = curr_view->frame();
    
    const int m = 1+(int)m_views.size();
    
    if (m <= skip+1) // we are done for the first frames
    {
        m_views.push_back(curr_view);
        m_fixed_mask.push_back(false);
        
        return true;
    }
    
    const int curr_view_idx = m-1;
    int prev_view_idx = curr_view_idx-skip-1;
    
    int count_connections = 0;
    
    
    // ---------------------------------------------
    //   local search for prev frame
    // ---------------------------------------------
    View *prev_view = m_views[prev_view_idx];
    Frame &prev_frame = prev_view->frame();
    
    cv::Mat E;
    FeatureMatches matches;
    Pose relPose = findInitialPose(*prev_view, *curr_view, E, matches, min_matches);
    
    if (local_rad()<5.0f)
    {
        return false;
    }
    
    m_views.push_back(curr_view);
    m_fixed_mask.push_back(false);
    
    //std::cout<<"local_rad = "<<local_rad()<<std::endl;
    
    refinePose(prev_frame, curr_frame, relPose, E, matches);
    
    if (matches.size()<min_matches)
    {
        std::cerr << "failed to connect current frame: Insuficient matches " << matches.size() << std::endl;
        std::exit(-1);
    }
    
    View::connect(*prev_view, *curr_view, matches, relPose);
    //std::cout << "# matches for (" << prev_view_idx << ", " << curr_view_idx << ")  =  " << matches.size() << std::endl;
    
    count_connections++;
    prev_view_idx--;
    
    //plotMatches(prev_frame, curr_frame, matches);
    
    
    // ----------------------------------------------
    // solve for next frames using global search
    // ----------------------------------------------
    View &pivot = *prev_view;
    // create map from matches
    std::vector<int> pivot2current_map(pivot.frame().keypoints().size(), -1);
    for (auto match: matches) //pivot-->current
    {
        pivot2current_map[match.queryIdx] = match.trainIdx;
    }
    
    while (prev_view_idx>= 0 && (curr_view_idx-prev_view_idx) <= win_size)
    {
        prev_view = m_views[prev_view_idx];
        Frame &prev_frame = prev_view->frame();
        
        if (!findPose(*prev_view, *curr_view, pivot, pivot2current_map, relPose, E, matches))
        {
            //std::cout << "cannot connect (" << prev_view_idx << ", " << curr_view_idx << ")  -- failed to find a pose" << std::endl;
            break;
        }
        
        if (matches.size()>10)
            refinePose(prev_frame, curr_frame, relPose, E, matches);
        
        if (matches.size() < min_matches)
        {
            std::cout << "cannot connect (" << prev_view_idx << ", " << curr_view_idx << ")  -- insufficient matches: " << matches.size() << std::endl;
            break;
        }
        
        std::cout << "# matches for (" << prev_view_idx << ", " << curr_view_idx << ")  =  " << matches.size() << std::endl;
        
        View::connect(*prev_view, *curr_view, matches, relPose);
        count_connections++;
        prev_view_idx--;
        
        //plotMatches(prev_frame, curr_frame, matches);
    }
    
    if (count_connections==0)
    {
        std::cerr << "could not connect frame!" << std::endl;
        std::exit(-1);
    }
    
    return true;
}


void ViewGraph::saveViewGraph(const std::string &filename) const
{
    cv::FileStorage file(filename, cv::FileStorage::WRITE);
    
    for (auto view: m_views)
    {
        int j = view->frame().id();
        for(const auto &x : view->connections())
        {
            auto v_ptr = x.first; // ptr to connected view to j
            auto matches_ptr = x.second; // ptr to ViewConnection object
            int i = v_ptr->frame().id();
            
            if (i < j)
            {
                const Pose &pose = matches_ptr->pose(); //get pose
                file <<"i"<< i;
                file <<"j"<< j;
                file <<"R"<< cv::Mat(pose.R());
                file <<"t"<< cv::Mat(pose.t());
            }
        }
    }
}



void rmat2quat(const Pose::Mat3 &R, Pose::Vec4 &Q)
{
    //double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    double trace = R(0,0) + R(1,1) + R(2,2);
    
    if (trace > 0.0)
    {
        double s = sqrt(trace + 1.0);
        Q(3) = s * 0.5;
        s = 0.5 / s;
        Q(0) = (R(2,1) - R(1,2)) * s;
        Q(1) = (R(0,2) - R(2,0)) * s;
        Q(2) = (R(1,0) - R(0,1)) * s;
    }
    else
    {
        int i = R(0,0) < R(1,1) ? ( R(1,1) < R(2,2) ? 2 : 1) : (R(0,0) < R(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        
        double s = sqrt(R(i, i) - R(j,j) - R(k,k) + 1.0);
        Q(i) = s * 0.5;
        s = 0.5 / s;
        
        Q(3) = (R(k,j) - R(j,k)) * s;
        Q(j) = (R(j,i) + R(i,j)) * s;
        Q(k) = (R(k,i) + R(i,k)) * s;
    }
}


void ViewGraph::savePoses(const std::string &filename) const
{
    Pose::Vec4 q;
    
    std::ofstream fs(filename);
    if (fs.is_open())
    {
        for (auto view: m_views)
        {
            const auto &pose = view->pose();
            const auto &id = view->frame().id();
            const auto &t = pose.t();
            rmat2quat(pose.R(), q);

            fs << id << "\t";
            fs << setprecision(17) << std::scientific << q(3) << "\t" << q(0) << "\t" << q(1) << "\t" << q(2) << "\t" ;
            fs << setprecision(17) << std::scientific << t(0) << "\t" << t(1) << "\t" << t(2) << "\n";
        }

        fs.close();
    }
    else
    {
        std::cerr << "Unable to save results." << std::endl;
    }
}


void ViewGraph::fixPose(int idx, Pose &new_pose)
{
    assert(m_fixed_mask.size() == m_views.size());
    
    m_fixed_mask[idx] = true;
    
    auto &view = m_views[idx];
    Pose &pose = view->pose();
    pose = new_pose;
    
    assert(m_fixed_mask.size() == m_views.size());
}

bool ViewGraph::isPoseFixed(int idx) const
{
    return m_fixed_mask[idx];
}

int ViewGraph::countFixedPoses() const
{
    int resp = 0;
    for (const auto x: m_fixed_mask)
    {
        if (x) resp++;
    }
    return resp;
}


void ViewGraph::rotAvg(int winSize)
{
    assert(winSize > 2);
    const long &m = m_views.size();
    
    // upgrade winSize if few views
    winSize = MIN( (int)m, winSize);
    if (winSize<2)
    {
        return; // no variables to optimise
    }
    
    // ---------------------------------------
    // Retrieve local connections
    // ---------------------------------------
    irotavg::I_t I;
    std::vector< Pose::Vec4 > qq_vec;
    
    std::set<int> vertices;
    for (long t = m-winSize; t<m; t++)
    {
        const auto &view = m_views[t];
        
        Pose::Vec4 q;
    
        int j = view->frame().id();
        for(const auto &x : view->connections())
        {
            auto v_ptr = x.first;
            auto matches_ptr = x.second;
            int i = v_ptr->frame().id();
            
            if (i < j)
            {
                const Pose &pose = matches_ptr->pose(); //get pose
                
                I.push_back( std::make_pair(i,j) ); // indices must be reacomodated!
                vertices.insert(i);
                vertices.insert(j);
                
                rmat2quat(pose.R(), q);
                qq_vec.push_back(q);
            }
        }
    }
    
    const long &num_of_edges = qq_vec.size();
    const long &num_of_vertices = vertices.size();
    //int f = ((int)num_of_vertices) - winSize;

    if (num_of_edges < winSize)
    {
        return;  // skip optimising if insufficient edges
    }
    
    if (num_of_vertices < winSize)
    {
        return;  // skip optimising if graph is unconnected
    }

    // acomodate indices in I
    std::map<int,int> vertex_to_idx, idx_to_vertex;
    
    // count the fixed cameras
    
    // we fix cameras out of the window
    int f = ((int)num_of_vertices) - winSize;
    
    assert(f>=0);
    
    // count non-fixed cameras in the window.
    for (auto const& x : vertices)
    {
        if ( x >= m-winSize && m_fixed_mask[x] )
            f++;
    }

    int t = 0; // new idx for fixed rotations
    int k = f; // new idx for non-fixed rotations

    for (auto const& x : vertices)
    {
        if (x >= m-winSize) // cam in the window
        {
            if ( isPoseFixed(x) )
            {
                idx_to_vertex[t] = x;
                vertex_to_idx[x] = t++;
            }
            else
            {
                idx_to_vertex[k] = x;
                vertex_to_idx[x] = k++;
            }
        }
        else // always fix poses out of the window
        {
            idx_to_vertex[t] = x;
            vertex_to_idx[x] = t++;
        }
    }
    
    for (auto &c: I)
    {
        c.first  = vertex_to_idx[c.first];
        c.second = vertex_to_idx[c.second];
    }
    
    // make Q
    irotavg::Mat Q(num_of_vertices, 4);
    Pose::Vec4 q;
    for (const auto &x : vertices)
    {
        const auto &view = m_views[x];
        Pose &pose = view->pose();
        rmat2quat(pose.R(), q);
        Q.row(vertex_to_idx[x]) << q(0), q(1), q(2), q(3);
    }
    
    if (f == 0)
    {
        Q.row(0) << 0, 0, 0, 1;
        f = 1;
    }
    
    // make QQ
    irotavg::Mat QQ(num_of_edges, 4);
    for (long i=0; i<num_of_edges; i++)
    {
        const auto &q = qq_vec[i];
        QQ.row(i) << q(0), q(1), q(2), q(3);
    }
    
    // comment next line for no initialisation -- just refine
    // irotavg::init_mst(Q, QQ, I, f);

    // make A
    irotavg::SpMat A = irotavg::make_A((int)num_of_vertices, f, I);
    
    const double change_th = .001;
    
    const int l1_iters = 100;
    int l1_iters_out;
    double l1_runtime;
    irotavg::l1ra(QQ, I, A, Q, f, l1_iters, change_th, l1_iters_out, l1_runtime);

    const int irls_iters = 100;
    int irls_iters_out;
    double irls_runtime;
    irotavg::Vec weights(num_of_edges);
    irotavg::Cost cost = irotavg::Cost::Geman_McClure;
    double sigma = 5*M_PI/180.0;
    
    irotavg::irls(QQ, I, A, cost, sigma, Q, f, irls_iters, change_th,
         weights, irls_iters_out, irls_runtime);
    
    // upgrade poses for the window
    for (k=f; k<num_of_vertices; k++)
    {
        auto &view = m_views[ idx_to_vertex[k] ];

        Pose &pose = view->pose();
        
        irotavg::Quat q(Q(k,3), Q(k,0), Q(k,1), Q(k,2));
        q = q.normalized();
        
        irotavg::Mat R = q.toRotationMatrix();
        R.transposeInPlace(); // opencv is row-major
        Pose::Mat3 R_cv(R.data());
        
        pose.setR(R_cv);
    }
}

const std::vector<View *> ViewGraph::views() const
{
    return m_views;
}


bool View::connect(View &v1, View &v2, FeatureMatches matches, Pose rel_pose)
{
    assert(matches.size()>4); //
    if (v1.m_connections.count(&v2)>0)
    {
        assert(v2.m_connections.count(&v1)>0); // we must have an undirected graph
        return false;
    }
    
    // Create a ViewConnection object
    ViewConnection *connection = new ViewConnection(v1, v2,
                                                    std::move(matches),
                                                    std::move(rel_pose));
    v1.m_connections[&v2] = connection;
    v2.m_connections[&v1] = connection;
    
    return true;
}

