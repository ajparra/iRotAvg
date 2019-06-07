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

#ifndef View_hpp
#define View_hpp

#include <stdio.h>
#include <vector>
#include "Frame.hpp"
#include "Pose.hpp"

#include <functional>   // std::greater
#include <algorithm>    // std::sort


namespace irotavg
{

typedef std::vector<cv::DMatch> FeatureMatches;
    
class View
{
    
public:
    
    class ViewConnection
    {
    public:
        ViewConnection(View &v1, View &v2, FeatureMatches matches, Pose rel_pose):
        m_v1(v1), m_v2(v2), m_matches(std::move(matches)), m_rel_pose(std::move(rel_pose))
        {}
        
        FeatureMatches &matches() { return m_matches; }
        size_t size() const { return m_matches.size(); }
        const Pose &pose() const { return m_rel_pose; } 
        
    private:
        View &m_v1;
        View &m_v2;
        FeatureMatches m_matches;
        Pose m_rel_pose;
    };
    
    typedef std::map<View*,ViewConnection*> Connections;
    
    
    View(Frame &frame): m_frame(frame) {}
    
    Frame &frame() { return m_frame; }
    
    Pose &pose() { return m_pose; }
    
    bool isConnectedTo(const View &view) const
    {
        return (m_connections.count(const_cast<View*>(&view))>0);
    }
    
    FeatureMatches &getFeatureMatches(const View &view)
    {
        //make sure noes are connected!
        assert(this->isConnectedTo(view));
        ViewConnection *view_connection = m_connections[const_cast<View*>(&view)];
        FeatureMatches &matches = view_connection->matches();
        return matches;
    }
    
    const Connections &connections() const { return m_connections; }
    
    
    static bool connect(View &v1, View &v2, FeatureMatches matches, Pose rel_pose);
    
    //TODO: implement the desctructor...
    
    
    
    std::vector<View*> getBestCovisibilityViews(const int n)
    {

        typedef std::pair<int, View*> WeightedView;
        
        std::vector<WeightedView> weighted_views;
        for (auto connection: m_connections)
        {
            View *view = connection.first;
            ViewConnection *view_connection = connection.second;
            
            // check there is no null elements in connections!!!
            assert(view != NULL);
            assert(view_connection != NULL);
            
            weighted_views.push_back( std::make_pair( (int)view_connection->size(), view) );
        }
        
        // sort by the number of connections in descending order
        std::sort(weighted_views.begin(), weighted_views.end(),
                  [](const WeightedView &x, const WeightedView &y) //TODO: fix x is becames null??? why???
                  {
                      return x.first > y.first;
                  }
                  );
        
        std::vector<View*> covisibility;
        covisibility.reserve(n);
        
        int i = 0;
        for (auto wv: weighted_views)
        {
            if (i++==n) break;

            covisibility.push_back(wv.second);
        }
        
        return covisibility;
    }
    
    
    
private:
    
    
    //int m_id;
    Frame m_frame; //make sure Frame is movable!
    Pose m_pose; // absolute pose
    std::map<View*,ViewConnection*> m_connections;
};

}


#endif /* View_hpp */
