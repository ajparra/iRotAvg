//
//  Abstraction of a view in the view-graph
//  linfslam
//
//  Created by Alvaro Parra on 13/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef View_hpp
#define View_hpp

#include <stdio.h>
#include <vector>
#include "Frame.hpp"
#include "Pose.hpp"


namespace linf
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
    
    //TODO: implement the desctructor, to clean the theap...
    
private:
    
    
    //int m_id;
    Frame m_frame; //make sure Frame is movable!
    Pose m_pose; // absolute pose
    std::map<View*,ViewConnection*> m_connections;
};

}


#endif /* View_hpp */
