//
//  Camera.hpp
//  linfslam
//
//  Created by Alvaro Parra on 5/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include <opencv2/opencv.hpp>

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

// singleton
//https://stackoverflow.com/questions/1008019/c-singleton-design-pattern


namespace linf
{

class CameraParameters
{
public:
    typedef cv::Matx33d Intrinsic_type;
    typedef cv::Vec4d Dist_type;
    
    CameraParameters():
    m_intrinsic( Intrinsic_type::eye() ),
    m_dist_coef( Dist_type::all(0) )
    {}

    CameraParameters(Intrinsic_type K):
    m_intrinsic(std::move(K))
    {}
    
    CameraParameters(Intrinsic_type K, Dist_type dist_coef):
    m_intrinsic(std::move(K)),
    m_dist_coef(std::move(dist_coef))
    {}
    
    double f() const { return m_intrinsic(0,0); }
    cv::Point2d pp() const { return cv::Point2d(m_intrinsic(0,2), m_intrinsic(1,2)); }
    const Intrinsic_type &intrinsic() const { return m_intrinsic; }
    const Dist_type &dist_coef() const { return m_dist_coef; }
    
private:
    
    Intrinsic_type m_intrinsic; // intrinsic matrix
    Dist_type m_dist_coef; // [k1 k2 p1 p2]
};



// Camera -- singleton class
class Camera
{
public:
    Camera(Camera const&)          = delete;

    void operator=(Camera const&)  = delete;
    
    const double &minX() const { return m_minX; }
    const double &minY() const { return m_minY; }
    const double &maxX() const { return m_maxX; }
    const double &maxY() const { return m_maxY; }
    const double &gridCellWidthInv() const { return m_gridCellWidthInv; }
    const double &gridCellHeightInv() const { return m_gridCellHeightInv; }
    
    
    static Camera& instance()
    {
        static Camera instance;
        return instance;
    }
    
    const CameraParameters &cameraParameters() const { return m_camera_parameters; }
    
    
    void init(CameraParameters intrinsics, const cv::Mat im);
   
    
    
private:
    
    Camera() {}
    
    double m_minX, m_maxX, m_minY, m_maxY;
    double m_gridCellWidthInv;
    double m_gridCellHeightInv;
    CameraParameters m_camera_parameters;
};

}

#endif /* Camera_hpp */
