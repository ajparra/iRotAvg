//
//  main.cpp
//  linfslam
//
//  Created by Alvaro Parra on 26/11/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

// to setup xcode see https://medium.com/@jaskaranvirdi/setting-up-opencv-and-c-development-environment-in-xcode-b6027728003

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>


#include "Frame.hpp"
#include "ORBExtractor.hpp"
#include "FeatureTracker.hpp"
#include "SequenceLoader.hpp"

using namespace linf;

CameraParameters cam_pars;
ORB_SLAM2::ORBextractor *orb_extractor;


void config(const std::string &filename)
{
    // check settings file
    cv::FileStorage settings(filename.c_str(), cv::FileStorage::READ);
    if(!settings.isOpened())
    {
        std::cerr << "Failed to open settings file at: " << filename << std::endl;
        std::exit(-1);
    }
    
    //--------------------------------------------------------------------------------------------
    // Camera Parameters
    //--------------------------------------------------------------------------------------------
    CameraParameters::Intrinsic_type K = CameraParameters::Intrinsic_type::eye();
    
    const double fx = settings["Camera.fx"];
    const double fy = settings["Camera.fy"];
    const double cx = settings["Camera.cx"];
    const double cy = settings["Camera.cy"];
    
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    
    CameraParameters::Dist_type dist_coef;
    
    dist_coef(0) = settings["Camera.k1"];
    dist_coef(1) = settings["Camera.k2"];
    dist_coef(2) = settings["Camera.p1"];
    dist_coef(3) = settings["Camera.p2"];
    
    //    const float k3 = settings["Camera.k3"];
    //    if(k3!=0)
    //    {
    //        DistCoef.resize(5);
    //        DistCoef.at<float>(4) = k3;
    //    }
    //    DistCoef.copyTo(mDistCoef);
    //
    //    mbf = fSettings["Camera.bf"];
    //
    //    float fps = settings["Camera.fps"];
    //    if(fps==0)
    //    {
    //        fps=30;
    //    }
    //
    //    // Max/Min Frames to insert keyframes and to check relocalisation
    //    mMinFrames = 0;
    //    mMaxFrames = fps;
    //
    
    cam_pars = CameraParameters(K,dist_coef);
    
    //--------------------------------------------------------------------------------------------
    // ORB Parameters
    //--------------------------------------------------------------------------------------------
    int n_features = settings["ORBextractor.nFeatures"];
    float scale_factor = settings["ORBextractor.scaleFactor"];
    int n_levels = settings["ORBextractor.nLevels"];
    int ini_th_FAST = settings["ORBextractor.iniThFAST"];
    int min_th_FAST = settings["ORBextractor.minThFAST"];
    
    orb_extractor = new ORB_SLAM2::ORBextractor(n_features,scale_factor,
                                            n_levels,ini_th_FAST,min_th_FAST);
    
    //--------------------------------------------------------------------------------------------
    // ORB Vocabulary
    //--------------------------------------------------------------------------------------------
    std::string vocab_filename = settings["ORB_vocabulary"];
    auto &orb_vocab = ORBVocabulary::instance();
    orb_vocab.load(vocab_filename);
}


int main(int argc, const char * argv[]) {
    std::string config_filename = "/Users/a1613915/dataset/maptek/LeftCam/config.yaml";
    std::string sequence_path = "/Users/a1613915/dataset/maptek/LeftCam/2-loops-clockwise/";
    std::string image_ext = ".tif";
    int timestamp_offset = 25; //cam144_2017-03-16-154907-1803.tif
    
    config(config_filename);
    
    std::cout<< "K:\n";
    std::cout<< cam_pars.intrinsic() << std::endl;
    
    std::cout<< "dist coefs:\n";
    std::cout<< cam_pars.dist_coef() << std::endl;
    
    SequenceLoader loader(sequence_path, image_ext, timestamp_offset);
    
    FeatureTracker tracker(orb_extractor->GetScaleSigmaSquares());
    
    //read GT
    std::vector<Pose::Mat3> gt_rots;
    {
        string gt_file = "maptek_gt.txt";
        std::ifstream myfile (gt_file);
        if (!myfile.is_open())
        {
            std::cerr << "Unable to open file " << gt_file << std::endl;
            std::exit(-1);
        }
        double r1, r2, r3, r4, r5, r6, r7, r8, r9;
        while (myfile >> r1 >> r2 >> r3 >> r4 >> r5 >> r6 >> r7 >> r8 >> r9)
        {
            //gt is given as orientations so I need to transpose
            Pose::Mat3 R(r1, r2, r3,
                         r4, r5, r6,
                         r7, r8, r9); // opencv is row-major!
            
            gt_rots.push_back(R);
        }
        myfile.close();
    }
    
    
    bool is_camera_init = false; //flag for camera initialization
    
    int id = 0;
    int count = 0;
    const int sampling_step = 5;
    for (auto frame : loader)
    {
        if (count++%sampling_step != 0) // sampling
        {
            continue;
        }
        
        clock_t tic = clock(), toc;
        
        std::string impath = frame.second.string();
        
        // Set camera parameters
        if (!is_camera_init)
        {
            cv::Mat im = cv::imread(impath, cv::IMREAD_GRAYSCALE);   // Read the file
            Camera::instance().init(cam_pars, im);
            is_camera_init = true;
        }
        
        // Create a Frame object
        Frame f(id, impath, *orb_extractor);
        
        toc = clock();
        double frame_creation_time = (double)(toc-tic)/CLOCKS_PER_SEC;
        
        tic = clock();
        tracker.processFrame(f);
        toc = clock();
        double frame_processing_time = (double)(toc-tic)/CLOCKS_PER_SEC;
        
        
        tic = clock();
        if (0 && id%200==0)
        {
            Pose pose_gt;
            pose_gt.setR( gt_rots[id*sampling_step] );
            
            tracker.fixPose(id, pose_gt);
            std::cout<<"Fixing pose for view id " << id << std::endl;
            
            tracker.rotAvg(5000000); //global
        }
        else
        {
            tracker.rotAvg(50);
        }
        toc = clock();
        double rotavg_time = (double)(toc-tic)/CLOCKS_PER_SEC;
        
        printf("frame %d  -- runtimes: frame creation %.3fl; frame processing %.3f, rotavg %.3f\n",
                id, frame_creation_time, frame_processing_time, rotavg_time);
        // fix camera with GT
        // keep fixed cameras
        // tracker.loopClosure();
        
        
        tracker.savePoses("rotavg_poses.txt");
        
//        if (id%10==0)
//        {
//            std::string vg_filename = "saved_view_graph2.yaml";
//            tracker.saveViewGraph(vg_filename);
//            std::cout<< "saved view-graph to "<<vg_filename<<std::endl;
//        }
        
        id++;
    }
    
//    std::string vg_filename = "/Users/a1613915/tmp/saved_view_graph.yaml";
//    tracker.saveViewGraph(vg_filename);
//    std::cout<< "saved view-graph to "<<vg_filename<<std::endl;
    
    return 0;
}
