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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

#include "Frame.hpp"
#include "ORBExtractor.hpp"
#include "ViewGraph.hpp"
#include "SequenceLoader.hpp"
#include "ViewDatabase.hpp"


using namespace irotavg;

CameraParameters cam_pars;
ORB_SLAM2::ORBextractor *orb_extractor;

void config(const std::string &filename)
{
    // check settings file
    cv::FileStorage settings(filename.c_str(), cv::FileStorage::READ);
    if (!settings.isOpened())
    {
        std::cerr << "Failed to open settings file: " << filename << std::endl;
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

    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;

    CameraParameters::Dist_type dist_coef;

    dist_coef(0) = settings["Camera.k1"];
    dist_coef(1) = settings["Camera.k2"];
    dist_coef(2) = settings["Camera.p1"];
    dist_coef(3) = settings["Camera.p2"];


    cam_pars = CameraParameters(K, dist_coef);

    //--------------------------------------------------------------------------------------------
    // ORB Parameters
    //--------------------------------------------------------------------------------------------
    int n_features = settings["ORBextractor.nFeatures"];
    float scale_factor = settings["ORBextractor.scaleFactor"];
    int n_levels = settings["ORBextractor.nLevels"];
    int ini_th_FAST = settings["ORBextractor.iniThFAST"];
    int min_th_FAST = settings["ORBextractor.minThFAST"];

    orb_extractor = new ORB_SLAM2::ORBextractor(n_features, scale_factor,
                                                n_levels, ini_th_FAST, min_th_FAST);
}

// move to some util class...
void myPlotMatches(Frame &prev_frame, Frame &curr_frame, FeatureMatches &matches)
{
    auto im1 = prev_frame.getImage();
    auto im2 = curr_frame.getImage();
    auto kps1 = prev_frame.keypoints();
    auto kps2 = curr_frame.keypoints();

    cv::Mat im_matches;
    cv::drawMatches(im1, kps1, im2, kps2, matches, im_matches);
    double s = 1;
    cv::Size size(s * im_matches.cols, s * im_matches.rows);
    resize(im_matches, im_matches, size);
    cv::imshow("matches after ransac", im_matches);
    cv::waitKey(1);
}

void showRotation(const Pose &pose)
{
    constexpr double arrowLength = 150;
    constexpr int width = 350;
    constexpr int height = 350;

    cv::Vec3d v(0.0, 0.0, -arrowLength);
    v = pose.R() * v;

    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    const cv::Point center = cv::Point(width / 2, height / 2);
    const cv::Point to = center + cv::Point((int) v[0], (int) v[2]);
    cv::arrowedLine(image, center, to, cv::Scalar(255, 255, 255));

    cv::imshow("rotation", image);
}


void saveSelectedFramesIds(const std::string &filename, std::vector<int> &selected)
{
    Pose::Vec4 q;

    std::ofstream fs(filename);
    if (fs.is_open())
    {
        for (auto id: selected)
        {
            fs << id << "\n";
        }
        fs.close();
    }
    else
    {
        std::cerr << "Unable to save selected frames." << std::endl;
    }
}


int main(int argc, const char *argv[])
{
    std::string licence_notice =
        "iRotAvg Copyright (C) 2019  Alvaro Parra\n"
        "iRotAvg comes with ABSOLUTELY NO WARRANTY.\n"
        "    This is free software, and you are welcome to redistribute it\n"
        "    under certain conditions; visit\n"
        "    https://github.com/ajparra/iRotAvg#License for details.\n"
        "\n"
        "This work was supported by Maptek (http://maptek.com) and the \n"
        "ARC Linkage Project LP140100946..\n"
        "\n";


    const cv::String keys =
        "{help h usage ?   |      | print this message            }"
        "{@orb_vocabulary  |<none>| orb vocabulary                }"
        "{@config          |<none>| config file                   }"
        "{@sequence_path   |<none>| path to images                }"
        "{image_ext        |.png  | image extension               }"
        "{timestamp_offset |0     | image's name timestamp offset }"
        "{gt               |      | ground truth                  }";

    std::cout << licence_notice << std::endl;

    const bool detect_loop_closure = true;
    const int vg_win_size = 4;
    const int rotavg_win_size = 10;
    const int vg_min_matches = 100;

    cv::CommandLineParser parser(argc, argv, keys);

    parser.about("IRA v0.0.1");
    parser.printMessage();
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    const std::string vocab_filename(parser.get<cv::String>(0));
    const std::string config_filename(parser.get<cv::String>(1));
    const std::string sequence_path(parser.get<cv::String>(2));
    const std::string image_ext(parser.get<cv::String>("image_ext"));  //".tif";
    const int timestamp_offset = parser.get<int>("timestamp_offset");    //25; //cam144_2017-03-16-154907-1803.tif

    //--------------------------------------------------------------------------------------------
    // ORB Vocabulary
    //--------------------------------------------------------------------------------------------
    auto &orb_vocab = ORBVocabulary::instance();
    orb_vocab.load(vocab_filename);

    bool gt_provided = parser.has("gt");
    std::string gt_file;
    if (gt_provided)
    {
        gt_file = parser.get<cv::String>("gt");
    }

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    // ----------------------------------------------
    // read GT -- when fixing some rotations...
    // ----------------------------------------------
    std::vector<Pose::Mat3> gt_rots;
    if (gt_provided)
    {
        std::ifstream myfile(gt_file);
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


    config(config_filename);

    std::cout << "K:\n";
    std::cout << cam_pars.intrinsic() << std::endl;

    std::cout << "dist coefs:\n";
    std::cout << cam_pars.dist_coef() << std::endl;

    SequenceLoader loader(sequence_path, image_ext, timestamp_offset);

    ViewGraph view_graph(orb_extractor->GetScaleSigmaSquares());

    // to check consistency for loop clodure
    std::vector<ViewGraph::ConsistentGroup> prev_consistent_groups;
    auto &db = ViewDatabase::instance();

    std::cout << "initialising database..." << std::endl;
    db.init();


    bool is_camera_init = false; //flag for camera initialization

    std::vector<int> selected_frames;

    int id = 0;
    int count = 0;
    const int sampling_step = 1; //5
    for (auto frame : loader)
    {
        if (count++ % sampling_step != 0) // sampling
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
        double frame_creation_time = (double) (toc - tic) / CLOCKS_PER_SEC;

        tic = clock();

        bool is_frame_selected = view_graph.processFrame(f, vg_win_size, vg_min_matches);

        if (!is_frame_selected)
        {
            std::cout << "skipping frame - local rad = " << view_graph.local_rad() << std::endl << std::endl;
            continue;
        }

        selected_frames.push_back(count);

        //std::cout<<"local rad = "<<view_graph.local_rad()<<std::endl<<std::endl;

        View &view = view_graph.currentView();

        // -------- loop closure
        bool loop_new_connections = false;

        if (detect_loop_closure)
        {

            std::vector<View *> loop_candidates;
            if (id % 1 == 0 && view_graph.detectLoopCandidates(view, loop_candidates))
            {
                std::vector<View *> consistent_candidates;
                std::vector<ViewGraph::ConsistentGroup> consistent_groups;

                if (view_graph.checkLoopConsistency(loop_candidates, consistent_candidates,
                                                    consistent_groups, prev_consistent_groups))
                {
                    std::cout << " * * * loop closure detected * * *\n" << std::endl;

                    for (View *prev_view : consistent_candidates)
                    {
                        //const int min_matches = 50;
                        const int min_matches = 150;

                        //find matches
                        double nnratio = .9;
                        std::vector<std::pair<int, int> > matched_pairs;
                        view_graph.findORBMatchesByBoW(prev_view->frame(), f,
                                                       matched_pairs, nnratio);
                        FeatureMatches matches;
                        for (auto &match_pair: matched_pairs)
                            matches.push_back(cv::DMatch(match_pair.first, match_pair.second, 0));

                        //find pose
                        int inlrs;
                        cv::Mat inlrs_mask;
                        inlrs_mask = cv::Mat();
                        cv::Mat E;
                        Pose relPose = view_graph.findRelativePose(prev_view->frame(), f,
                                                                   matches, inlrs, inlrs_mask, E);
                        if (inlrs < min_matches)
                            continue;

                        view_graph.filterMatches(matches, inlrs_mask, inlrs);
                        view_graph.refinePose(prev_view->frame(), f, relPose, E, matches);

                        //std::cout<<"  #matches after refining " << matches.size() << std::endl;

                        if (matches.size() < min_matches)
                            continue;

                        View::connect(*prev_view, view, matches, relPose);
                        std::cout << "   new connection: ( " << prev_view->frame().id() << ", " << view.frame().id()
                                  << " ) ";
                        std::cout << " # matches: " << matches.size() << std::endl;
                        loop_new_connections = true;

                        myPlotMatches(prev_view->frame(), view.frame(), matches);
                    }
                }
                prev_consistent_groups = std::move(consistent_groups);
            }
            db.add(&view);

        }
        // -------- end loop closure

        toc = clock();
        double frame_processing_time = (double) (toc - tic) / CLOCKS_PER_SEC;

        tic = clock();
        bool add_correction = gt_provided && id % 20 == 0;
        if (add_correction)
        {
            Pose pose_gt;
            pose_gt.setR(gt_rots[id * sampling_step]);

            view_graph.fixPose(id, pose_gt);
            std::cout << "Fixing pose for view id " << id << std::endl;
        }


        if (loop_new_connections || add_correction)
        {
            view_graph.rotAvg(5000000); //global
        }
        else
        {
            view_graph.rotAvg(rotavg_win_size); //local
        }
        toc = clock();
        double rotavg_time = (double) (toc - tic) / CLOCKS_PER_SEC;

        printf("frame %d  -- runtimes: frame creation %.3fl; frame processing %.3f, rotavg %.3f\n",
               id, frame_creation_time, frame_processing_time, rotavg_time);

        if (id % 5 == 0)
        {
            view_graph.savePoses("rotavg_poses.txt");
            saveSelectedFramesIds("rotavg_poses_ids.txt", selected_frames);
        }

        showRotation(view.pose());
        cv::waitKey(1);

        id++;
    }

    view_graph.savePoses("rotavg_poses.txt");
    saveSelectedFramesIds("rotavg_poses_ids.txt", selected_frames);

    return 0;
}
