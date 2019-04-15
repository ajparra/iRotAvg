//
//  SequenceLoader.cpp
//  linfslam
//
//  Created by Alvaro Parra on 6/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#include "SequenceLoader.hpp"

using namespace linf;

SequenceLoader::SequenceLoader(std::string path, std::string im_ext, int timestamp_offset)
{
    //im_ext -- image extension, includes the dot Ex: ".png"
    //timestamp_offset -- offset of the timestamp in the filenae
    
    namespace fs = ::boost::filesystem;
    pair_id_path frame_pair;
    int timestamp;
    fs::path p(path);
    for (auto it = fs::directory_iterator(p); it != fs::directory_iterator(); it++)
    {
        if(fs::is_regular_file(*it) && it->path().extension().string() == im_ext)
        {
            timestamp = std::stoi(it->path().stem().string().substr(timestamp_offset));
            frame_pair = std::make_pair(timestamp, it->path());
            m_frames.push_back(frame_pair);
        }
    }
    // sort by the timestamp
    std::sort(m_frames.begin(), m_frames.end());
}
