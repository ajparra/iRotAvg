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

#include "SequenceLoader.hpp"

using namespace irotavg;

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
