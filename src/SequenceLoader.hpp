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

#ifndef SequenceLoader_hpp
#define SequenceLoader_hpp

#include <stdio.h>
#include <string.h>
#include <boost/filesystem.hpp>


namespace irotavg
{
    class SequenceLoader
    {
    public:

        typedef std::pair<int,boost::filesystem::path> pair_id_path;

        SequenceLoader(std::string path, std::string im_ext, int timestamp_offset=0);
        
        std::vector<pair_id_path>::const_iterator begin() const
        {
            return m_frames.begin();
        }
        
        std::vector<pair_id_path>::const_iterator end() const
        {
            return m_frames.end();
        }

    private:
        std::vector<pair_id_path> m_frames;
    };
}
    
#endif /* SequenceLoader_hpp */
