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

#ifndef ViewDatabase_hpp
#define ViewDatabase_hpp

#include "ViewDatabase.hpp"
#include <stdio.h>
#include "Frame.hpp"
#include "View.hpp"
#include "ORBVocabulary.hpp"


namespace ira
{
    class ViewDatabase
    {
    public:
        
        ViewDatabase(ViewDatabase const&) = delete;
        
        void operator=(ViewDatabase const&) = delete;
        
        static ViewDatabase& instance()
        {
            static ViewDatabase instance;
            return instance;
        }
        
        void init()
        {
            const ORBVocabulary &voc = ORBVocabulary::instance();
            m_inverted_file.resize(voc.vocabulary().size());
        }
        
        
        void add(View *view);
        void erase(View *view);
        
        void clear()
        {
            m_inverted_file.clear();
            init();
        }

        std::list<View*> findViewsSharingWords(View &view, std::map<View*, int> &num_of_shared_words);
        std::vector<View*> detectLoopCandidates(View &view, double min_score);
        
      
        
    private:
        
        ViewDatabase(){}
        
        //std::string m_vocabulary_filename;
        
        // Inverted file
        std::vector< std::list<View*> > m_inverted_file;
    };
}


#endif /* ViewDatabase_hpp */
