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


#ifndef ORBVocabulary_hpp
#define ORBVocabulary_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

// ORB Vocabulary
#include"FORB.h"
#include"TemplatedVocabulary.h"
// We assume the vocabulary tree has 6 levels, change the 4 otherwise
#define ORB_VOCAB_LEVELS 4



namespace irotavg
{
    
    // ORBVocabulary -- singleton class
    class ORBVocabulary
    {
    public:
        
        typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> Vocabulary;
        
        ORBVocabulary(ORBVocabulary const&) = delete;
        
        void operator=(ORBVocabulary const&)  = delete;
        
        static ORBVocabulary& instance()
        {
            static ORBVocabulary instance;
            return instance;
        }
        
        const Vocabulary &vocabulary() const { return m_vocabulary; }
        
        void load(const std::string &filename);
        
        
    private:
        
        ORBVocabulary(){}
        
        std::string m_vocabulary_filename;
        Vocabulary m_vocabulary;
    };
}


#endif /* ORBVocabulary_hpp */
