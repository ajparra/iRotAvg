//
//  ORBVocabulary.hpp
//  linfslam
//
//  Created by Alvaro Parra on 10/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef ORBVocabulary_hpp
#define ORBVocabulary_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

// ORB Vocabulary
#include"FORB.h"
#include"TemplatedVocabulary.h"
// We assume the vocabulary tree has 6 levels, change the 4 otherwise
#define ORB_VOCAB_LEVELS 4



namespace linf
{
    
    // ORBVocabulary -- singleton class
    class ORBVocabulary
    {
    public:
        
        typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
        Vocabulary;
        
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
