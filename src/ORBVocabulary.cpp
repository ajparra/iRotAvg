//
//  ORBVocabulary.cpp
//  linfslam
//
//  Created by Alvaro Parra on 10/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#include "ORBVocabulary.hpp"

using namespace linf;

void ORBVocabulary::load(const std::string &filename)
{
    //Load ORB Vocabulary
    std::cout << "\nLoading ORB Vocabulary from " << filename <<
    "\nThis could take a while..." << std::endl;

    if( !m_vocabulary.loadFromTextFile(filename) )
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Failed to open: " << filename << std::endl;
        exit(-1);
    }
    std::cout << "Vocabulary loaded!\n" << std::endl;
}
