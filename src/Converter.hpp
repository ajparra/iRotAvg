//
//  Converter.hpp
//  linfslam
//
//  Created by Alvaro Parra on 10/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#ifndef Converter_hpp
#define Converter_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace ira
{
    namespace Converter
    {
        std::vector<cv::Mat> xxxdescriptorsMatToVector(const cv::Mat &descriptors);
    }
}


#endif /* Converter_hpp */
