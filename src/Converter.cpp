//
//  Converter.cpp
//  linfslam
//
//  Created by Alvaro Parra on 10/12/18.
//  Copyright © 2018 Alvaro Parra. All rights reserved.
//

#include "Converter.hpp"

using namespace ira;

//// TODO: move to an static fucntion in some util class
std::vector<cv::Mat> Converter::xxxdescriptorsMatToVector(const cv::Mat &descriptors)
{
    std::vector<cv::Mat> out;
    const int n = descriptors.rows;
    out.reserve(n);
    for (int j=0; j<n; j++)
        out.push_back(descriptors.row(j));

    return out;
}

