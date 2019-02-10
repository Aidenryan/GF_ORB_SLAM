/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif


#define DETECT_WITH_FAST
// alternative detectors are crazily slow; no wonder FAST being so popular!
//#define DETECT_WITH_HARRIS
//#define DETECT_WITH_SHITOMASI
//#define DETECT_WITH_DOG
//#define DETECT_WITH_BOXLOG

#ifdef DETECT_WITH_BOXLOG
#include "BoxLOG.hpp"
#endif


//#define ORB_EXTRACTOR_TIME_LOGGING


namespace ORB_SLAM
{

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures = 1000, float scaleFactor = 1.2f, int nlevels = 8, int scoreType=FAST_SCORE, int fastTh = 20);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

#ifdef ORB_EXTRACTOR_TIME_LOGGING
    double time_pre, time_detect, time_blur, time_extract;
#endif

protected:

    void ComputePyramid(cv::Mat image, cv::Mat Mask=cv::Mat());
    void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int scoreType;
    int fastTh;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;

    std::vector<cv::Mat> mvImagePyramid;
    std::vector<cv::Mat> mvMaskPyramid;


#ifdef DETECT_WITH_BOXLOG
    ORB_SLAM::BoxLoGDetector * boxLoGDetector_;
#endif

};

} //namespace ORB_SLAM

#endif

