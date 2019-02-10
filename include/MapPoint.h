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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include<opencv2/core/core.hpp>
#include"KeyFrame.h"
#include"Map.h"

#define ARMA_NO_DEBUG
#include "armadillo"

#include<boost/thread.hpp>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM
{

class ImageFeature;
class KeyFrame;
class Map;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    // for unit test only; not used in actual application
    MapPoint() {};

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);

    void IncreaseVisible();
    void IncreaseFound();
    float GetFoundRatio();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    //
    static bool sortMapPointGood(const MapPoint * p1,  const MapPoint * p2) {
        return p1->goodAtFrameId > p2->goodAtFrameId;
    }


public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;

    // XXX: Changed from protected to public!
    // Tracking counters
    int mnVisible;
    int mnFound;

    // Observability
    arma::mat H_meas;
    arma::mat H_proj;
    arma::mat ObsMat;
    arma::vec ObsVector;
    double ObsScore;
    int ObsRank;
    //
    float u_proj, v_proj;
    //
    long unsigned int matchedAtFrameId;
    long unsigned int updateAtFrameId;
    long unsigned int goodAtFrameId;
    long unsigned int mnUsedForLocalMap;

    std::vector<size_t> mvMatchCandidates;

protected:

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     boost::mutex mMutexPos;
     boost::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
