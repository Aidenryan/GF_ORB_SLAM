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

#ifndef TRACKING_H
#define TRACKING_H

#include "FramePublisher.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapPublisher.h"
//#include "PoseSequence.h"
//#include "ObsThreshSequence.h"

#include <tf/transform_broadcaster.h>

#include "Observability.h"

#include <set>
#include <utility>
#include <algorithm>

#include <Eigen/Dense>
using namespace Eigen;

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

//#include <gtsam/geometry/Pose3.h>
//#include <gtsam/base/Vector.h>
//#include <gtsam/base/Matrix.h>
//#include <gtsam/base/Manifold.h>
//#include <gtsam/navigation/CombinedImuFactor.h>
//#include <gtsam/navigation/ImuFactor.h>


//#define TIMECOST_VERBOSE
//#define LMKNUM_VERBOSE

/* --- options for exporting data in demo figures --- */
//#define LOGGING_MATCHING


/* --- options of baseline methods --- */
//#define ORB_SLAM_BASELINE
//#define INLIER_ONLY_BASELINE


/* --- options of non-necessary viz codes --- */
// when running on long-term large-scale dataset, this will save us a lot of time!
#define DISABLE_MAP_VIZ


/* --- options of additional search after pose estimation --- */
 #define DELAYED_MAP_MATCHING


/* --- options to priortize feature matching wrt local map --- */

//#define RANDOM_FEATURE_MAP_MATCHING
//#define LONGLIVE_FEATURE_MAP_MATCHING

#define GOOD_FEATURE_MAP_MATCHING
#define FRAME_MATCHING_INFO_PRIOR

#define USE_INFO_MATRIX
//#define USE_HYBRID_MATRIX
//#define USE_OBSERVABILITY_MATRIX


// For low-power devices with 2-cores, disable multi-thread matrix building
#define USE_MULTI_THREAD        true // false //


/* --- options to fair comparison wrt other VO pipelines --- */
// time to init tracking with full feature set
#define TIME_INIT_TRACKING      5 // 10 //

//#define DISABLE_RELOC

#define MAX_FRAME_LOSS_DURATION     5

//#define WITH_IMU_PREINTEGRATION
//#define MIN_NUM_GOOD_FEATURES     40 // For KITTI
//#define MIN_NUM_GOOD_FEATURES     30 // For TUM
#define MIN_NUM_GOOD_FEATURES       20 // 30 // 50 // 80 // (with 80 the performance of GF is closed to ALL) // 100 // For EuRoC
// 40 is used when insert GF at trackMotionModel


//#define BUDGET_NUM_MATCHES          80 // 120 // 160
#define BUCKET_WIDTH                50
#define BUCKET_HEIGHT               50

//
//#define ENABLE_EXPLICIT_OUTLIER_REJ
//#define RANSAC_POOL_FOR_SUBSET
#define RANSAC_ITER_NUMBER      150 // 100 // 50 //


/* --- options of candidate pooling methods; discarded since good feature 2.0 --- */
//#define QUALITY_POOL_FOR_SUBSET
//#define QUALITY_POOL_SCALE         2 // 1.25 // 2.5 // 1.0 // 1.5 // 1.75 //
//#define QUALITY_POOL_PERCENTILE     0.75 // 0.5 // 0.3 // 0.85 //
//#define QUALITY_POOL_MAXSIZE        250


/* ================================================================ */


namespace ORB_SLAM
{

class FramePublisher;
class Map;
class LocalMapping;
class LoopClosing;


class Tracking
{

public:
    Tracking(ORBVocabulary* pVoc, FramePublisher* pFramePublisher, MapPublisher* pMapPublisher,
             Map* pMap, string strSettingPath, string rosTopicImageRaw, size_t numGoodInlierPredef); // double ratioGoodInlierPredef); //


    // for test only
    Tracking(const cv::Mat K, const cv::Mat DistCoef) {
        K.copyTo(mK);
        DistCoef.copyTo(mDistCoef);
    };


    ~Tracking();

    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        INITIALIZING=2,
        WORKING=3,
        LOST=4
    };

    enum eInitMethod{
        Struct_Motion=0,
        Struct_Only=1,
        ARTag=2
    };

    //    cv::Mat Rcw_init, tcw_init;

    void GrabMotionInit(const geometry_msgs::PoseStampedConstPtr & msg);
    void GrabMotionPred(const geometry_msgs::PoseStampedConstPtr & msg);

    // init file stream for writing real time tracking result
    void SetRealTimeFileStream(string fNameRealTimeTrack);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    // This is the main function of the Tracking Thread
    void Run();

    bool calculatePVelocity();

    void ForceRelocalisation();

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    Frame mCurrentFrame;

    // Initialization Variables
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    void CheckResetByPublishers();

    // Observability computation
    Observability * mObsHandler;
    //    OOMC_computer * OOMCComputor;
    arma::mat Xv_pred;
    arma::mat mCurrentInfoMat;

    std::vector<size_t> mRandLmkIdx;

    std::ofstream f_realTimeTrack;

    std::string msRosTopicImageRaw, msRosTopicInit, msRosTopicPred, msRosTopicPub;

    ros::Publisher pub_init;

    //#ifdef INIT_WITH_MOTION_PRIOR
    std::vector<std::pair<double, cv::Mat>> mvInitPoses;
    std::vector<std::pair<double, cv::Mat>> mvPredPoses;
    //#endif

    //
    // ATTENTION!
    // this is something used in old GF impl, and has been suspended since we move feature selection to TrackLocalMap
    //
    //    int NumPtsUsedLast;

    size_t num_good_inlier_predef;
    //    double ratio_good_inlier_predef;
    size_t num_good_feature_found;


    //    int mToMatchMeasurement;
    //    int mMatchedLocalMapPoint;

    //
    bool first_hit_tracking;
    double time_frame_init;
    double camera_fps;

    //    vector<LmkSelectionInfo> obs_thres_arr;
    //    vector<FramePose> mFramePoseSeq;
    //    vector<std::pair<double, int> > mFrameInlierSeq;

    void SaveTimeLog(std::string filename) {
        //
        ofstream fFrameTimeLog;
        fFrameTimeLog.open(filename.c_str());
        fFrameTimeLog << fixed;
        fFrameTimeLog << "#frame_time_stamp time_ORB_extraction time_track_motion time_track_frame time_track_map time_match ..." << std::endl;
        for(size_t i=0; i<mFrameTimeLog.size(); i++)
        {
            fFrameTimeLog << setprecision(6)
                          << mFrameTimeLog[i].frame_time_stamp << " "
                          << mFrameTimeLog[i].time_ORB_extraction << " "
                          << mFrameTimeLog[i].time_track_motion << " "
                          << mFrameTimeLog[i].time_track_frame << " "
                          << mFrameTimeLog[i].time_track_map << " "
                          << mFrameTimeLog[i].time_match << " "
                          << mFrameTimeLog[i].time_select << " "
                          << mFrameTimeLog[i].time_optim << " "
                          << mFrameTimeLog[i].time_mat_pred << " "
                          << mFrameTimeLog[i].time_mat_online << " "
                          << setprecision(0)
                          << mFrameTimeLog[i].lmk_num_refTrack << " "
                          << mFrameTimeLog[i].lmk_num_refInlier << " "
                          << mFrameTimeLog[i].lmk_num_initTrack << " "
                          << mFrameTimeLog[i].lmk_num_BA << std::endl;
        }
        fFrameTimeLog.close();
    }

    void SaveORBLog(std::string filename) {
        //
        ofstream fFrameTimeLog;
        fFrameTimeLog.open(filename.c_str());
        fFrameTimeLog << fixed;
        fFrameTimeLog << "#frame_time_stamp time_pre time_detect time_blur time_extract ..." << std::endl;
        for(size_t i=0; i<mFrameTimeLog.size(); i++)
        {
            fFrameTimeLog << setprecision(6)
                          << mFrameTimeLog[i].frame_time_stamp << " "
                          << mFrameTimeLog[i].time_pre << " "
                          << mFrameTimeLog[i].time_detect << " "
                          << mFrameTimeLog[i].time_blur << " "
                          << mFrameTimeLog[i].time_extract << std::endl;
        }
        fFrameTimeLog.close();
    }

    void SaveMatchingLog(std::string filename) {
        //
        ofstream fMatchingLog;
        fMatchingLog.open(filename.c_str());
        fMatchingLog << fixed;
        fMatchingLog << "#matchingTrials accumulatedLogDet" << std::endl;
        for(size_t i=0; i<mvMatchingLog.size(); i++)
        {
            fMatchingLog << setprecision(0)
                         << mvMatchingLog[i].match_try << " "
                         << setprecision(6)
                         << mvMatchingLog[i].accu_logDet << std::endl;
        }
        fMatchingLog.close();
    }

    //
    void BucketingMatches(const Frame *pFrame, vector<GoodPoint> & mpBucketed);
    void LongLivedMatches(const Frame *pFrame, vector<GoodPoint> & mpLongLived);
    void RanSACMatches(const Frame *pFrame, vector<GoodPoint> & mpRanSAC);


    //protected:
public:

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    //
    void GrabIMU(const sensor_msgs::ImuConstPtr & msg);

    void FirstInitialization(const eInitMethod &init_methd = eInitMethod::Struct_Motion);
    void Initialize(const eInitMethod &init_methd = eInitMethod::Struct_Motion);
    void CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw);
    void CreateInitialMap_withRT();

    void Reset();

    bool TrackPreviousFrame();
    bool TrackWithMotionModel();
    //
    bool TrackWithMotionModel_Experimental();

    bool RelocalisationRequested();
    bool Relocalisation();

    void UpdateReference();
    void UpdateReferencePoints();
    void UpdateReferenceKeyFrames();

    bool TrackLocalMap();
    bool TrackLocalMap_ICRA18();
    bool TrackLocalMap_TRO18();

    void RunMapPointsSelection(const double time_for_select, const size_t num_map_point_select);

    int SearchReferencePointsInFrustum();
    // DEBUG ONLY
    int SearchReferencePointsInFrustumWithLogging();

    void SearchAdditionalMatchesInFrame(const double time_for_search, Frame & frame_matching);

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    void PlotFrameWithPointDetected(Frame * mframe, std::string mfname);
    void PlotFrameWithPointMatches();
    //    size_t frame_counter;
    void PlotInitialMap3D(const cv::Mat & image_1, const cv::Mat & image_2);

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    //    std::vector<MapPoint*> mvpLocalAdditionalPoints;

    //Publishers
    FramePublisher* mpFramePublisher;
    MapPublisher* mpMapPublisher;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;


    // frame counter after initialization
    size_t mFrameAfterInital;

    // frame counter after track loss
    size_t mFrameLossTrack;

    size_t mbTrackLossAlert;

    //Current matches in frame
    size_t mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    // Matching log
    vector<MatchingLog> mvMatchingLog;
//    MatchingLog logCurrentMatching;

    // Time log
    vector<TimeLog> mFrameTimeLog;
    TimeLog logCurrentFrame;

    //Mutex
    boost::mutex mMutexTrack;
    boost::mutex mMutexForceRelocalisation;

    //Reset
    bool mbPublisherStopped;
    bool mbReseting;
    boost::mutex mMutexReset;

    //Is relocalisation requested by an external thread? (loop closing)
    bool mbForceRelocalisation;

    //Motion Model
    bool mbMotionModel;
    // relative motion from kinematic (constant motion model)
    cv::Mat mVelocity;
    // relative motion from IMU integration
    cv::Mat pVelocity;
    // universal, is set by m or pVelocity
    cv::Mat zVelocity;
    //
    //    gtsam::PreintegratedImuMeasurements * imu_preintegrated_;
    //    gtsam::NavState prev_state;
    //    gtsam::imuBias::ConstantBias prev_bias;
    double timeStamp_prev;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    // Transfor broadcaster (for visualization in rviz)
    tf::TransformBroadcaster mTfBr;
    // for stixel
    tf::TransformBroadcaster mTfBr_vel;

#if CV_MAJOR_VERSION == 3
    cv::viz::Viz3d * moViz3D;
    cv::Mat canvas_match_frames;
#endif


    //
    //    PoseSequence* mRefPoseSeq;
    //    cv::Mat cur_pose_ref;
    //    cv::Mat pre_pose_ref;
    //    cv::Mat pre_pose_est;
    //    float rel_scale;
    //    //
    //    ObsThreshSequence* mObsThreshSeq;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
