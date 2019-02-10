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

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include "ORBmatcher.h"
#include "Converter.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "Tracking.h"

#include "FramePublisher.h"
#include "Map.h"
#include "Initializer.h"

using namespace std;

namespace ORB_SLAM
{

Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher *pFramePublisher, MapPublisher *pMapPublisher, Map *pMap,
                   string strSettingPath, string rosTopicImageRaw, size_t numGoodInlierPredef):
    mState(NO_IMAGES_YET), mpORBVocabulary(pVoc), mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher),
    mpMap(pMap), mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false),
    mbMotionModel(false)
{

#if CV_MAJOR_VERSION == 3
    moViz3D = NULL;
#endif
    msRosTopicImageRaw = rosTopicImageRaw;

#ifdef INIT_WITH_MOTION_PRIOR
    mvInitPoses.clear();
    msRosTopicInit = "/snakey_slam_init_pose";
    msRosTopicPub = "/init_trigger";
#endif

#ifdef PRED_WITH_MOTION_PRIOR
    mvPredPoses.clear();
    msRosTopicPred = "/snakey_gait_exec_pose";
#endif

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    this->num_good_feature_found = 0;

    assert(numGoodInlierPredef > 0);
    this->num_good_inlier_predef = numGoodInlierPredef;

    // Load camera model2 parameters for observability computation
    //    NumPtsUsedLast = 1000;
    //    double C2_f = fSettings["Camera2.f"];
    //    double C2_k1= fSettings["Camera2.k1"];
    //    double C2_k2= fSettings["Camera2.k2"];
    //    double C2_cx= fSettings["Camera2.cx"];
    //    double C2_cy= fSettings["Camera2.cy"];
    //    double C2_dx= fSettings["Camera2.dx"];
    //    double C2_dy= fSettings["Camera2.dy"];
    //    double C2_nRows= fSettings["Camera2.nRows"];
    //    double C2_nCols= fSettings["Camera2.nCols"];
    //    arma::mat C2_K;
    //    C2_K << C2_f/C2_dx << 0.0 << C2_cx << arma::endr
    //         << 0.0 << C2_f/C2_dy << C2_cy << arma::endr
    //         << 0.0 << 0.0 <<  1.0 << arma::endr;

    //    cout << "- C2_f: " << C2_f << endl;
    //    cout << "- C2_dx: " << C2_dx << endl;
    //    cout << "- C2_dy: " << C2_dy << endl;
    //    cout << "- C2_k1: " << C2_k1 << endl;
    //    cout << "- C2_k2: " << C2_k2 << endl;
    //    C2_K.print("- C2_K: ");
    //    mObsHandler = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
    //                                    C2_k1, C2_k2, C2_dx, C2_dy, C2_K);
    // Since the max-vol is assessed with undistorted measurement, there is
    // no need to include distortion param here !
    //    mObsHandler = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
    //                                    0, 0, C2_dx, C2_dy, C2_K);

    // Load camera parameters from settings file
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // default distortion model: radial-tangential
    //    cv::Mat DistCoef;
    //    cv::FileNode tmpNode = fSettings["Camera.k3"];
    //    if (tmpNode.isReal()) {
    //        DistCoef = cv::Mat(5,1,CV_32F);
    //        DistCoef.at<float>(4) = fSettings["Camera.k3"];
    //        std::cout << "5-size distortion parameters being provided, with k3 = "
    //                  << DistCoef.at<float>(4) << "! " << std::endl;
    //    }
    //    else {
    //        DistCoef = cv::Mat(4,1,CV_32F);
    //        std::cout << "4-size distortion parameters being provided! " << std::endl;
    //    }
    cv::Mat DistCoef = cv::Mat(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    // additional model: equidistant
    // TODO

    camera_fps = fSettings["Camera.fps"];
    if(camera_fps==0)
        camera_fps=30;

    int nRows= fSettings["Camera2.nRows"];
    int nCols= fSettings["Camera2.nCols"];
    mObsHandler = new Observability(fx, fy, nRows, nCols, cx, cy, 0, 0);
    //    std::cout << "H13_mul_fac: " << mObsHandler->H13_mul_fac << std::endl;


    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    // mMaxFrames = camera_fps;
    mMaxFrames = 18*camera_fps/30;


    mFrameAfterInital = 0;
    mFrameLossTrack = 0;
    mbTrackLossAlert = 0;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << camera_fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fastTh = fSettings["ORBextractor.fastTh"];
    int Score = fSettings["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

#if defined ORB_SLAM_BASELINE || defined INLIER_ONLY_BASELINE
    // take the budget number from input arg as feature extraction constaint
    mpORBextractor = new ORBextractor(this->num_good_inlier_predef,fScaleFactor,nLevels,Score,fastTh);
#else
    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);
#endif
    //    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    mRandLmkIdx.clear();
    for (size_t i=0; i<nFeatures; ++i)
        mRandLmkIdx.push_back(i);
    std::random_shuffle ( mRandLmkIdx.begin(), mRandLmkIdx.end() );


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    //    mpIniORBextractor = new ORBextractor(nFeatures*2,1.2,8,Score,fastTh);
    mpIniORBextractor = new ORBextractor(2000,1.2,8,Score,fastTh);

    int nMotion = fSettings["UseMotionModel"];
    mbMotionModel = nMotion;

    if(mbMotionModel)
    {
        mVelocity = cv::Mat::eye(4,4,CV_32F);
        cout << endl << "Motion Model: Enabled" << endl << endl;
    }
    else
        cout << endl << "Motion Model: Disabled (not recommended, change settings UseMotionModel: 1)" << endl << endl;


#ifdef DISABLE_RELOC
    cout << "Tracking: relocalization disabled!" << endl;
#else
    cout << "Tracking: relocalization enabled!" << endl;
#endif


    //    mToMatchMeasurement = nFeatures;
    //    mMatchedLocalMapPoint = 0;

#ifdef USE_INFO_MATRIX
    mCurrentInfoMat.set_size(7, 7);
#elif defined USE_HYBRID_MATRIX
    mCurrentInfoMat.set_size(13, 13);
#endif
    mCurrentInfoMat.zeros();

    //    frame_counter = 0;

    //
    //    mFramePoseSeq.clear();
    //    mFrameInlierSeq.clear();

    tf::Transform tfT;
    tfT.setIdentity();
    mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORB_SLAM/World", "/ORB_SLAM/Camera"));

#if defined OBS_THRESH_SWEEP
    //
    //    if (mRefPoseSeq != NULL)
    //        delete mRefPoseSeq;
    std::string strGTPosePath = fSettings["GTPosePath"];

    cout << strGTPosePath << endl;

    mRefPoseSeq = new PoseSequence(strGTPosePath);
    rel_scale =  1.0; // / 13.0;
    first_hit_tracking = true;
    time_frame_init = -999;
#elif defined OBS_THRESH_APPLY
    std::string strObsThresPath = fSettings["ObsThresPath"];
    mObsThreshSeq = new ObsThreshSequence(strObsThresPath);
#else
    //
#endif


#ifdef WITH_IMU_PREINTEGRATION
    // initial pre intergration object
    // the following code are copied from GTSAM preintegration example; need verification
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.0003924;
    double gyro_noise_sigma = 0.000205689024915;
    double accel_bias_rw_sigma = 0.004905;
    double gyro_bias_rw_sigma = 0.000001454441043;
    gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
    gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    gtsam::imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    imu_preintegrated_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    prev_bias = prior_imu_bias;

#endif

}


Tracking::~Tracking() {
    f_realTimeTrack.close();
}

void Tracking::SetRealTimeFileStream(string fNameRealTimeTrack)
{
    f_realTimeTrack.open(fNameRealTimeTrack.c_str());
    f_realTimeTrack << fixed;
    f_realTimeTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

void Tracking::Run()
{
#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::Run()");
#endif

    ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/cam0/image_raw", 1, &Tracking::GrabImage, this);
    ros::Subscriber sub = nodeHandler.subscribe(this->msRosTopicImageRaw, 1, &Tracking::GrabImage, this);

#ifdef INIT_WITH_MOTION_PRIOR
    ros::Subscriber sub_init = nodeHandler.subscribe(this->msRosTopicInit, 1, &Tracking::GrabMotionInit, this);
    pub_init = nodeHandler.advertise<std_msgs::String>(this->msRosTopicPub, 1);
#endif

#ifdef PRED_WITH_MOTION_PRIOR
    ros::Subscriber sub_pred = nodeHandler.subscribe(this->msRosTopicPred, 1, &Tracking::GrabMotionPred, this);
#endif

#ifdef WITH_IMU_PREINTEGRATION
    //
    ros::NodeHandle nodeHandler_IMU;
    ros::Subscriber sub_IMU = nodeHandler_IMU.subscribe("/imu0", 1000, &Tracking::GrabIMU, this);
#endif

    ros::spin();
}

void Tracking::GrabMotionInit(const geometry_msgs::PoseStampedConstPtr & msg) {

    double timeStamp = msg->header.stamp.toSec();

    cv::Mat Rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);

    // translation
    twc.at<float>(0, 0) = msg->pose.position.x / 1000.0;
    twc.at<float>(1, 0) = msg->pose.position.y / 1000.0;
    twc.at<float>(2, 0) = msg->pose.position.z / 1000.0;

    // quaternion
    arma::rowvec quat;
    quat = { msg->pose.orientation.w , msg->pose.orientation.x , msg->pose.orientation.y , msg->pose.orientation.z };
    QUAT2DCM_float(quat, Rwc);

    //
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
    Tcw.rowRange(0,3).colRange(0,3) = Rwc.t();
    Tcw.rowRange(0,3).col(3) = -Rwc.t() * twc;

    mvInitPoses.push_back(std::make_pair(timeStamp, Tcw));
}

//TODO verify the conversion for pose message is correct?
void Tracking::GrabMotionPred(const geometry_msgs::PoseStampedConstPtr & msg) {

    double timeStamp = msg->header.stamp.toSec();

    cv::Mat Rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);

    // translation
    twc.at<float>(0, 0) = msg->pose.position.x / 1000.0;
    twc.at<float>(1, 0) = msg->pose.position.y / 1000.0;
    twc.at<float>(2, 0) = msg->pose.position.z / 1000.0;

    // quaternion
    arma::rowvec quat;
    quat = { msg->pose.orientation.w , msg->pose.orientation.x , msg->pose.orientation.y , msg->pose.orientation.z };
    QUAT2DCM_float(quat, Rwc);

    //
    cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
    Tcw.rowRange(0,3).colRange(0,3) = Rwc.t();
    Tcw.rowRange(0,3).col(3) = -Rwc.t() * twc;

    mvPredPoses.push_back(std::make_pair(timeStamp, Tcw));
}


void Tracking::GrabIMU(const sensor_msgs::ImuConstPtr & msg) {

#ifdef TRACKING_VERBOSE
    ROS_INFO("IMU TimeStamp: [%.06f]", msg->header.stamp.toSec());
    ROS_INFO("IMU Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
             msg->orientation.x,
             msg->orientation.y,
             msg->orientation.z,
             msg->orientation.w);
    ROS_INFO("IMU Angular Vel x: [%f], y: [%f], z: [%f]",
             msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);
    ROS_INFO("IMU Acceleration x: [%f], y: [%f], z: [%f]",
             msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
#endif
    //
    // TODO
    // parse IMU msg into acc & omega
    // integral IMU data with nonlinear 3D IMU fusion
    // save the motion estimation as motion prediction
    double timeStamp = msg->header.stamp.toSec();
    //
    // Adding the IMU preintegration.
    Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
    imu(0) = msg->linear_acceleration.x;
    imu(1) = msg->linear_acceleration.y;
    imu(2) = msg->linear_acceleration.z;
    imu(3) = msg->angular_velocity.x;
    imu(4) = msg->angular_velocity.y;
    imu(5) = msg->angular_velocity.z;
    //
    //    imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), timeStamp - timeStamp_prev);
    timeStamp_prev = timeStamp;

}

void Tracking::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    logCurrentFrame.setZero();

    arma::wall_clock timer_all;
    timer_all.tic();

#ifdef TRACKING_VERBOSE
    cout << endl;
    ROS_INFO(" ");
    ROS_INFO("Tracking::GrabImage()");
#endif
    ROS_INFO("Image TimeStamp: [%.06f]", msg->header.stamp.toSec());

    cv::Mat im;

    //---------------------------------------
    // 1. Capture image
    // Copy the ros image message to cv::Mat. Convert to grayscale if it is a color image.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        //        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_ASSERT(cv_ptr->image.channels()==3 || cv_ptr->image.channels()==1);

    arma::wall_clock timer;
    timer.tic();

    if(cv_ptr->image.channels()==3)
    {
        if(mbRGB)
            cvtColor(cv_ptr->image, im, CV_RGB2GRAY);
        else
            cvtColor(cv_ptr->image, im, CV_BGR2GRAY);
    }
    else if(cv_ptr->image.channels()==1)
    {
        cv_ptr->image.copyTo(im);
    }

    //---------------------------------------
    // 2.Create the corresponding frame according to the system status (including feature extraction)
    //
    //    enum eTrackingState{
    //    SYSTEM_NOT_READY=-1,
    //    NO_IMAGES_YET=0,
    //    NOT_INITIALIZED=1,
    //    INITIALIZING=2,
    //    WORKING=3,
    //    LOST=4
    //};
    if(mState==WORKING || mState==LOST) {
        mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpORBextractor,mpORBVocabulary,mK,mDistCoef);
    }
    else {
        mCurrentFrame = Frame(im,cv_ptr->header.stamp.toSec(),mpIniORBextractor,mpORBVocabulary,mK,mDistCoef);
    }

    double timeExtract = timer.toc();
    logCurrentFrame.time_ORB_extraction = timeExtract;
#ifdef TIMECOST_VERBOSE
    std::cout << "% ===================================================================" << std::endl;
    std::cout << "func GrabImage: time cost of ORB extraction = " << logCurrentFrame.time_ORB_extraction << std::endl;
#endif


#ifdef ORB_EXTRACTOR_TIME_LOGGING
    // update the orb extaction profiling
    logCurrentFrame.time_pre = mpORBextractor->time_pre;
    logCurrentFrame.time_detect = mpORBextractor->time_detect;
    logCurrentFrame.time_detect = mpORBextractor->time_detect;
    logCurrentFrame.time_extract = mpORBextractor->time_extract;
#endif

    ///---------------------------------------
    //  3. perform tracking
    //
    // Depending on the state of the Tracker we perform different tasks
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED)
    {
#ifdef INIT_WITH_MOTION_PRIOR
        FirstInitialization(eInitMethod::Struct_Only);
#else
        FirstInitialization(eInitMethod::Struct_Motion);
#endif
    }
    else if(mState==INITIALIZING)
    {
#ifdef INIT_WITH_MOTION_PRIOR
        Initialize(eInitMethod::Struct_Only);
#else
        Initialize(eInitMethod::Struct_Motion);
#endif
    }
    else   // main process
    {
        //        std::cerr << "% ===================================================================" << std::endl;
        //        std::cerr << "k = k +1;" << std::endl;

        mFrameAfterInital ++;

        // set up the time log struct
        logCurrentFrame.frame_time_stamp = mCurrentFrame.mTimeStamp;

        // System is initialized. Track Frame.
        bool bOK;

        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if(mState==WORKING && !RelocalisationRequested()) {
#ifdef WITH_IMU_PREINTEGRATION
            // replace constant rel motion with imu integration
            //
            gtsam::NavState prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
            // note that the prediction is for IMU; an additional transformation is needed to obtain pose of camera
            prop_state.t();
            prop_state.R();

            //            if (usePvel && calculatePVelocity() == true) {
            //                //replacing mVelocity with pVelocity, using IMU information
            //                zVelocity = pVelocity;
            //            }
            //            else {
            //                zVelocity = mVelocity;
            //            } //<<<<<<<<<<<....>>>>>>>>>>>>>>>>
#endif
            //
            if(!mbMotionModel || mpMap->KeyFramesInMap()<4 || mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2) {
                timer.tic();
                bOK = TrackPreviousFrame();
                logCurrentFrame.time_track_frame = timer.toc();
            }
            else
            {
                timer.tic();
                bOK = TrackWithMotionModel();
                logCurrentFrame.time_track_motion = timer.toc();
                if(!bOK) {  // Fall back
                    timer.tic();
                    bOK = TrackPreviousFrame();
                    logCurrentFrame.time_track_frame = timer.toc();
                }
            }
            //
            if (!bOK)
                std::cout << "Track Lost When Estimating Initial Pose!" << std::endl;
        }
        else //If lost track, or at initialization, force to relocalize
        {
#ifdef DISABLE_RELOC
            // do nothing
#else
            bOK = Relocalisation();
#endif
            //
            if (!bOK) {
                mFrameLossTrack ++;
                std::cout << "Track Lost When Relocalizing!" << std::endl;
            }
            else {
                // reset the frame counter
                mFrameLossTrack = 0;
            }

            // terminate the whole pipeline if certain amount of frames being loss track
            if (mFrameLossTrack > MAX_FRAME_LOSS_DURATION * camera_fps) {
                std::cout << "Too many frames loss track, terminate the SLAM pipeline!" << std::endl;
                ros::shutdown();
            }
        }


        // VIZ only
        //        PlotFrameWithPointMatches();


        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK) {
            timer.tic();
            //#if defined ORB_SLAM_BASELINE || defined GOOD_FEATURE_MAP_MATCHING
            // call original ORB SLAM map tracking
            bOK = TrackLocalMap();
            //#else
            //            // NOTE
            //            // instead of calling subset tracking method right away
            //            // utilizing all features at the initial stage (e.g. begining 10 seconds)
            //            // and turn subset selection on after that
            //            if (mFrameAfterInital < camera_fps * TIME_INIT_TRACKING ||
            //                    mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames) {
            //                // call original ORB SLAM map tracking
            //                bOK = TrackLocalMap_Orig_ORB();
            //            }
            //            else {
            //                // call subset-selected map tracking
            //                bOK = TrackLocalMap_TRO18();
            //            }
            //#endif
            logCurrentFrame.time_track_map = timer.toc();
            if (!bOK)
                std::cout << "Track Lost When Refining Pose!" << std::endl;
        }


#ifdef LOGGING_MATCHING

        string fname_matching_log = "/mnt/DATA/tmp/ScreenShot/" + std::to_string(long(mCurrentFrame.mTimeStamp * pow(10, 9))) + "_match.log";
        SaveMatchingLog(fname_matching_log);
#endif

        // VIZ only
        //        if (mCurrentFrame.mnId % 20 == 0)
        PlotFrameWithPointMatches();

        //        // If tracking were good, check if we insert a keyframe
        //        if(bOK)
        //        {
        //            timer.tic();

        //            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

        //            if(NeedNewKeyFrame())
        //                CreateNewKeyFrame();

        //            // We allow points with high innovation (considererd outliers by the Huber Function)
        //            // pass to the new keyframe, so that bundle adjustment will finally decide
        //            // if they are outliers or not. We don't want next frame to estimate its position
        //            // with those points so we discard them in the frame.
        //            for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
        //            {
        //                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        //                    mCurrentFrame.mvpMapPoints[i]=NULL;
        //            }

        //            logCurrentFrame.time_create_kf = timer.toc();
        //        }
        if(bOK)
            mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }

        // Update motion model
        if(mbMotionModel)
        {
            timer.tic();

            if(bOK && !mLastFrame.mTcw.empty())
            {
                cv::Mat LastRwc = mLastFrame.mTcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat Lasttwc = -LastRwc*mLastFrame.mTcw.rowRange(0,3).col(3);
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                LastRwc.copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                Lasttwc.copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            }
            else
                mVelocity = cv::Mat();

#ifdef PRED_WITH_MOTION_PRIOR
            // in the presense of motion prior, update the const prediction with motion prior
            // check the timestamp of lastest prior
            if (!mvPredPoses.empty()) {
                cout << mvPredPoses.size() << endl;
                cout << fabs(mvPredPoses.back().first - mCurrentFrame.mTimeStamp) << endl;
                if (fabs(mvPredPoses.back().first + PRED_HORIZON - mCurrentFrame.mTimeStamp) < TIME_STAMP_MATCH_THRESH) {
                    cout << "predicted with constant motion: " << endl << mVelocity << endl;
                    mvPredPoses.back().second.copyTo(mVelocity);
                    cout << "predicted with motion prior: " << endl << mVelocity << endl;
                }
            }
#endif

            // For Stixel, the relative transform between last and current frame is published
            cv::Mat R21 = mVelocity.rowRange(0,3).colRange(0,3).t();
            cv::Mat t21 = -R21*mVelocity.rowRange(0,3).col(3);
            tf::Matrix3x3 M(R21.at<float>(0,0),R21.at<float>(0,1),R21.at<float>(0,2),
                            R21.at<float>(1,0),R21.at<float>(1,1),R21.at<float>(1,2),
                            R21.at<float>(2,0),R21.at<float>(2,1),R21.at<float>(2,2));
            tf::Vector3 V(t21.at<float>(0), t21.at<float>(1), t21.at<float>(2));
            tf::Transform tfT21(M,V);
            //
            mTfBr_vel.sendTransform(tf::StampedTransform(tfT21, msg->header.stamp, "ORB_SLAM/Camera", "ORB_SLAM/CameraLast"));


#ifdef WITH_IMU_PREINTEGRATION
            // Overwrite the beginning of the preintegration for the next step.
            // Similarly, an additional transformation is needed to obtain the pose of IMU from the pose of camera
            Matrix3d rotM_eigen;
            cv::cv2eigen( mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3), rotM_eigen );

            prev_state = gtsam::NavState(
                        gtsam::Rot3( rotM_eigen ),
                        gtsam::Point3( mCurrentFrame.mTcw.at<float>(0, 3),
                                       mCurrentFrame.mTcw.at<float>(1, 3),
                                       mCurrentFrame.mTcw.at<float>(2, 3) ),
                        gtsam::Velocity3( mVelocity.at<float>(0, 3),
                                          mVelocity.at<float>(1, 3),
                                          mVelocity.at<float>(2, 3) )
                        );

            // Reset the preintegration object.
            imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
#endif

            logCurrentFrame.time_update_motion = timer.toc();
        }

#if defined GOOD_FEATURE_MAP_BOUND || defined GOOD_FEATURE_MAP_MATCHING
        // predict the pose at next frame
        if (bOK) {
            mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                       mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());
            mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 2 );
        }
#endif

        // Since we search for additional matches after publishing the camera state,
        // the creation of last frame should be postponed after the additional matching.
        //        mLastFrame = Frame(mCurrentFrame);
    }

    // Update drawer
    mpFramePublisher->Update(this);

    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Rwc = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*mCurrentFrame.mTcw.rowRange(0,3).col(3);
        tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                        Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                        Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
        tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

        tf::Transform tfTcw(M,V);

        //
        mTfBr.sendTransform(tf::StampedTransform(tfTcw, msg->header.stamp, "ORB_SLAM/World", "ORB_SLAM/Camera"));


        // Save tracking result into vector
        //        mFramePoseSeq.push_back(FramePose(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw));

        // Write the real time tracking result to file
        FramePose poseTmp = FramePose(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw);

        double timestamp = poseTmp.time_stamp;
        cv::Mat Homm = poseTmp.homm;
        cv::Mat R = Homm.rowRange(0,3).colRange(0,3).t();
        cv::Mat t = - R * Homm.rowRange(0,3).col(3);

        //
        // TODO
        // check the output R & T are identical with the ones send to visualizer
        //
        //        std::cout << twc - t << std::endl;
        //        std::cout << Rwc - R << std::endl;

        vector<float> q = ORB_SLAM::Converter::toQuaternion(R);

        f_realTimeTrack << setprecision(6) << timestamp << setprecision(7)
                        << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                        << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    timer.tic();

    if (mState == WORKING) {

#if defined DELAYED_MAP_MATCHING

        //        UpdateReferenceKeyFrames();
#ifdef LOGGING_MATCHING
        double timeCost_sofar = timer_all.toc(), timeCost_rest = 1.0 / (0.2*camera_fps) - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        std::cout << "========== total time to proc 1 frame = " << 1.0 / (0.2*camera_fps)
                  << "; already taken " << timeCost_sofar
                  << "; left " << timeCost_rest << " ==========" << std::endl;
#else
        double timeCost_sofar = timer_all.toc(), timeCost_rest = 1.0 / (0.5 * camera_fps) - timeCost_sofar - 0.002;
        // Compute the good feature for local Map
        std::cout << "========== total time to proc 1 frame = " << 1.0 / (0.5 * camera_fps)
                  << "; already taken " << timeCost_sofar
                  << "; left " << timeCost_rest << " ==========" << std::endl;
#endif

        // predict info matrix for visible local points
        mObsHandler->mnFrameId = mCurrentFrame.nNextId;

        mObsHandler->mBoundXInFrame = (ORB_SLAM::Frame::mnMaxX - ORB_SLAM::Frame::mnMinX) * 0.1; // 20;
        mObsHandler->mBoundYInFrame = (ORB_SLAM::Frame::mnMaxY - ORB_SLAM::Frame::mnMinY) * 0.1; // 20;
        mObsHandler->mBoundDepth = 0;

        // No selection, only predict infor matrix at next frame
        //        RunMapPointsSelection(timeCost_rest, 0);

#if defined GOOD_FEATURE_MAP_MATCHING
        std::thread thread_Select(&Tracking::RunMapPointsSelection, this, timeCost_rest, 0);
#endif
        
        SearchAdditionalMatchesInFrame(timeCost_rest, mCurrentFrame);

#if defined GOOD_FEATURE_MAP_MATCHING
        thread_Select.join();
#endif

#endif
        // If tracking were good, check if we insert a keyframe
        //        timer.tic();
        if(NeedNewKeyFrame())
            CreateNewKeyFrame();

        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for(size_t i=0; i<mCurrentFrame.mvbOutlier.size();i++)
        {
            if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                mCurrentFrame.mvpMapPoints[i]=NULL;
        }
        //        logCurrentFrame.time_create_kf = timer.toc();

        mLastFrame = Frame(mCurrentFrame);
    }

    logCurrentFrame.time_mat_pred = timer.toc();

    // push the time log of current frame into the vector
    mFrameTimeLog.push_back(logCurrentFrame);
}


void Tracking::FirstInitialization(const eInitMethod &init_methd)
{
#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::FirstInitialization()");
#endif

    //We ensure a minimum ORB features to continue, otherwise discard frame
    //    if(mCurrentFrame.mvKeys.size()>100)
    if (init_methd == eInitMethod::Struct_Motion) {
        if(mCurrentFrame.mvKeys.size()>THRES_INIT_MPT_NUM)
        {
            std::cout << "func Tracking::FirstInitialization: trying to first initialize!" << std::endl;

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            mState = INITIALIZING;
        }
    }
    else if (init_methd == eInitMethod::Struct_Only) {
        //
        // NOTE
        // assuming the pose are published sequentially, which is identical to how images are published
        if(mCurrentFrame.mvKeys.size()>THRES_INIT_MPT_NUM)
        {
            int vld_idx = 0;
            for (vld_idx=0; vld_idx<mvInitPoses.size(); ++vld_idx) {
                if (fabs(mCurrentFrame.mTimeStamp - mvInitPoses[vld_idx].first) < TIME_STAMP_MATCH_THRESH)
                    break ;
            }
            if (vld_idx < mvInitPoses.size()) {
                std::cout << "func Tracking::FirstInitialization: trying to first initialize!" << std::endl;

                mInitialFrame = Frame(mCurrentFrame);
                mLastFrame = Frame(mCurrentFrame);
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

                if(mpInitializer)
                    delete mpInitializer;

                mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

                mvInitPoses[vld_idx].second.copyTo(mInitialFrame.mTcw);

                mState = INITIALIZING;
            }
            else {
                cout << "func FirstInitialization: first frame has no pose prior! " << endl;
            }
        }
        else {
            cout << "func FirstInitialization: first frame has few texture to extract features! " << endl;
        }
    }

}


void Tracking::Initialize(const eInitMethod &init_methd)
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::Initialize()");
#endif

    if (init_methd == eInitMethod::Struct_Motion) {
        // default init of orb-slam
        // Check if current frame has enough keypoints, otherwise reset initialization process
        //    if(mCurrentFrame.mvKeys.size()<=100)
        if(mCurrentFrame.mvKeys.size()<=THRES_INIT_MPT_NUM)
        {
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,SRH_WINDOW_SIZE_INIT);

        std::cout << "func Tracking::Initialize: trying to initialize with " << nmatches << " matches!" << std::endl;

        // Check if there are enough correspondences
        if(nmatches<THRES_INIT_MPT_NUM)
        {
            mState = NOT_INITIALIZED;
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            CreateInitialMap(Rcw,tcw);

            //            // DEBUG
            //            PlotFrameWithPointDetected(&mInitialFrame, std::string("init_") + std::to_string(mInitialFrame.mTimeStamp));
            //            PlotFrameWithPointDetected(&mCurrentFrame, std::string("curr_") + std::to_string(mCurrentFrame.mTimeStamp));
        }
    }
    else if (init_methd == eInitMethod::Struct_Only) {
        // init with motion prior, e.g. from controller
        cout << "=============== INITIALIZATION BEGIN ===============" << endl;
        //
        if(mCurrentFrame.mvKeys.size()<=THRES_INIT_MPT_NUM) {
            //
            cout << "func Initialize: second frame has few texture to extract features! " << endl;
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }

        // query for the top index
        int vld_idx;
        for (vld_idx=mvInitPoses.size()-1; vld_idx>=0; --vld_idx) {
            if (fabs(mCurrentFrame.mTimeStamp - mvInitPoses[vld_idx].first) < TIME_STAMP_MATCH_THRESH)
                break ;
        }
        if (vld_idx < 0) {
            // no valid pose received to fix motion
            // give up and wait till new image come in with pose
            cout << "func Initialize: second frame is not paired with pose prior! wait for next frame! " << endl;
            mState = INITIALIZING;
            //            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            //            mState = NOT_INITIALIZED;
            return;
        }

        std::cout << "func Tracking::Initialize: pass the checking for init pair!" << std::endl;

        mvInitPoses[vld_idx].second.copyTo(mCurrentFrame.mTcw);

        //            cv::Mat T21 = mInitialFrame.mTcw.inv() * mCurrentFrame.mTcw;
        cv::Mat T21 = mCurrentFrame.mTcw * mInitialFrame.mTcw.inv();

        std::cout << "frame 1: " << mInitialFrame.mTcw << std::endl;
        std::cout << "frame 2: " << mCurrentFrame.mTcw << std::endl;

        cv::Mat R21 = T21.rowRange(0,3).colRange(0,3);
        cv::Mat t21 = T21.rowRange(0,3).col(3);
        //        if (Rcw_init.rows != 3 || Rcw_init.cols != 3 || tcw_init.rows != 3 || tcw_init.cols != 1) {
        //            std::cout << "func Tracking::Initialize: R & T are not initialized with prior!" << std::endl;
        //            mState = NOT_INITIALIZED;
        //            return ;
        //        }
        if (cv::norm(t21) < 0.05) {
            std::cout << "func Tracking::Initialize: baseline too small " << cv::norm(t21) << "! wait for next frame!" << std::endl;
            mState = INITIALIZING;
            //            mState = NOT_INITIALIZED;
            return ;
        }

        //        PlotFrameWithPointDetected();

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,SRH_WINDOW_SIZE_INIT);

        std::cout << "func Tracking::Initialize: trying to initialize with " << nmatches << " matches!" << std::endl;

        // Check if there are enough correspondences
        if(nmatches<THRES_INIT_MPT_NUM)
        {
            std::cout << "func Tracking::Initialize: too few matching between 2 frames! wait for next frame!" << std::endl;
            mState = INITIALIZING;
            //            mState = NOT_INITIALIZED;
            return;
        }


        std::cout << "func Tracking::Initialize: Init R & t "
                  << R21 << " and " << t21 << std::endl;
        double t21_norm = cv::norm(t21);


        // DEBUG
        //        PlotFrameWithPointDetected(&mInitialFrame, std::string("init_") + std::to_string(mInitialFrame.mTimeStamp));
        //        PlotFrameWithPointDetected(&mCurrentFrame, std::string("curr_") + std::to_string(mCurrentFrame.mTimeStamp));


        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        //       if(mpInitializer->Initialize_withRT(mCurrentFrame, mvIniMatches, R21, t21, mvIniP3D, vbTriangulated) > 0)
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, R21, t21, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            std::cout << "func Tracking::Initialize: try creating init map with "
                      << nmatches << " points!" << std::endl;

            // re-scale the translation component with motion prior
            t21 = t21 / cv::norm(t21) * t21_norm;

            CreateInitialMap(R21, t21);

            //
            std::cout << "func Tracking::Initialize: done. Init map created with "
                      << mvpLocalMapPoints.size() << " points!" << std::endl;

            std::cout << "func Tracking::Initialize: Final R & t "
                      << R21 << " and " << t21 << std::endl;

#ifdef INIT_WITH_MOTION_PRIOR
            // publish message to finish initial for snake
            std_msgs::String msg;

            std::stringstream ss;
            ss << "initialized";
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());
            pub_init.publish(msg);
#endif
        }
        else {
            //
            std::cout << "func Tracking::Initialize: failed to solve structure with matched feature set! wait for next frame!" << std::endl;
            mState = INITIALIZING;
            //            mState = NOT_INITIALIZED;
            return;
        }
        //
        //        cv::Mat T12 = T21.inv();
        //        cv::Mat R12 = T12.rowRange(0,3).colRange(0,3);
        //        cv::Mat t12 = T12.rowRange(0,3).col(3);
        //        if(mpInitializer->Initialize_withRT(mCurrentFrame, mvIniMatches, R12, t12, mvIniP3D, vbTriangulated) > 0)
        //        {
        //            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        //            {
        //                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
        //                {
        //                    mvIniMatches[i]=-1;
        //                    nmatches--;
        //                }
        //            }

        //            std::cout << "func Tracking::Initialize: try creating init map with "
        //                      << nmatches << " points!" << std::endl;

        //            CreateInitialMap(R12, t12);
        //        }
        //        std::cout << "func Tracking::Initialize: done. Init map created with "
        //                  << mvpLocalMapPoints.size() << " points!" << std::endl;
    }
    else {
        // init with AR Tag
        // TODO
    }
}


void Tracking::CreateInitialMap(cv::Mat &Rcw, cv::Mat &tcw)
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::CreateInitialMap()");
#endif

    // Set Frame Poses
    mInitialFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    mCurrentFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));

    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }

    //    ROS_INFO("New Map created with %d points", mpMap->MapPointsInMap());

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    //    ROS_INFO("Connection updated");

    // Bundle Adjustment
#ifdef TRACKING_VERBOSE
    ROS_INFO("New Map created with %d points", mpMap->MapPointsInMap());
#endif
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    //    ROS_INFO("GlobalBundleAdjustemnt");

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    if(medianDepth<0 || pKFcur->TrackedMapPoints()<THRES_INIT_MPT_NUM)
    {

        //#ifdef TRACKING_VERBOSE
        ROS_INFO("Wrong initialization, reseting...");
        //        cout << "Wrong initialization, reseting..." << endl;
        //#endif
        Reset();
        return;
    }

#ifndef INIT_WITH_MOTION_PRIOR
    //
    float invMedianDepth = 1.0f/medianDepth;
    ROS_INFO("Scaling new map with %f inv median depth", invMedianDepth);

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }
#endif

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    //    ROS_INFO("InsertKeyFrame");

    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

    mState=WORKING;
}


bool Tracking::TrackPreviousFrame()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackPreviousFrame()");
#endif
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);

    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
        }
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(&mCurrentFrame);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);

    logCurrentFrame.lmk_num_initTrack = nmatches;

    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    //    mToMatchMeasurement = mCurrentFrame.N - nmatches;

    if(nmatches<10)
        return false;

    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }


    logCurrentFrame.lmk_num_initTrack = nmatches;
    //    mToMatchMeasurement = mCurrentFrame.N - nmatches;

    return nmatches>=10;
}


bool Tracking::TrackWithMotionModel_Experimental() {

    arma::wall_clock timer;
    timer.tic();

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackWithMotionModel()");
#endif

    ORBmatcher matcher(0.9,true);

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity * mLastFrame.mTcw;

#ifdef TRACKING_VERBOSE
    std::cout << "mLastFrame.mTcw: " << mLastFrame.mTcw << std::endl;
    std::cout << "mVelocity is :" << mVelocity << std::endl;
    std::cout << "mCurrentFrame.mTcw  is: " << mCurrentFrame.mTcw  << std::endl;
#endif

    // Deal with Map Points
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

#ifdef ROBUST_GF_MATCHING

    int nmatches;
    if (this->num_good_feature_found > MIN_NUM_GOOD_FEATURES * 3) {
        //    if (this->num_good_feature > MIN_NUM_GOOD_FEATURES * 2) {
        float th = 15;
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th);
    }
    else if (this->num_good_feature_found >= MIN_NUM_GOOD_FEATURES * 1.5) {
        //    else if (this->num_good_feature > MIN_NUM_GOOD_FEATURES) {
        float th_gf = 25; // 35; // 45; //
        float th_rt = 15; // 25; // 35; //
        nmatches = matcher.SearchByProjection_GoodFeature(mCurrentFrame, mLastFrame, th_gf, th_rt);
    }
    else {
        // only perform increased-radius search when GF number running low / likely to lose track
        float th_gf = 55; // 65; // 45; // 55; //
        float th_rt = 45; // 55; //30; // 45; //
        nmatches = matcher.SearchByProjection_GoodFeature(mCurrentFrame, mLastFrame, th_gf, th_rt);
    }

#ifdef LMKNUM_VERBOSE
    std::cout << "func TrackWithMotionModel: num of matches found = " << nmatches << std::endl;
#endif

#else

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 15);

#endif

    double timeMatchSrh = timer.toc();

    if(nmatches<20)
        return false;

#ifdef TRACKING_VERBOSE
    ROS_INFO(">>> Number of matches: %d", nmatches);
#endif

    timer.tic();

    Optimizer::PoseOptimization(&mCurrentFrame);

    double timeInitOpt = timer.toc();

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }

    //
    //    std::cerr << "timeMatchSrh(k) = " << timeMatchSrh << ";" << std::endl;
    //    std::cerr << "timeInitOpt(k) = " << timeInitOpt << ";" << std::endl;
    //    std::cerr << "timeTrackMotionModel(k) = " << timeMatchSrh + timeInitOpt << ";" << std::endl;

#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackWithMotionModel: time used = " << timeMatchSrh + timeInitOpt << std::endl;
#endif

#ifdef TRACKING_VERBOSE
    ROS_INFO(">>> Number of inliers: %d", nmatches);
#endif
    return nmatches>=10;
}

bool Tracking::TrackWithMotionModel() {

    //    arma::wall_clock timer;
    //    timer.tic();
    //    double time_Search = 0, time_Pool = 0, time_Select = 0, time_Opt = 0, time_Post = 0;
    //    std::vector<GoodPoint> mPtObs;

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackWithMotionModel()");
#endif
    ORBmatcher matcher(0.9,true);

    // Compute current pose by motion model
    mCurrentFrame.mTcw = mVelocity * mLastFrame.mTcw;

#ifdef TRACKING_VERBOSE
    std::cout << "mLastFrame.mTcw: " << mLastFrame.mTcw << std::endl;
    std::cout << "mVelocity is :" << mVelocity << std::endl;
    std::cout << "mCurrentFrame.mTcw  is: " << mCurrentFrame.mTcw  << std::endl;
#endif

    // Deal with Map Points
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

#ifdef ROBUST_LOCAL_SEARCHING

    int nmatches = 0;

    if (mbTrackLossAlert == 1) {
        std::cout << "func TrackWithMotionModel: increase searching range to avoid track loss !!!" << std::endl;
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 45);
    }
    else {
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 15);
    }

#else

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 15);
    logCurrentFrame.lmk_num_initTrack = nmatches;

#endif

    //    mToMatchMeasurement = mCurrentFrame.N - nmatches;

    if (nmatches < 30) {
        std::cout << "func TrackWithMotionModel: set mbTrackLossAlert to 2nd level !!!" << std::endl;
        mbTrackLossAlert = 2;
    }

    //    time_Search = timer.toc();

    if(nmatches<20) {

        std::cout << "func TrackWithMotionModel: nmatches = " << nmatches << std::endl;
        //        logCurrentFrame.time_track_motion = time_Search;
        return false;

    }

    //    timer.tic();
    //    //
    //    mCurrentFrame.mvbCandidate = vector<bool>(mCurrentFrame.N, true);

    //#if defined RANSAC_POOL_FOR_SUBSET || defined INLIER_ONLY_BASELINE

    //    this->RanSACMatches(&mCurrentFrame, mPtObs);
    //    mCurrentFrame.mvbCandidate.assign(mCurrentFrame.N, false);

    //    for(size_t i= 0; i<mPtObs.size(); i++)  {
    //        mCurrentFrame.mvbCandidate[mPtObs[i].idx] = true;
    //    }

    //#endif

    //    time_Pool = timer.toc();
    //    std::cout << "number of selected inlier to init the pose: " << mPtObs.size() << std::endl;

    //    // Pose optimization
    //    timer.tic();
    //    if (this->num_good_feature_found > MIN_NUM_GOOD_FEATURES ) {
    //        Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    //    }
    //    else {
    //        Optimizer::PoseOptimization(&mCurrentFrame);
    //    }
    //    time_Opt = timer.toc();

    //    timer.tic();
    Optimizer::PoseOptimization(&mCurrentFrame);
    //    time_Opt = timer.toc();

    // Discard outliers
    //    timer.tic();
    for(size_t i =0; i<mCurrentFrame.mvpMapPoints.size(); i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }
    //    time_Post = timer.toc();

#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackWithMotionModel: time of searching matches = "  << time_Search << std::endl;
    std::cout << "func TrackWithMotionModel: time of optimization = "       << time_Opt << std::endl;
    std::cout << "func TrackWithMotionModel: time of post proc = "          << time_Post << std::endl;
#endif
    //
    //    std::cout << "func TrackWithMotionModel: time used = " << time_Search + time_Pool + time_Select + time_Opt + time_Post << std::endl;

    //    // print out info for future analysis
    //    // report absolute feature numbers
    //    std::cerr << "lmkNum(k) = "             << mCurrentFrame.mvpMapPoints.size()    << ";" << std::endl;
    //    std::cerr << "inlierNum(k) = "          << nmatches                             << ";" << std::endl;
    //    // report time
    //    std::cerr << "timeTrackMotionModel(k) = "   << time_Search + time_Opt + time_Post << ";" << std::endl;
    //    std::cerr << "timeMatchSrh(k) = "           << time_Search      << ";"                    << std::endl;
    //    std::cerr << "timeOpt(k) = "                << time_Opt         << ";"                    << std::endl;
    //    std::cerr << "timePost(k) = "               << time_Post        << ";"                    << std::endl;

    //    logCurrentFrame.time_track_motion = time_Search + time_Pool + time_Select + time_Opt + time_Post;

    logCurrentFrame.lmk_num_initTrack = nmatches;
    //    mToMatchMeasurement = mCurrentFrame.N - nmatches;

#ifdef TRACKING_VERBOSE
    ROS_INFO(">>> Number of inliers: %d", nmatches);
#endif
    return nmatches>=10;
}


//bool Tracking::calculatePVelocity()
//{

//    if (!((mVelocity.cols == 4) && (mVelocity.rows == 4))) {
//        ROS_INFO("mVelocity not initialized");
//        return false;
//    }

//    pVelocity = mVelocity.clone();

//    //cv::Mat empty;

//    //ROS_INFO("Calculating pVelocity");
//    //////// getting IMU information //////
//    geometry_msgs::TransformStamped transformStamped;
//    try{
//        /*
//        transformStamped = tfBuffer.lookupTransform("fcu_optical", ros::Time(mCurrentFrame.mTimeStamp), "fcu_optical",
//                                ros::Time(mLastFrame.mTimeStamp), "fcu_ot", ros::Duration(.01));
//        transformStamped = tfBuffer.lookupTransform("fcu_optical", ros::Time(mCurrentFrame.mTimeStamp), "fcu_optical",
//                                ros::Time(mLastFrame.mTimeStamp), "local", ros::Duration(.01));
//        */
//        if (useVicon) {
//            transformStamped = tfBuffer.lookupTransform("vicon_optical", ros::Time(mCurrentFrame.mTimeStamp), "vicon_optical",
//                                                        ros::Time(mLastFrame.mTimeStamp), "local", ros::Duration(.01));
//        }
//        else {
//            transformStamped = tfBuffer.lookupTransform("fcu_optical", ros::Time(mCurrentFrame.mTimeStamp), "fcu_optical",
//                                                        ros::Time(mLastFrame.mTimeStamp), "local", ros::Duration(.01));
//        }

//    }
//    catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ROS_INFO("Transform Exception!");
//        ros::Duration(1.0).sleep();
//        //return false; //this could be an issue
//        return false;
//    }

//    //converting from geometry quaternion --> tf2 quaternion
//    tf2::Quaternion Q;
//    tf2::fromMsg(transformStamped.transform.rotation, Q);


//    tf2::Matrix3x3 RotMatrix(Q); //converting from tf2 Quaternion --> tf2 Rotation Matrix

//    //const double &var1 = RotMatrix.getRow(0).getX(); //getting info

//    //converting from 3x3 TF to cv::Mat


//    pVelocity.at<float>(0,0) = RotMatrix.getRow(0).getX(); //replacing Rotation Matrix in pVelocity with IMU Rot Matrix
//    pVelocity.at<float>(0,1) = RotMatrix.getRow(0).getY();
//    pVelocity.at<float>(0,2) = RotMatrix.getRow(0).getZ();
//    pVelocity.at<float>(1,0) = RotMatrix.getRow(1).getX();
//    pVelocity.at<float>(1,1) = RotMatrix.getRow(1).getY();
//    pVelocity.at<float>(1,2) = RotMatrix.getRow(1).getZ();
//    pVelocity.at<float>(2,0) = RotMatrix.getRow(2).getX();
//    pVelocity.at<float>(2,1) = RotMatrix.getRow(2).getY();
//    pVelocity.at<float>(2,2) = RotMatrix.getRow(2).getZ();

//    return true;
//}




// TODO
// TODO
// TODO
// inject max vol to label the good map points from the rest
void Tracking::RunMapPointsSelection(const double time_for_select, const size_t num_map_point_select) {

#ifdef GOOD_FEATURE_MAP_BOUND
    if (mObsHandler->kinematic.size() < 2)
    {
        std::cout << "invalid motion prediction!" << std::endl;
        return ;
    }
#endif

    if (time_for_select <= 0)
    {
        std::cout << "too little budget available!" << std::endl;
        return ;
    }

    arma::wall_clock timer;
    timer.tic();

    // find visible map points
    std::vector<MapPoint*> tmpMapPoints;
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnUsedForLocalMap==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad()) {
                //                cv::Mat Pw = pMP->GetWorldPos();
                //                if ( mObsHandler->visible_Point_To_Frame(Pw, mObsHandler->kinematic[1].Tcw) == true ) {
                //                    tmpMapPoints.push_back(pMP);
                //                }
                pMP->mnUsedForLocalMap=mCurrentFrame.mnId;
                tmpMapPoints.push_back(pMP);
            }
        }
    }
    //    std::cout << "func RunMapPointsSelection: number of local map points before visibility check = " << tmpMapPoints.size() << std::endl;

    // select map points with low score
    //    size_t num_map_point_selected = 4000; // 2000; //

    if (num_map_point_select >= tmpMapPoints.size()) {
        // simply set all map points as good
        for (size_t i=0; i<tmpMapPoints.size(); ++i) {
            tmpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId;
        }
    }
    else {
        double time_used = timer.toc();

#ifdef GOOD_FEATURE_MAP_MATCHING

        mObsHandler->mKineIdx = 1;
        mObsHandler->mMapPoints = &tmpMapPoints;
#ifdef USE_INFO_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_INFO_MATRIX, time_for_select - time_used, USE_MULTI_THREAD, true);
#elif defined USE_HYBRID_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_HYBRID_MATRIX, time_for_select - time_used, USE_MULTI_THREAD, true);
#elif defined USE_OBSERVABILITY_MATRIX
        // TODO
#endif
        //        std::cout << "func RunMapPointsSelection: finish updating info matrix! " << std::endl;

#elif defined GOOD_FEATURE_MAP_BOUND
        vector<GoodPoint> mPtObs;
        mObsHandler->lmkSelectPool.clear();
        //        std::cout << "before calling setSelction_Number " << std::endl;
        bool flagSucc = mObsHandler->setSelction_Number(num_map_point_select, 3, time_for_select - time_used, &tmpMapPoints, &mPtObs);
        //        std::cout << "after calling setSelction_Number " << std::endl;
        if (flagSucc) {
            // set flag for good map points
            for (size_t i=0; i<mPtObs.size(); ++i) {
                tmpMapPoints[mPtObs[i].idx]->goodAtFrameId = mCurrentFrame.nNextId;
            }
            //    std::cout << "number of good map points found: " << mPtObs.size() << std::endl;
        }
        else {
            // TODO
            // take whatever the maxvol grabed
            for (size_t i=0; i<mPtObs.size(); ++i) {
                tmpMapPoints[mPtObs[i].idx]->goodAtFrameId = mCurrentFrame.nNextId;
            }

            // Random shuffle the inliers
            std::random_shuffle ( tmpMapPoints.begin(), tmpMapPoints.end() );

            // fill in the rest with random map points
            if (mPtObs.size() < num_map_point_select) {
                // set flag for good map points
                size_t j = mPtObs.size(), i = 0;
                while (j < num_map_point_select && i < tmpMapPoints.size()) {
                    if (tmpMapPoints[i]->goodAtFrameId != mCurrentFrame.nNextId) {
                        //
                        tmpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId;
                        ++ j;
                    }
                    ++ i;
                }

                std::cout << "func RunMapPointsSelection: select " << j - mPtObs.size() << " random enhancement to good map!" << std::endl;
            }
            else if (mPtObs.size() > num_map_point_select) {
                // set flag for good map points
                size_t j = mPtObs.size(), i = 0;
                while (j > num_map_point_select && i < tmpMapPoints.size()) {
                    if (tmpMapPoints[i]->goodAtFrameId == mCurrentFrame.nNextId) {
                        //
                        tmpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId - 1;
                        -- j;
                    }
                    ++ i;
                }

                std::cout << "func RunMapPointsSelection: delete " << mPtObs.size() - j << " random enhancement to good map!" << std::endl;
            }
        }
#elif defined RANDOM_MAP_BOUND

        vector<GoodPoint> mPtObs;
        mObsHandler->lmkSelectPool.clear();
        mObsHandler->setSelction_Number(tmpMapPoints.size(), 3, time_for_select - time_used, &tmpMapPoints, &mPtObs);

        // fill in the rest with random map points
        if (mPtObs.size() < num_map_point_select) {
            // take whatever passed the visibility check
            for (size_t i=0; i<mPtObs.size(); ++i) {
                tmpMapPoints[mPtObs[i].idx]->goodAtFrameId = mCurrentFrame.nNextId;
            }

            // add additional invisible set
            size_t j = mPtObs.size(), i = 0;
            while (j < num_map_point_select && i < tmpMapPoints.size()) {
                if (tmpMapPoints[i]->goodAtFrameId != mCurrentFrame.nNextId) {
                    //
                    tmpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId;
                    ++ j;
                }
                ++ i;
            }
        }
        else if (mPtObs.size() > num_map_point_select) {
            // Random shuffle the inliers
            std::random_shuffle ( mPtObs.begin(), mPtObs.end() );

            // set flag for good map points
            for (size_t i=0; i<num_map_point_select; ++i) {
                //            tmpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId;
                tmpMapPoints[mPtObs[i].idx]->goodAtFrameId = mCurrentFrame.nNextId;
            }
        }

#endif
    }

    //    double time_map_bound = timer.toc();
    //        logCurrentFrame.time_mat_pred = time_map_bound;
    //    std::cout << "func RunMapPointsSelection: actual time used  = " << time_map_bound << std::endl;

}

// we tested some idea on coreset & random enhancement in this func;
// not sure if any of the results being used in publication; most likely not.
bool Tracking::TrackLocalMap_TRO18() {

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackLocalMap()");
#endif
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    arma::wall_clock timer;
    timer.tic();
    // Update Local Map
    UpdateReference();
    double time_Upd_Ref = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of updating reference = " << time_Upd_Ref << std::endl;
#endif

    double time_Pool = 0, time_Subset = 0, time_Opti = 0, time_Post = 0;
    size_t mnMatchesGoodInliers, mnInitMatches = 0;
    std::vector<GoodPoint> mPtObs;

    //
    mCurrentFrame.mvbCandidate = vector<bool>(mCurrentFrame.N, true);

#if defined GOOD_FEATURE_FAIR_COMPARISON
    mPtObs.clear();
    mObsHandler->lmkSelectPool.clear();
    mCurrentFrame.mvbJacobBuilt = vector<bool>(mCurrentFrame.N, false);
    // NOTE
    // there is no need to do motion prediction again, since it's already be
    // predicted and somewhat optimzed in the 1st stage of pose tracking
    mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                               mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());
    // NOTE
    // instead of using actual time between consequtive frames, we construct virtual frame with slightly longer horizon;
    // the motivation being: reducing the size of matrix to 2-segments (which is the minimim-size); meanwhile preserving the spectral property
    mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 1 );

    // start a thread for matrix construction
    mObsHandler->pFrame = &mCurrentFrame;
    std::thread thread_Mat(&Observability::runMatrixBuilding, mObsHandler, 0.003, true);

#endif

    timer.tic();
    // Search Local MapPoints
    SearchReferencePointsInFrustum();

#if defined GOOD_FEATURE_FAIR_COMPARISON
    thread_Mat.join();
#endif

    double time_Search_Ref = timer.toc();
    //#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of searching more refs = " << time_Search_Ref << std::endl;
    //#endif

    ////    int mnInitOutliers = 0;
    //    for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
    //        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
    //        if(pMP) {

    //            //            // NOTE
    //            //            // experimental, set all matchings as inliers
    //            //            mCurrentFrame.mvbOutlier[i] = false;

    ////            if (mCurrentFrame.mvbOutlier[i] == false) {
    ////                mnInitMatches ++;
    ////            }
    //            assert(mCurrentFrame.mvbOutlier[i] == false);
    ////            if (mCurrentFrame.mvbOutlier[i] == true) {
    ////                mnInitOutliers ++;
    ////            }
    //        }
    //    }
    ////    std::cout << "func TrackLocalMap: init outlier number = " << mnInitOutliers << std::endl;

    timer.tic();
#ifdef ENABLE_EXPLICIT_OUTLIER_REJ

    cv::Mat pose_init, pose_opt_full;
    mCurrentFrame.mTcw.copyTo(pose_init);

    // Optimize Pose
    mnInitMatches = Optimizer::PoseOptimization(&mCurrentFrame);
    std::cout << "number of inlier chosen " << mnInitMatches << std::endl;
    // this->num_good_inlier_predef = float(mnInitMatches) * this->ratio_good_inlier_predef;
    mCurrentFrame.mTcw.copyTo(pose_opt_full);

    pose_init.copyTo(mCurrentFrame.mTcw);

#endif

#if defined RANSAC_POOL_FOR_SUBSET || defined INLIER_ONLY_BASELINE
    this->RanSACMatches(&mCurrentFrame, mPtObs);
    //    // Sort features according to the orb matching score
    //    std::sort(mPtObs.begin(), mPtObs.end(), obsLmkScore_ascend);
    //
    //    for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
    //        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
    //        if(pMP) {
    //            mCurrentFrame.mvbCandidate[i] = false;
    //        }
    //    }
    mCurrentFrame.mvbCandidate.assign(mCurrentFrame.N, false);

    mnInitMatches = mPtObs.size();
    for(size_t i= 0; i<mPtObs.size(); i++)  {
        mCurrentFrame.mvbCandidate[mPtObs[i].idx] = true;
    }
#endif

#if defined QUALITY_POOL_FOR_SUBSET
    if (mPtObs.empty()) {
        for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP) {
                if (mCurrentFrame.mvbOutlier[i] == false && mCurrentFrame.mvbCandidate[i] == true) {
                    GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
                    mPtObs.push_back( tmpLmk );
                }
            }
        }
    }
    //    std::cout << "func TrackLocalMap: number of quality feature before outlier rejection = " << mPtObs.size() << std::endl;

    mnInitMatches = mPtObs.size();
    // Sort features according to the orb matching score
    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::obsLmkScore_ascend);

    //    mnMatchesInliers  = static_cast<size_t>( this->num_good_inlier_predef * float(QUALITY_POOL_SCALE) );
    //    mnMatchesInliers  = static_cast<size_t>( mPtObs.size() );
    if (mnInitMatches * QUALITY_POOL_PERCENTILE > this->num_good_inlier_predef)
        mnMatchesInliers  = static_cast<size_t>(mnInitMatches * QUALITY_POOL_PERCENTILE);
    else
        mnMatchesInliers = mnInitMatches;
    for (size_t i=mnMatchesInliers; i<mPtObs.size(); ++i) {
        //        mCurrentFrame.mvbOutlier[mPtObs[i].idx] = true;
        mCurrentFrame.mvbCandidate[mPtObs[i].idx] = false;
        mnInitMatches --;
    }
#endif
    time_Pool = timer.toc();

    timer.tic();

    // In the following, budget constaint is enforced via a range of subset selection techniques
#if defined INLIER_ONLY_BASELINE
    // do nothing
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());
    //    std::cout << "number of ransac inlier to refine the pose: " << mPtObs.size() << std::endl;

#elif defined RANDOM_FAIR_COMPARISON

    mPtObs.clear();
    for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            if (mCurrentFrame.mvbOutlier[i] == false && mCurrentFrame.mvbCandidate[i] == true) {
                GoodPoint tmpLmk(static_cast<size_t>(i), 1);
                mPtObs.push_back( tmpLmk );
            }
        }
    }

    // Random shuffle the inliers
    std::random_shuffle ( mPtObs.begin(), mPtObs.end() );

    // Erase features that are ranked after ratio_good_inlier
    // this->num_good_inlier_predef = float(mPtObs.size()) * this->ratio_good_inlier_predef;
    size_t pivotPos  = static_cast<size_t>(this->num_good_inlier_predef);
    if( mPtObs.size() > pivotPos) {
        mPtObs.erase(mPtObs.begin() + pivotPos, mPtObs.end());
    }
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

#elif defined QUALITY_FAIR_COMPARISON

    mPtObs.clear();
    // Collect the mpSorted
    for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            if (mCurrentFrame.mvbOutlier[i] == false && mCurrentFrame.mvbCandidate[i] == true) {
                GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
                mPtObs.push_back( tmpLmk );
            }
        }
    }

    // Sort features according to the orb matching score
    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::obsLmkScore_ascend);

    // Erase features that are ranked after ratio_good_inlier
    // this->num_good_inlier_predef = float(mPtObs.size()) * this->ratio_good_inlier_predef;
    size_t pivotPos  = static_cast<size_t>(this->num_good_inlier_predef);
    if( mPtObs.size() > pivotPos) {
        mPtObs.erase(mPtObs.begin() + pivotPos, mPtObs.end());
    }
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

#elif defined BUCKETING_FAIR_COMPARISON

    mPtObs.clear();
    this->BucketingMatches(&mCurrentFrame, mPtObs);
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

#elif defined LONGEST_TRACK_FAIR_COMPARISON

    mPtObs.clear();
    this->LongLivedMatches(&mCurrentFrame, mPtObs);
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

#elif defined GOOD_FEATURE_FAIR_COMPARISON

    mPtObs.clear();
    double time_for_select = 0.005; // 0.0005; // 0.001; // 0.0015; //
    bool flagSucc = mObsHandler->setSelction_Number(this->num_good_inlier_predef, 3, time_for_select, &mCurrentFrame, &mPtObs);
    //    bool flagSucc = mObsHandler->setSelction_Number(this->num_good_inlier_predef, 4, time_for_select, &mCurrentFrame, &mPtObs);
    if (flagSucc == false) {
        // recover with the quality top-K back up
        //        mPtObs = mPtQual;
        // roll back to full set of matchings
        //        mPtObs.clear();
        //
        //        std::cout << "After lazier-greedy, size of the pool = " << mPtObs.size() << std::endl;
        if (mPtObs.size() < this->num_good_inlier_predef) {
            // append features to the select core set
            std::cout << "Adding " << this->num_good_inlier_predef - mPtObs.size() << " more lmks" << std::endl;

#if defined QUALITY_ENHANCEMENT
            std::sort(mObsHandler->lmkSelectPool.begin(), mObsHandler->lmkSelectPool.end(), GoodPoint::rankObsScore_ascend);
            for (size_t i = 0; i < mObsHandler->lmkSelectPool.size(); i++)  {
                if (mObsHandler->lmkSelectPool[i].selected == false && mObsHandler->lmkSelectPool[i].obs_score >= 0) {
                    mObsHandler->lmkSelectPool[i].selected = true;
                    mPtObs.push_back(mObsHandler->lmkSelectPool[i]);
                    //                    std::cout << "Take additional lmk with norm = " << mObsHandler->lmkSelectPool[i].obs_score << std::endl;
                }
                if (mPtObs.size() >= this->num_good_inlier_predef)
                    break ;
            }
            //
#elif defined RANDOM_ENHANCEMENT
            //            std::random_shuffle ( mRandLmkIdx.begin(), mRandLmkIdx.end() );
            //
            for(size_t i = 0; i < mRandLmkIdx.size(); i++)  {
                if (mRandLmkIdx[i] >= mCurrentFrame.mvpMapPoints.size())
                    continue ;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[mRandLmkIdx[i]];
                if(pMP) {
                    if (mCurrentFrame.mvbOutlier[mRandLmkIdx[i]] == false &&
                            mCurrentFrame.mvbCandidate[mRandLmkIdx[i]] == true &&
                            mCurrentFrame.mvbGoodFeature[mRandLmkIdx[i]] == false) {

                        GoodPoint tmpLmk(static_cast<size_t>(mRandLmkIdx[i]), 1);
                        mPtObs.push_back( tmpLmk );
                        mCurrentFrame.mvbGoodFeature[mRandLmkIdx[i]] = true;

                        if (mPtObs.size() >= this->num_good_inlier_predef)
                            break ;
                    }
                }
            }
#endif
        }
        else if (mPtObs.size() > this->num_good_inlier_predef) {
            // delet features from the select core set
            std::cout << "Deleting " << mPtObs.size() - this->num_good_inlier_predef << " more lmks" << std::endl;

#if defined QUALITY_ENHANCEMENT
            std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::rankObsScore_ascend);
#elif defined RANDOM_ENHANCEMENT
            std::random_shuffle ( mPtObs.begin(), mPtObs.end() );
#endif

            mPtObs.erase(mPtObs.begin() + this->num_good_inlier_predef, mPtObs.end());
            //
        }
    }
    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

#endif

    time_Subset = timer.toc();
    std::cout << "number of selected inlier to refine the pose: " << mPtObs.size() << std::endl;


    // Pose optimization
    timer.tic();
    if (this->num_good_feature_found > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        mnMatchesGoodInliers = Optimizer::PoseOptimization(&mCurrentFrame);
    }
    time_Opti = timer.toc();


    // Update MapPoints Statistics
    timer.tic();
    mnMatchesInliers = 0;
    size_t num_outlier = 0;
    for (size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i] == false && mCurrentFrame.mvbCandidate[i] == true) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                mnMatchesInliers ++;
            }
            else {
                num_outlier ++;
            }

            // recycle the obs mat per lmk
            mCurrentFrame.mvpMapPoints[i]->ObsScore = -1.0;

        }
    }
    //
    time_Post = timer.toc();

#ifdef LMKNUM_VERBOSE
    std::cout << "func TrackLocalMap: num of inliers = "  << mnMatchesInliers << ";" << std::endl;
    std::cout << "func TrackLocalMap: num of outliers = " << num_outlier      << ";" << std::endl;
#endif

    logCurrentFrame.lmk_num_refTrack = mnInitMatches; // mCurrentFrame.mvpMapPoints.size();
    logCurrentFrame.lmk_num_refInlier = mnMatchesInliers;
    logCurrentFrame.lmk_num_good = this->num_good_feature_found;
    logCurrentFrame.lmk_num_inlier_good = mnMatchesGoodInliers;
    //
    //    logCurrentFrame.time_track_map = time_Upd_Ref + time_Search_Ref + time_Pool + time_Subset + time_Opti + time_Post;
    logCurrentFrame.time_match = time_Upd_Ref + time_Search_Ref;
    logCurrentFrame.time_select = time_Pool + time_Subset;
    logCurrentFrame.time_select_Mat = mObsHandler->time_MatBuild;
    logCurrentFrame.time_select_Sel = mObsHandler->time_Selection;
    logCurrentFrame.time_optim = time_Opti + time_Post;
    //    logCurrentFrame.time_re_optim = time_ReOpt;
    //

#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time of candidate pool generation = "  << time_Pool << std::endl;
    std::cout << "func TrackLocalMap: time of subset selection = "  << time_Subset << std::endl;
    std::cout << "func TrackLocalMap: time of post processing = "   << time_Post << std::endl;
    std::cout << "func TrackLocalMap: time of matrix building = "   << mObsHandler->time_MatBuild << std::endl;
    std::cout << "func TrackLocalMap: time of lazier greedy = "     << mObsHandler->time_Selection << std::endl;
    std::cout << "func TrackLocalMap: time of optimization = "      << time_Opti << std::endl;
    std::cout << "func TrackLocalMap: total time = "           << logCurrentFrame.time_track_map << std::endl;
#endif

    //    // print out info for future analysis
    //    // report absolute feature numbers
    //    std::cerr << "lmkNum(k) = "             << mCurrentFrame.mvpMapPoints.size()    << ";" << std::endl;
    //    std::cerr << "goodLmkNum(k) = "         << this->num_good_feature_found         << ";" << std::endl;
    //    std::cerr << "inlierNum(k) = "          << mnMatchesInliers                     << ";" << std::endl;
    //    // write the number of outliers being cleared with Obs info
    //    //    std::cerr << "outlierNum(k) = "       << num_outlier << ";"                    << std::endl;
    //    // report time
    //    std::cerr << "timeTrackLocalMap(k) = "  << time_Upd_Ref + time_Search_Ref + time_Obs + time_Opt + time_Post << ";" << std::endl;
    //    std::cerr << "timeUpdRef(k) = "         << time_Upd_Ref     << ";"                    << std::endl;
    //    std::cerr << "timeMatchSrhEnh(k) = "    << time_Search_Ref  << ";"                    << std::endl;
    //    std::cerr << "timeObs(k) = "            << time_Obs         << ";"                    << std::endl;
    //    std::cerr << "timeReOpt(k) = "          << time_Opt         << ";"                    << std::endl;
    //    std::cerr << "timePost(k) = "           << time_Post        << ";"                    << std::endl;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<25)
        return false;

    //    if (mnMatchesInliers<30)
    if (mnMatchesInliers<15)
        return false;
    else
    {
        //        mFramePoseSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw));
        //        mFrameInlierSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mnMatchesInliers));
        return true;
    }
}


// According to the tests on KITTI, the scale drift would increase drastically if we cut the number of map points with obs
// To get rid of that part, we can evaluate the relative motion error at short-term
// Seems like there's no much difference when applied lmk selection to TrackLocalMap
bool Tracking::TrackLocalMap_ICRA18() {

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackLocalMap()");
#endif
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    arma::wall_clock timer;
    timer.tic();
    // Update Local Map
    UpdateReference();
    double time_Upd_Ref = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of updating reference = " << time_Upd_Ref << std::endl;
#endif

    timer.tic();
    // Search Local MapPoints
    SearchReferencePointsInFrustum();
    double time_Search_Ref = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of searching more refs = " << time_Search_Ref << std::endl;
#endif

    double time_Opt = 0, time_Post = 0, time_Obs = 0, time_ReOpt = 0;
#if defined OBS_RATIO_AT_TRACKLOCALMAP || defined OBS_NUMBER_AT_TRACKLOCALMAP

    timer.tic();
    // Get the camera state Xv for ObsComputor
    ObsComputor->updateCamState(mLastFrame.mTimeStamp, mLastFrame.getTwc(),
                                mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

    vector<GoodPoint>  mPtObs;
#ifdef OBS_RATIO_AT_TRACKLOCALMAP
    ObsComputor->getObsScore_Ratio(OBS_SEGMENT_NUM, mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp, // 1.0/double(camera_fps),
                                   this->ratio_good_inlier_predef, &mCurrentFrame, &mPtObs);
#else
    ObsComputor->getObsScore_Number(Xv_pred, OBS_SEGMENT_NUM, 1.0/double(camera_fps),
                                    this->num_good_inlier_predef, &mCurrentFrame, &mPtObs);
#endif
    time_Obs = timer.toc();

    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());

    //
    if (this->num_good_feature_found <= MIN_NUM_GOOD_FEATURES ) {

#ifdef LMKNUM_VERBOSE
        std::cout << "func TrackLocalMap: too few good features: " << this->num_good_feature_found << std::endl;
#endif

        timer.tic();
        mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);
        time_Opt = timer.toc();
        //
        timer.tic();
        // Mark the good features in the local map for robust matching
        for(size_t i = 0; i < mPtObs.size(); ++ i) {
            size_t j = mPtObs[i].idx;
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[j];
            if(pMP) {
                // for visualization
                mCurrentFrame.mvbGoodFeature[j] = true;
                // for robust matching
                pMP->goodAtFrameId = mCurrentFrame.nNextId;
            }
        }
        // Update MapPoints Statistics
        for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++) {
            if(mCurrentFrame.mvpMapPoints[i]) {
                if(!mCurrentFrame.mvbOutlier[i]) {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
            }
        }
        time_Post = timer.toc();

    }
    else {

#ifdef LMKNUM_VERBOSE
        std::cout << "func TrackLocalMap: num of good feature feed into opt: " << this->num_good_feature_found << std::endl;
#endif
        timer.tic();
        mnMatchesInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
        time_Opt = timer.toc();
        //
        timer.tic();
        // Update MapPoints Statistics-
        size_t num_outlier = 0, num_good_inlier = 0, num_norm_inlier = 0;
        for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++) {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP) {
                if (mCurrentFrame.mvbGoodFeature[i]) {
                    // only the outliers within the good feature set are rejected in tracking next frame
                    if (mCurrentFrame.mvbOutlier[i]) {
                        //                        // XXX: Clear the OBS info for all not-found points!!!!!!
                        //                        pMP->ObsMat.clear();
                        //                        pMP->ObsScore = 0.0;
                        //                        pMP->ObsRank = 0.0;
                        num_outlier ++;
                    }
                    else {
                        pMP->IncreaseFound();
                        num_good_inlier ++;
                    }
                }
                else {

#ifdef TRACKMAP_OUTLIER_REJECTION
                    // TODO
                    // TODO
                    // TODO
                    // Idealy there should be some outlier detection proc to set the proper flag for the rest of map points;
                    // say projecting them into the camera with pose estimated with good features, and set those with large
                    // reprojection error as outliers
                    //
                    // here the thresh, 2, is chosen after numbers of tests and comparison; it seems to reject a portion of
                    // outliers without hurting too much to build a complete map
                    //
                    float dU, dV;
                    mCurrentFrame.getProjectError(pMP, &(mCurrentFrame.mvKeysUn[i]), dU, dV);
                    arma::vec eVec = {dU, dV};
                    // get information matrix (identical as g2o optimization)
                    float invSigma2 = mCurrentFrame.mvInvLevelSigma2[mCurrentFrame.mvKeysUn[i].octave];
                    arma::mat infoMat(2, 2, arma::fill::eye);
                    infoMat = infoMat * invSigma2;
                    //                    std::cout << "infoMat = " << infoMat << std::endl;
                    // compute the chi2 error
                    float eChi2 = dot(eVec, infoMat * eVec);
                    //                    std::cout << "chi2 error = " << eChi2 << std::endl;
                    //                  if (eChi2 <= 5.991) {
                    if (eChi2 <= 7.378) {
                        //                  if (eChi2 <= 10) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        pMP->IncreaseFound();
                        num_norm_inlier ++;
                    }
                    else {
                        mCurrentFrame.mvbOutlier[i] = true;
                        num_outlier ++;
                    }
#else
                    //
                    pMP->IncreaseFound();
                    num_norm_inlier ++;
#endif
                }
            }
        }
        time_Post = timer.toc();
        //        mnMatchesInliers = num_good_inlier + num_norm_inlier;
        //
#ifdef LMKNUM_VERBOSE
        std::cout << "func TrackLocalMap: num of good inliers = " << num_good_inlier << ";" << std::endl;
        std::cout << "func TrackLocalMap: num of norm inliers = " << num_norm_inlier << ";" << std::endl;
        std::cout << "func TrackLocalMap: num of outliers = "     << num_outlier     << ";" << std::endl;
#endif

    }

#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time of computing obs = " << time_Obs << std::endl;
    std::cout << "func TrackLocalMap: time of optimization = " << time_Opt << std::endl;
    std::cout << "func TrackLocalMap: time of post proc = " << time_Post << std::endl;
#endif
    //
    std::cout << "func TrackLocalMap: time used = " << time_Upd_Ref + time_Search_Ref + time_Obs + time_Opt + time_Post << std::endl;

    // print out info for future analysis
    // report absolute feature numbers
    std::cerr << "lmkNum(k) = "             << mCurrentFrame.mvpMapPoints.size()    << ";" << std::endl;
    std::cerr << "goodLmkNum(k) = "         << this->num_good_feature_found         << ";" << std::endl;
    std::cerr << "inlierNum(k) = "          << mnMatchesInliers                     << ";" << std::endl;
    // write the number of outliers being cleared with Obs info
    //    std::cerr << "outlierNum(k) = "       << num_outlier << ";"                    << std::endl;
    // report time
    std::cerr << "timeTrackLocalMap(k) = "  << time_Upd_Ref + time_Search_Ref + time_Obs + time_Opt + time_Post << ";" << std::endl;
    std::cerr << "timeUpdRef(k) = "         << time_Upd_Ref     << ";"                    << std::endl;
    std::cerr << "timeMatchSrhEnh(k) = "    << time_Search_Ref  << ";"                    << std::endl;
    std::cerr << "timeObs(k) = "            << time_Obs         << ";"                    << std::endl;
    std::cerr << "timeReOpt(k) = "          << time_Opt         << ";"                    << std::endl;
    std::cerr << "timePost(k) = "           << time_Post        << ";"                    << std::endl;

#else

    cv::Mat pose_init;
    mCurrentFrame.mTcw.copyTo(pose_init);

    // Optimize Pose
    timer.tic();
    mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);
    time_Opt = timer.toc();

    int mnMatchesGoodInliers = 0;
    vector<GoodPoint>  mPtObs;
    //
    cv::Mat pose_opt_all;
    mCurrentFrame.mTcw.copyTo(pose_opt_all);

#if defined APPEND_GOOD_INLIER_OPT
    // take the inlier set from original ORB-SLAM optimization
    // perform good feature selection on the inlier set
    // discard the optimization result from orginal ORB-SLAM
    // use the result from good inlier set only
    std::cout << "number of inliers feed into selection: " << mnMatchesInliers << std::endl;

    timer.tic();

    pose_init.copyTo(mCurrentFrame.mTcw);
    // NOTE
    // there is no need to do motion prediction again, since it's already be
    // predicted and somewhat optimzed in the 1st stage of pose tracking
    ObsComputor->updateCamState(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

    //    std::cout << "duration = " << mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp << std::endl;
    //    std::cout << "mTcw = " << mCurrentFrame.mTcw << std::endl;
    //    std::cout << "Xv = " << ObsComputor->Xv << std::endl;

    //    ObsComputor->getObsScore_Ratio(OBS_SEGMENT_NUM, mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp,
    //                                   this->ratio_good_inlier_predef, &mCurrentFrame, &mPtObs);
    ObsComputor->getObsScore_Number(OBS_SEGMENT_NUM, mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp,
                                    this->num_good_inlier_predef, &mCurrentFrame, &mPtObs);

    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());
    time_Obs = timer.toc();

    std::cout << "number of good inlier to refine the pose: " << this->num_good_feature_found << std::endl;
    //
    timer.tic();
    if (this->num_good_feature_found > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        pose_opt_all.copyTo(mCurrentFrame.mTcw);
    }
    time_ReOpt = timer.toc();

#elif defined BUCKETING_FAIR_COMPARISON
    // for fair comparison, applying bucketing on top of inlier set of ORB-SLAM
    // perform optimization with the bucketed inlier set only
    // replace the original optimization result with the inlier only one
    pose_init.copyTo(mCurrentFrame.mTcw);

    timer.tic();
    this->BucketingMatches(&mCurrentFrame, mPtObs);
    time_Obs = timer.toc();

    std::cout << "number of bucketed inlier to refine the pose: " << mPtObs.size() << std::endl;

    timer.tic();
    if (mPtObs.size() > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        pose_opt_all.copyTo(mCurrentFrame.mTcw);
    }
    time_ReOpt = timer.toc();

#elif defined ORB_SLAM_FAIR_COMPARISON
    // for fair comparison, simply taking all inliers from original ORB-SLAM
    // perform optimization with the inlier set only
    // replace the original optimization result with the inlier only one
    pose_init.copyTo(mCurrentFrame.mTcw);

    // Collect the mpSorted
    for(int i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            if (mCurrentFrame.mvbOutlier[i] == false) {
                GoodPoint tmpLmk(static_cast<size_t>(i), 1);
                mPtObs.push_back( tmpLmk );
            }
        }
    }

    timer.tic();
    if (mPtObs.size() > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        pose_opt_all.copyTo(mCurrentFrame.mTcw);
    }
    time_ReOpt = timer.toc();

#elif defined QUALITY_FAIR_COMPARISON
    timer.tic();
    // for fair comparison, simply taking all inliers from original ORB-SLAM
    // perform optimization with the inlier set only
    // replace the original optimization result with the inlier only one
    pose_init.copyTo(mCurrentFrame.mTcw);

    // Collect the mpSorted
    for(int i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            if (mCurrentFrame.mvbOutlier[i] == false) {
                GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
                mPtObs.push_back( tmpLmk );
            }
        }
    }

    // Sort features according to the orb matching score
    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::obsLmkScore_ascend);

    // Erase features that are ranked after ratio_good_inlier
    size_t pivotPos  = static_cast<size_t>(this->num_good_inlier_predef);
    if( mPtObs.size() > pivotPos) {
        mPtObs.erase(mPtObs.begin() + pivotPos, mPtObs.end());
    }
    time_Obs = timer.toc();

    timer.tic();
    if (mPtObs.size() > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        pose_opt_all.copyTo(mCurrentFrame.mTcw);
    }
    time_ReOpt = timer.toc();

#elif defined QUALITY_GOOD_INLIER_COMB

    timer.tic();

    // Collect the mpSorted
    for(int i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            if (mCurrentFrame.mvbOutlier[i] == false) {
                GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
                mPtObs.push_back( tmpLmk );
            }
        }
    }

    // Sort features according to the orb matching score
    std::sort(mPtObs.begin(), mPtObs.end(), obsLmkScoreAscend);

    // Erase features that are ranked after ratio_good_inlier
    size_t pivotPos  = static_cast<size_t>(this->num_good_inlier_predef * 2);
    //    if( mPtObs.size() > pivotPos) {
    //        mPtObs.erase(mPtObs.begin() + pivotPos, mPtObs.end());
    //    }
    for (int i=pivotPos; i<mPtObs.size(); ++i) {
        size_t idx = mPtObs[i].idx;
        mCurrentFrame.mvbCandidate[idx] = false;
    }
    mPtObs.clear();


    pose_init.copyTo(mCurrentFrame.mTcw);
    // NOTE
    // there is no need to do motion prediction again, since it's already be
    // predicted and somewhat optimzed in the 1st stage of pose tracking
    ObsComputor->updateCamState(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

    //    ObsComputor->getObsScore_Ratio(OBS_SEGMENT_NUM, mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp,
    //                                   this->ratio_good_inlier_predef, &mCurrentFrame, &mPtObs);
    ObsComputor->getObsScore_Number(OBS_SEGMENT_NUM, mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp,
                                    this->num_good_inlier_predef, &mCurrentFrame, &mPtObs);

    this->num_good_feature_found = static_cast<size_t>(mPtObs.size());
    time_Obs = timer.toc();

    std::cout << "number of good inlier to refine the pose: " << this->num_good_feature_found << std::endl;
    //
    timer.tic();
    if (this->num_good_feature_found > MIN_NUM_GOOD_FEATURES ) {
        mnMatchesGoodInliers = Optimizer::PoseOptimization_Selected(&mCurrentFrame, mPtObs);
    }
    else {
        pose_opt_all.copyTo(mCurrentFrame.mTcw);
    }
    time_ReOpt = timer.toc();

#endif

    // Update MapPoints Statistics
    timer.tic();
    size_t num_outlier = 0;
    for (size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
            }
            else {
                num_outlier ++;
            }
        }
    }
    time_Post = timer.toc();

#ifdef LMKNUM_VERBOSE
    std::cout << "func TrackLocalMap: num of inliers = "  << mnMatchesInliers << ";" << std::endl;
    std::cout << "func TrackLocalMap: num of outliers = " << num_outlier      << ";" << std::endl;
#endif

#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time of computing obs = " << time_Obs << std::endl;
    std::cout << "func TrackLocalMap: time of optimization = " << time_Opt << std::endl;
    std::cout << "func TrackLocalMap: time of post proc = " << time_Post << std::endl;
#endif
    //
    std::cout << "func TrackLocalMap: time used = " << time_Upd_Ref + time_Search_Ref + time_Opt + time_Obs + time_ReOpt + time_Post << std::endl;
    //    // print out info for future analysis
    //    // report absolute feature numbers
    //    std::cerr << "lmkNum(k) = "             << mCurrentFrame.mvpMapPoints.size()    << ";" << std::endl;
    //    std::cerr << "goodLmkNum(k) = "         << this->num_good_feature_found         << ";" << std::endl;
    //    std::cerr << "inlierNum(k) = "          << mnMatchesInliers                     << ";" << std::endl;
    //    // write the number of outliers being cleared with Obs info
    //    //    std::cerr << "outlierNum(k) = "       << num_outlier << ";"                    << std::endl;
    //    // report time
    //    std::cerr << "timeTrackLocalMap(k) = "  << time_Upd_Ref + time_Search_Ref + time_Obs + time_Opt + time_Post << ";" << std::endl;
    //    std::cerr << "timeUpdRef(k) = "         << time_Upd_Ref     << ";"                    << std::endl;
    //    std::cerr << "timeMatchSrhEnh(k) = "    << time_Search_Ref  << ";"                    << std::endl;
    //    std::cerr << "timeObs(k) = "            << time_Obs         << ";"                    << std::endl;
    //    std::cerr << "timeReOpt(k) = "          << time_Opt         << ";"                    << std::endl;
    //    std::cerr << "timePost(k) = "           << time_Post        << ";"                    << std::endl;


    logCurrentFrame.lmk_num_refTrack = mCurrentFrame.mvpMapPoints.size();
    logCurrentFrame.lmk_num_refInlier = mnMatchesInliers;
    logCurrentFrame.lmk_num_good = this->num_good_feature_found;
    logCurrentFrame.lmk_num_inlier_good = mnMatchesGoodInliers;
    //
    //    logCurrentFrame.time_track_map = time_Upd_Ref + time_Search_Ref + time_Opt + time_Obs + time_ReOpt + time_Post;
    logCurrentFrame.time_match = time_Upd_Ref + time_Search_Ref;
    logCurrentFrame.time_select = time_Obs;
    logCurrentFrame.time_select_Mat = mObsHandler->time_MatBuild;
    logCurrentFrame.time_select_Sel = mObsHandler->time_Selection;
    logCurrentFrame.time_optim = time_Opt + time_Post;
    logCurrentFrame.time_re_optim = time_ReOpt;

#endif

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<25)
        return false;

    //    if (mnMatchesInliers<30)
    if (mnMatchesInliers<15)
        return false;
    else
    {
        //        mFramePoseSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw));
        //        mFrameInlierSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mnMatchesInliers));
        return true;
    }
}



bool Tracking::TrackLocalMap()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::TrackLocalMap()");
#endif
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    arma::wall_clock timer;
    timer.tic();
    // Update Local Map
    UpdateReference();
    double time_Upd_Ref = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of updating reference = " << time_Upd_Ref << std::endl;
#endif

    timer.tic();
    // Search Local MapPoints
#ifdef LOGGING_MATCHING
    int mnMatchesFound = SearchReferencePointsInFrustumWithLogging();
#else
    int mnMatchesFound = SearchReferencePointsInFrustum();
#endif
    double time_Search_Ref = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of searching more refs = " << time_Search_Ref << std::endl;
#endif

    timer.tic();

    //    size_t mnInitMatches = 0;
    //    for(size_t i= 0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
    //        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
    //        if(pMP) {
    //            if (mCurrentFrame.mvbOutlier[i] == false) {
    //                mnInitMatches ++;
    //            }
    //        }
    //    }

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame);
    double time_Opti = timer.toc();
#ifdef TIMECOST_VERBOSE
    std::cout << "func TrackLocalMap: time cost of pose optimization = " << time_Opti << std::endl;
#endif

    timer.tic();
    // Update MapPoints Statistics
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                //
                mCurrentFrame.mvpMapPoints[i]->goodAtFrameId = mCurrentFrame.nNextId;
            }
            //            else {
            //                /// XXX: Clear the OBS info for all not-found points!!!!!!
            //               mCurrentFrame.mvpMapPoints[i]->ObsMat.clear();
            //               mCurrentFrame.mvpMapPoints[i]->ObsScore = 0.0;
            //               mCurrentFrame.mvpMapPoints[i]->ObsRank = 0.0;
            //            }
        }
    }
    double time_Post = timer.toc();

#ifdef LMKNUM_VERBOSE
    std::cout << "func TrackLocalMap: number of lmks feed into optimization: "
              << mCurrentFrame.mvpMapPoints.size() << std::endl;
    std::cout << "func TrackLocalMap: number of lmks taken as inlier in optimization: "
              << mnMatchesInliers << std::endl;
#endif

    logCurrentFrame.lmk_num_refTrack = mnMatchesFound; // mCurrentFrame.mvpMapPoints.size();
    logCurrentFrame.lmk_num_refInlier = mnMatchesInliers;
    logCurrentFrame.lmk_num_BA = mnMatchesFound;
    //
    //    logCurrentFrame.time_track_map = time_Upd_Ref + time_Search_Ref + time_Opti + time_Post;
    logCurrentFrame.time_match = time_Upd_Ref + time_Search_Ref;
    logCurrentFrame.time_optim = time_Opti + time_Post;

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    if (mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<25)
        return false;

    //    if (mnMatchesInliers<30)
    if (mnMatchesInliers<15)
        return false;
    else
    {
        //        mFramePoseSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw));
        //        mFrameInlierSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mnMatchesInliers));
        return true;
    }

    //    // Decide if the tracking was succesful
    //    // More restrictive if there was a relocalization recently
    //    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    //        return false;

    //    if(mnMatchesInliers<30)
    //        return false;
    //    else {
    //        //        mFrameInlierSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mnMatchesInliers));
    //        //        mFramePoseSeq.push_back(std::make_pair(mCurrentFrame.mTimeStamp, mCurrentFrame.mTcw));
    //        return true;
    //    }
}


void Tracking::BucketingMatches(const Frame *pFrame, vector<GoodPoint> & mpBucketed) {

    //    int32_t max_features = BUCKET_FEATURE_NUM;
    float bucket_width = BUCKET_WIDTH;
    float bucket_height = BUCKET_HEIGHT;

    // find max values
    float u_max = -99999,   v_max = -99999;
    float u_min = 99999,    v_min = 99999;
    int32_t inlier_num = 0;
    for(int i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                //
                if (kpUn.pt.x > u_max)
                    u_max = kpUn.pt.x;
                if (kpUn.pt.y > v_max)
                    v_max = kpUn.pt.y;
                //
                if (kpUn.pt.x < u_min)
                    u_min = kpUn.pt.x;
                if (kpUn.pt.y < v_min)
                    v_min = kpUn.pt.y;
                //
                inlier_num ++;
            }
        }
    }

    //    std::cout << "u_max = " << u_max << "; "  << "v_max = " << v_max << "; " << std::endl;
    //    std::cout << "u_min = " << u_min << "; "  << "v_min = " << v_min << "; " << std::endl;

    // allocate number of buckets needed
    int32_t bucket_cols = (int32_t)floor( (u_max - u_min) / float(bucket_width) )  + 1;
    int32_t bucket_rows = (int32_t)floor( (v_max - v_min) / float(bucket_height) ) + 1;
    vector<size_t> *buckets = new vector<size_t>[bucket_cols*bucket_rows];

    //    std::cout << "bucket_cols = " << bucket_cols << "; "  << "bucket_rows = " << bucket_rows << "; " << std::endl;
    //    int32_t max_features = (int32_t)floor( float(inlier_num) * this->ratio_good_inlier_predef / float(bucket_cols*bucket_rows) );
    int32_t max_features = (int32_t)ceil( float(this->num_good_inlier_predef) / float(bucket_cols*bucket_rows) ) + 1;

    // assign matches to their buckets
    for(int i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                //                std::cout << "enter one map point" << std::endl;
                // kpUn.pt.x, kpUn.pt.y;
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];

                //                std::cout << kpUn.pt.x << "; " << kpUn.pt.y << ";" << bucket_width << "; " << bucket_height << std::endl;

                int32_t u = (int32_t)floor( float(kpUn.pt.x - u_min) / float(bucket_width) );
                int32_t v = (int32_t)floor( float(kpUn.pt.y - v_min) / float(bucket_height) );

                //                std::cout << "u = " << u << "; v = " << v << std::endl;
                buckets[ v * bucket_cols + u ].push_back( static_cast<size_t>(i) );
            }
        }
    }
    //    std::cout << "fill in content for buckets!" << std::endl;

    // refill p_matched from buckets
    size_t total_num = 0;
    bool stop_bucketing = false;
    mpBucketed.clear();
    for (size_t i=0; i<bucket_cols*bucket_rows; i++) {

        if (stop_bucketing == true)
            break ;

        // shuffle bucket indices randomly
        std::random_shuffle(buckets[i].begin(),buckets[i].end());

        // add up to max_features features from this bucket to p_matched
        size_t k=0;
        for (vector<size_t>::iterator it=buckets[i].begin(); it!=buckets[i].end(); it++) {
            //
            //            std::cout << "select match " << *it << " from bucket " << i << std::endl;
            GoodPoint tmpLmk(*it, 1);
            mpBucketed.push_back(tmpLmk);
            k++;
            total_num ++;
            //
            if (total_num >= this->num_good_inlier_predef) {
                stop_bucketing = true;
                break ;
            }
            if (k >= max_features)
                break;
        }
    }

    //    std::cout << "feature bucketed = " << total_num << std::endl;
    //    std::cout << "done with bucketing!" << std::endl;

    // free buckets
    delete []buckets;
}


void Tracking::LongLivedMatches(const Frame *pFrame, vector<GoodPoint> & mpLongLived) {

    mpLongLived.clear();
    for(size_t i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                GoodPoint tmpLmk(static_cast<size_t>(i), pMP->mnVisible);
                mpLongLived.push_back( tmpLmk );
            }
        }
    }

    std::sort(mpLongLived.begin(), mpLongLived.end(), GoodPoint::rankObsScore_descend);

    //
    if( mpLongLived.size() > this->num_good_inlier_predef) {
        mpLongLived.erase(mpLongLived.begin() + this->num_good_inlier_predef, mpLongLived.end());
    }
}


void Tracking::RanSACMatches(const Frame *pFrame, vector<GoodPoint> & mpRanSAC) {

    //
    //    cv::Mat pt3DMat, pt2DMat;
    vector<cv::Point3f> listPt3D;
    vector<cv::Point2f> listPt2D;
    vector<size_t> idxMap;
    for(size_t i= 0; i<pFrame->mvpMapPoints.size(); i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP) {
            if (pFrame->mvbOutlier[i] == false && pFrame->mvbCandidate[i] == true) {
                // add the 3D point
                cv::Mat pt3D = pMP->GetWorldPos();
                listPt3D.push_back( cv::Point3f(pt3D.at<float>(0), pt3D.at<float>(1), pt3D.at<float>(2)) );
                // add the 2D measurement
                //                cv::KeyPoint pt2D = pFrame->mvKeysUn[i];
                cv::KeyPoint pt2D = pFrame->mvKeys[i];
                listPt2D.push_back(pt2D.pt);
                //
                idxMap.push_back(i);
            }
        }
    }

    // call opencv PnP ransac solver to identify inliers
    cv::Mat inlierIdx;
    //    cv::Mat rvec, tvec;
    cv::Mat Rwc = pFrame->mTcw.rowRange(0,3).colRange(0,3), rvec;
    cv::Rodrigues(Rwc, rvec);
    cv::Mat tvec = pFrame->mTcw.rowRange(0,3).col(3);
    //    cv::Mat emptyCoef;
    //        std::cout << "initial: " << rvec << "; " << tvec << std::endl;
#if CV_MAJOR_VERSION == 2
    // Old OpenCV 2 code goes here.
    //    cv::solvePnPRansac(listPt3D, listPt2D, mK, mDistCoef, rvec, tvec, true, 100, 8.0, 0.95, inlierIdx, cv::SOLVEPNP_DLS);
    //    cv::solvePnPRansac(listPt3D, listPt2D, mK, emptyCoef, rvec, tvec, true, 40, 3.0, std::round(0.99 * float(listPt3D.size())), inlierIdx, CV_EPNP);
    cv::solvePnPRansac(listPt3D, listPt2D, mK, mDistCoef, rvec, tvec, true, RANSAC_ITER_NUMBER,
                       6.0, std::round(0.99 * float(listPt3D.size())), inlierIdx, CV_P3P);
#elif CV_MAJOR_VERSION == 3
    // New OpenCV 3 code goes here.
    cv::solvePnPRansac(listPt3D, listPt2D, mK, mDistCoef, rvec, tvec, true, RANSAC_ITER_NUMBER,
                       6.0, 0.99, inlierIdx, cv::SOLVEPNP_P3P);
#else
#error Unsupported opencv version
#endif
    //        std::cout << "solution: " << rvec << "; " << tvec << std::endl;

    // push the inliers accepted by PnP solver into the result vector
    mpRanSAC.clear();
    for (int i=0; i<inlierIdx.rows; ++i) {
        GoodPoint tmpLmk( static_cast<size_t>( idxMap[inlierIdx.at<int>(i)] ),
                double(pFrame->mvpMatchScore[ idxMap[inlierIdx.at<int>(i)] ])
                );
        //        std::cout << "adding inlier indexed by " << tmpLmk.idx << std::endl;
        mpRanSAC.push_back( tmpLmk );
    }

    //    std::cout << "func RanSACMatches: number of outliers being rejected = " << listPt3D.size() - mpRanSAC.size() << std::endl;

}


bool Tracking::NeedNewKeyFrame()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::NeedNewKeyFrame()");
#endif
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{


#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::CreateNewKeyFrame()");
#endif
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}




void Tracking::SearchAdditionalMatchesInFrame(const double time_for_search, Frame & F) {

    if (mObsHandler == NULL || mObsHandler->mLeftMapPoints.size() == 0)
        return ;

    arma::wall_clock timer;
    timer.tic();

    if (mObsHandler->mbNeedVizCheck) {
        //
        for(vector<MapPoint*>::iterator vit=mObsHandler->mLeftMapPoints.begin(), vend=mObsHandler->mLeftMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == F.mnId)
                continue;
            if(pMP->isBad())
                continue;

            if (timer.toc() > time_for_search / 2.0) {
                std::cout << "func SearchAdditionalMatchesInFrame: early stop in visibility check!" << std::endl;
                mObsHandler->mLeftMapPoints.erase(vit, vend);
                break;
            }

            // Project (this fills MapPoint variables for matching)
            if(F.isInFrustum(pMP,0.5))
            {
                pMP->IncreaseVisible();
            }
        }
    }

    double time_so_far = timer.toc();

    ORBmatcher matcher(0.8);
    double th = 0.8; // 0.2; // 1;
    //    if (mbTrackLossAlert == 1) {
    //        std::cout << "func SearchAdditionalMatchesInFrame: increase searching range to avoid track loss !!!" << std::endl;
    //        th = 1.6; // 0.4; // 3;
    //    }
    int nMatched = matcher.SearchByProjection_Budget(F,mObsHandler->mLeftMapPoints,th,time_for_search-time_so_far);

    std::cout << "func SearchAdditionalMatchesInFrame: found " << nMatched
              << " additional matches from " << mObsHandler->mLeftMapPoints.size()
              << " map points, with total time cost = " << timer.toc() << std::endl;

    logCurrentFrame.lmk_num_BA = logCurrentFrame.lmk_num_refTrack + nMatched;

}



int Tracking::SearchReferencePointsInFrustum()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::SearchReferencePoitsInFrustum()");
#endif
    // Do not search map points already matched
    size_t nMatchesFound = 0; // , nMatchesMinimum = static_cast<size_t>( double(mCurrentFrame.N) / 5.0 );
    arma::wall_clock timer;

#ifdef GOOD_FEATURE_MAP_MATCHING
    //    mCurrentInfoMat.zeros();
    mCurrentInfoMat = arma::eye( size(mCurrentInfoMat) ) * 0.00001;
    if (mFrameAfterInital > camera_fps * TIME_INIT_TRACKING && mCurrentFrame.mnId >= mnLastRelocFrameId+2) {
        //    cout << "update pose info in obs class" << endl;
        // NOTE
        // there is no need to do motion prediction again, since it's already be
        // predicted and somewhat optimzed in the 1st stage of pose tracking
        mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                   mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

        //    cout << "propagate pose info in obs class" << endl;
        // NOTE
        // instead of using actual time between consequtive frames, we construct virtual frame with slightly longer horizon;
        // the motivation being: reducing the size of matrix to 2-segments (which is the minimim-size); meanwhile preserving the spectral property
        mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 1 );

        mObsHandler->mKineIdx = 0;
        mObsHandler->mnFrameId = mCurrentFrame.mnId;

#ifdef FRAME_MATCHING_INFO_PRIOR
        //    mObsHandler->mMapPoints = &mCurrentFrame.mvpMapPoints;
        mObsHandler->pFrame = &mCurrentFrame;
        // compute info matrix for frame-by-frame matches
#ifdef USE_INFO_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::FRAME_INFO_MATRIX, 0.001, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::FRAME_HYBRID_MATRIX, 0.001, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
        // TODO
#endif

#endif
    }
#endif

    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;

#if defined GOOD_FEATURE_MAP_MATCHING && defined FRAME_MATCHING_INFO_PRIOR
                if (pMP->updateAtFrameId == mCurrentFrame.mnId)
                    mCurrentInfoMat += pMP->ObsMat;
#endif

                //
                nMatchesFound ++;
            }
        }
    }
    //    cout << mCurrentInfoMat << endl;

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch=0;

#if defined GOOD_FEATURE_MAP_MATCHING || defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING
    //
    mObsHandler->mLeftMapPoints.clear();
    mObsHandler->mbNeedVizCheck = false;
    double time_total_match = 0.015; //  1.0; //
    int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
    if (num_to_match <= 0) {
        // skip the rest
        for (size_t i=0; i<mvpLocalMapPoints.size(); ++i) {
            if (mvpLocalMapPoints[i] == NULL)
                continue ;
            if (mvpLocalMapPoints[i]->isBad())
                continue ;
            if(mvpLocalMapPoints[i]->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (mvpLocalMapPoints[i]->mbTrackInView == false)
                continue ;
            //
            mObsHandler->mLeftMapPoints.push_back(mvpLocalMapPoints[i]);
        }
        mObsHandler->mbNeedVizCheck = true;
        //
        return nMatchesFound;
    }

    double time_Viz = 0;
    timer.tic();
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;

        time_Viz = timer.toc();
        if (time_Viz > time_total_match / 2.0) {
            //
            mObsHandler->mLeftMapPoints = vector<MapPoint*>(vit, vend);
            mvpLocalMapPoints.erase(vit, vend);
            mObsHandler->mbNeedVizCheck = true;
            //
            break ;
        }

        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    cout << "time_Viz = " << time_Viz << "; mvpLocalMapPoints.size() = " << mvpLocalMapPoints.size() << endl;

#else
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

#endif
    //
    // TODO
    // Here is the place to inject Obs computation; to reduce the load of computation, we might need to approximate exact point Obs with region;
    // Following are the time cost of each step in TrackLocalMap:
    //
    int nMatched = 0;
    if(nToMatch>0)
    {

#ifdef GOOD_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400) { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            // TEST
            // try computing Jacobian for each map point
            mObsHandler->mMapPoints = &mvpLocalMapPoints;
#ifdef USE_INFO_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_INFO_MATRIX, (time_total_match-time_Viz)/2, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
            mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_HYBRID_MATRIX, (time_total_match-time_Viz)/2, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

            double time_Mat_Online = timer.toc();
            logCurrentFrame.time_mat_online = time_Mat_Online;
            std::cout << "func SearchReferencePointsInFrustum: time cost of matrix building = " << time_Mat_Online << endl;

#ifdef USE_INFO_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM::FRAME_INFO_MATRIX, mCurrentInfoMat,
                                                         th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_HYBRID_MATRIX
            nMatched = mObsHandler->runActiveMapMatching(&mCurrentFrame, ORB_SLAM::FRAME_HYBRID_MATRIX, mCurrentInfoMat,
                                                         th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#elif defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING || nToMatch < 400) { // 800)
            nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            // TEST
            // try computing Jacobian for each map point
            // print out the time cost
            // double time_total_match = 1.0; // 0.009; //

            mObsHandler->mMapPoints = &mvpLocalMapPoints;

            //
            // int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
#ifdef RANDOM_FEATURE_MAP_MATCHING
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM::BASELINE_RANDOM,
                                                           th,matcher,num_to_match,time_total_match-time_Viz);
#else
            nMatched = mObsHandler->runBaselineMapMatching(&mCurrentFrame, ORB_SLAM::BASELINE_LONGLIVE,
                                                           th,matcher,num_to_match,time_total_match-time_Viz);
#endif

        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#else

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        nMatched = matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);

#endif
    }

    return nMatched + nMatchesFound;
}

int Tracking::SearchReferencePointsInFrustumWithLogging()
{
    // Do not search map points already matched
    size_t nMatchesFound = 0; // , nMatchesMinimum = static_cast<size_t>( double(mCurrentFrame.N) / 5.0 );
    arma::wall_clock timer;

    //    mCurrentInfoMat.zeros();
    mCurrentInfoMat = arma::eye( size(mCurrentInfoMat) ) * 0.00001;
    if (mFrameAfterInital > camera_fps * TIME_INIT_TRACKING && mCurrentFrame.mnId >= mnLastRelocFrameId+2) {
        //    cout << "update pose info in obs class" << endl;
        // NOTE
        // there is no need to do motion prediction again, since it's already be
        // predicted and somewhat optimzed in the 1st stage of pose tracking
        mObsHandler->updatePWLSVec(mLastFrame.mTimeStamp, mLastFrame.mTcw,
                                   mCurrentFrame.mTimeStamp, mCurrentFrame.getTwc());

        //    cout << "propagate pose info in obs class" << endl;
        // NOTE
        // instead of using actual time between consequtive frames, we construct virtual frame with slightly longer horizon;
        // the motivation being: reducing the size of matrix to 2-segments (which is the minimim-size); meanwhile preserving the spectral property
        mObsHandler->predictPWLSVec( (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp), 1 );

        mObsHandler->mKineIdx = 0;
        mObsHandler->mnFrameId = mCurrentFrame.mnId;

        //    mObsHandler->mMapPoints = &mCurrentFrame.mvpMapPoints;
        mObsHandler->pFrame = &mCurrentFrame;
        // compute info matrix for frame-by-frame matches
#ifdef USE_INFO_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::FRAME_INFO_MATRIX, 0.01, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::FRAME_HYBRID_MATRIX, 0.01, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
        // TODO
#endif
    }

    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;

                if (pMP->updateAtFrameId == mCurrentFrame.mnId)
                    mCurrentInfoMat += pMP->ObsMat;

                //
                nMatchesFound ++;
            }
        }
    }
    //    cout << mCurrentInfoMat << endl;

    mCurrentFrame.UpdatePoseMatrices();

    int nToMatch=0;

#if defined GOOD_FEATURE_MAP_MATCHING || defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING
    //
    mObsHandler->mLeftMapPoints.clear();
    mObsHandler->mbNeedVizCheck = false;

#ifdef LOGGING_MATCHING
    double time_total_match = 1.0;
#else
    double time_total_match = 0.015; //  1.0; //
#endif

    int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
    if (num_to_match <= 0) {
        // skip the rest
        for (size_t i=0; i<mvpLocalMapPoints.size(); ++i) {
            if (mvpLocalMapPoints[i] == NULL)
                continue ;
            if (mvpLocalMapPoints[i]->isBad())
                continue ;
            if(mvpLocalMapPoints[i]->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (mvpLocalMapPoints[i]->mbTrackInView == false)
                continue ;
            //
            mObsHandler->mLeftMapPoints.push_back(mvpLocalMapPoints[i]);
        }
        mObsHandler->mbNeedVizCheck = true;
        //
        return nMatchesFound;
    }

    double time_Viz = 0;
    timer.tic();
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;

        time_Viz = timer.toc();
        if (time_Viz > time_total_match / 2.0) {
            //
            mObsHandler->mLeftMapPoints = vector<MapPoint*>(vit, vend);
            mvpLocalMapPoints.erase(vit, vend);
            mObsHandler->mbNeedVizCheck = true;
            //
            break ;
        }

        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    std::cout << "func SearchReferencePointsInFrustum: time cost of viz check = " << time_Viz
              << " with " << mvpLocalMapPoints.size() << " points!" << endl;

#else
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

#endif
    //
    // TODO
    // Here is the place to inject Obs computation; to reduce the load of computation, we might need to approximate exact point Obs with region;
    // Following are the time cost of each step in TrackLocalMap:
    //
    int nMatched = 0;
    if(nToMatch>0)
    {
        // TEST
        // try computing Jacobian for each map point
        mObsHandler->mMapPoints = &mvpLocalMapPoints;
#ifdef USE_INFO_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_INFO_MATRIX, 0.02, USE_MULTI_THREAD, false);
#elif defined USE_HYBRID_MATRIX
        mObsHandler->runMatrixBuilding(ORB_SLAM::MAP_HYBRID_MATRIX, 0.02, USE_MULTI_THREAD, false);
#elif defined USE_OBSERVABILITY_MATRIX
        // TODO
#endif


#ifdef GOOD_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            mvMatchingLog.clear();
            nMatched = matcher.SearchByProjectionWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                             mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING) // || nToMatch < 400) { // 800)
        {
            mvMatchingLog.clear();
            nMatched = matcher.SearchByProjectionWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                             mCurrentFrame,mvpLocalMapPoints,th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            double time_Mat_Online = timer.toc();
            logCurrentFrame.time_mat_online = time_Mat_Online;
            std::cout << "func SearchReferencePointsInFrustum: time cost of matrix building = " << time_Mat_Online << endl;

            mvMatchingLog.clear();
#ifdef USE_INFO_MATRIX
            nMatched = mObsHandler->runActiveMapMatchingWithLogging(mvMatchingLog,
                                                                    &mCurrentFrame, ORB_SLAM::FRAME_INFO_MATRIX, mCurrentInfoMat,
                                                                    th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_HYBRID_MATRIX
            nMatched = mObsHandler->runActiveMapMatchingWithLogging(mvMatchingLog,
                                                                    &mCurrentFrame, ORB_SLAM::FRAME_HYBRID_MATRIX, mCurrentInfoMat,
                                                                    th,matcher,num_to_match,time_total_match-time_Mat_Online-time_Viz);
#elif defined USE_OBSERVABILITY_MATRIX
            // TODO
#endif

        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#elif defined RANDOM_FEATURE_MAP_MATCHING ||  defined LONGLIVE_FEATURE_MAP_MATCHING

        timer.tic();

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2) {
            th=5;
            mvMatchingLog.clear();
            nMatched = matcher.SearchByProjectionWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                             mCurrentFrame,mvpLocalMapPoints,th);
        }
        else if (mFrameAfterInital <= camera_fps * TIME_INIT_TRACKING) // || nToMatch < 400) // 800)
        {
            mvMatchingLog.clear();
            nMatched = matcher.SearchByProjectionWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                             mCurrentFrame,mvpLocalMapPoints,th);
            cout << "nToMatch = " << nToMatch << "; nMatched = " << nMatched << endl;
        }
        else {
            // TEST
            // try computing Jacobian for each map point
            // print out the time cost
            // double time_total_match = 1.0; // 0.009; //

            mObsHandler->mMapPoints = &mvpLocalMapPoints;

            //
            mvMatchingLog.clear();
            // int num_to_match = this->num_good_inlier_predef - nMatchesFound; // 50;  //
#ifdef RANDOM_FEATURE_MAP_MATCHING
            nMatched = mObsHandler->runBaselineMapMatchingWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                                      &mCurrentFrame, ORB_SLAM::BASELINE_RANDOM,
                                                                      th,matcher,num_to_match,time_total_match-time_Viz);
#else
            nMatched = mObsHandler->runBaselineMapMatchingWithLogging(mvMatchingLog, mCurrentInfoMat,
                                                                      &mCurrentFrame, ORB_SLAM::BASELINE_LONGLIVE,
                                                                      th,matcher,num_to_match,time_total_match-time_Viz);
#endif

        }
        double time_Match = timer.toc();
        std::cout << "func SearchReferencePointsInFrustum: time cost of active matching = " << time_Match << "; matched feature = " << nMatched << endl;

#else

        ORBmatcher matcher(0.8);

        int th = 1;  // 1.5; // NOTE try increase the window size for vicon seq

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        mvMatchingLog.clear();
        nMatched = matcher.SearchByProjectionWithLogging(mvMatchingLog, mCurrentInfoMat, mCurrentFrame,mvpLocalMapPoints,th);

#endif
    }

    return nMatched + nMatchesFound;
}


void Tracking::UpdateReference()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::UpdateReference()");
#endif
    //    // This is for visualization
    //    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();

#ifndef DISABLE_MAP_VIZ
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
#endif

}

void Tracking::UpdateReferencePoints()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::UpdateReferencePoints()");
#endif
    mvpLocalMapPoints.clear();

    //    mvpLocalAdditionalPoints.clear();

    unsigned long minFrameId = mCurrentFrame.mnId; // - 2; // - 1; //
    if (mbTrackLossAlert == 2)
        minFrameId = 0;
    //    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    //        if (minFrameId > (*itKF)->mnFrameId)
    //            minFrameId = (*itKF)->mnFrameId;

    //    std::cout << "func RunMapPointsSelection: mToMatchMeasurement = " << mToMatchMeasurement
    //              << "; mMatchedLocalMapPoint = " << mMatchedLocalMapPoint << std::endl;
    //    if (mToMatchMeasurement > float(mCurrentFrame.N) * 0.875 && mMatchedLocalMapPoint + float(mCurrentFrame.N) * 0.9 < mToMatchMeasurement) {
    //        std::cout << "func RunMapPointsSelection: include more map points to avoid track loss !!!" << std::endl;
    //        minFrameId = 0;
    //    }

    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {

#if defined GOOD_FEATURE_MAP_BOUND || defined RANDOM_MAP_BOUND
                //                if (pMP->goodAtFrameId > 0 || mvpLocalMapPoints.size() < 2000) {
                //                    acceptMapPoint = true;
                //                }
                if (pMP->goodAtFrameId >= minFrameId) {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
                else {
                    // for those unselected map points, we still keep them and try matching them after current pose being published
                    mvpLocalAdditionalPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
#else
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
#endif

            }
        }
    }

    //    std::cout << "func UpdateReferencePoints: size of local map = " << mvpLocalMapPoints.size() << std::endl;
}

void Tracking::UpdateReferenceKeyFrames()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::UpdateReferenceKeyFrames()");
#endif
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(size_t i=0, iend=mCurrentFrame.mvpMapPoints.size(); i<iend;i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax=NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

bool Tracking::Relocalisation()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::Relocalisation()");
#endif
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;
    if(!RelocalisationRequested())
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(&mCurrentFrame);
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {

#ifdef TRACKING_VERBOSE
        ROS_INFO(">>>> Forced Relocalization == True!");
#endif
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
#ifdef TRACKING_VERBOSE
            std::cout << "Tcw: " << Tcw << std::endl;
#endif
            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mCurrentFrame.mvbOutlier.size(); io<ioend; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mCurrentFrame.mvpMapPoints.size(); ip<ipend; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(size_t io =0; io<mCurrentFrame.mvbOutlier.size(); io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::ForceRelocalisation()
{


#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::ForceRelocalization()");
#endif
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;
    mnLastRelocFrameId = mCurrentFrame.mnId;
}

bool Tracking::RelocalisationRequested()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::RelocalisationRequested()");
#endif
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}

void Tracking::Reset()
{

#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking::Reset()");
#endif
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = false;
        mbReseting = true;
    }

    // Wait until publishers are stopped
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped)
                break;
        }
        r.sleep();
    }

    // Reset Local Mapping
    mpLocalMapper->RequestReset();
    // Reset Loop Closing
    mpLoopClosing->RequestReset();
    // Clear BoW Database
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
}

void Tracking::CheckResetByPublishers()
{


#ifdef TRACKING_VERBOSE
    ROS_INFO("Tracking:CheckResetByPublishers:()");
#endif
    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }

    // Hold until reset is finished
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
        r.sleep();
    }
}


void Tracking::PlotFrameWithPointDetected(Frame * mframe, std::string mfname) {

    // create new image to modify it
    cv::Mat img_l_aux;
    mframe->im.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cv::cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    cv::Point       p;
    double          thick = 2.0; // 1.0;
    //    size_t match_counter = 0;
    //    std::string frame_wid;
    //    long time_stamp = mframe->mTimeStamp * pow(10, 9);
    //
    for(size_t i=0; i<mframe->mvKeysUn.size(); i++)  {
        cv::KeyPoint kpUn = mframe->mvKeysUn[i];
        p = cv::Point( int(kpUn.pt.x),
                       int(kpUn.pt.y) );
        //
        cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);
        //        match_counter ++;
    }

    //
    //    frame_wid = "/home/yipuzhao/ros_workspace/package_dir/GF_ORB_SLAM/test/";
    //    frame_wid += std::to_string(time_stamp);
    cv::imwrite( mfname + ".png", img_l_aux );

    //    ++ frame_counter;
}


void Tracking::PlotFrameWithPointMatches() {

    // create new image to modify it
    cv::Mat img_l_aux;
    mCurrentFrame.im.copyTo( img_l_aux );
    if( img_l_aux.channels() == 1 )
        cv::cvtColor(img_l_aux,img_l_aux,CV_GRAY2BGR);

    // Variables
    cv::Point       p;
    double          thick = 2.0; // 1.0;
    size_t match_counter = 0;
    std::string frame_wid;
    long time_stamp = mCurrentFrame.mTimeStamp * pow(10, 9);

    vector<GoodPoint>  mPtObs;

    // Collect the mpSorted
    for(size_t i=0; i<mCurrentFrame.mvpMapPoints.size(); i++)  {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP) {
            //            if (mCurrentFrame.mvbOutlier[i] == false) {
            GoodPoint tmpLmk(static_cast<size_t>(i), double(mCurrentFrame.mvpMatchScore[i]));
            mPtObs.push_back( tmpLmk );
            //            }
        }
    }

    // Sort features according to the orb matching score
    //    std::sort(mPtObs.begin(), mPtObs.end(), GoodPoint::rankObsScore_ascend);


    for(size_t i=0; i<mCurrentFrame.mvKeys.size(); i++)  {
        cv::KeyPoint kpUn = mCurrentFrame.mvKeys[i];
        p = cv::Point( int(kpUn.pt.x),
                       int(kpUn.pt.y) );
        //
        cv::drawMarker( img_l_aux, p, cv::Scalar(255,0,0), cv::MARKER_CROSS, 10, 2);
    }

    //
    for(size_t i =0; i<mPtObs.size(); i++)
    {
        cv::KeyPoint kpUn = mCurrentFrame.mvKeys[mPtObs[i].idx];
        p = cv::Point( int(kpUn.pt.x),
                       int(kpUn.pt.y) );
        //
//        if(mCurrentFrame.mvbOutlier[mPtObs[i].idx])
//        {
//            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,0,255), thick);
//        }
//        else
//        {
//            cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);
//        }
cv::circle( img_l_aux, p, 3.0, cv::Scalar(0,255,0), thick);

        match_counter ++;

        //
        //        if (match_counter == 50)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m050.png", img_l_aux );
        //        }
        //        else if (match_counter == 100)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m100.png", img_l_aux );
        //        }
        //        else if (match_counter == 150)
        //        {
        //            frame_wid = "/mnt/DATA/tmp/ScreenShot/";
        //            frame_wid += std::to_string(time_stamp);
        //            cv::imwrite( frame_wid + "_m150.png", img_l_aux );
        //        }
    }

    cv::putText(img_l_aux,
                "Keypoints Detected: " + std::to_string(mCurrentFrame.N) + "; Good Features Matched: " + std::to_string(match_counter),
                cv::Point(10, 470), // Coordinates
                cv::FONT_HERSHEY_PLAIN, // Font
                1.5, // Scale. 2.0 = 2x bigger
                cv::Scalar(0,0,255), // Color
                2.0); // Thickness

    //
    frame_wid = "/mnt/DATA/tmp/ScreenShot/";
    frame_wid += std::to_string(time_stamp);
    cv::imwrite( frame_wid + "_mAll.png", img_l_aux );

    //    ++ frame_counter;
}

//
#if CV_MAJOR_VERSION == 3
void Tracking::PlotInitialMap3D(const cv::Mat & image_1, const cv::Mat & image_2) {
    //
    //    assert(this->moViz3D != NULL);
    //
    //    if (this->moViz3D == NULL) {
    //        // re-init the window when being closed
    //        moViz3D = new cv::viz::Viz3d("Init Map Visualization");
    //    }
    moViz3D = new cv::viz::Viz3d("mapVisualization");
    //    this->viz_3D_lines->removeAllWidgets();
    cout << "creat the 3d viz widget" << endl;
    cout << "mpMap " << mpMap << endl;
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    cout << "mvpLocalMapPoints " << mvpLocalMapPoints.size() << endl;
    cv::Mat clout3d = cv::Mat(1, mvpLocalMapPoints.size(), CV_32FC3);
    for (int i=0; i<mvpLocalMapPoints.size(); ++i) {
        //
        cv::Mat pt3d = mvpLocalMapPoints[i]->GetWorldPos();
        cout << pt3d << endl;
        clout3d.at<cv::Vec3f>(0, i)[0] = pt3d.at<float>(0);
        clout3d.at<cv::Vec3f>(0, i)[1] = pt3d.at<float>(1);
        clout3d.at<cv::Vec3f>(0, i)[2] = pt3d.at<float>(2);
        //        if (pt3d.rows == 1)
        //            cv::hconcat(clout3d, pt3d.t(), clout3d);
        //        else
        //            cv::hconcat(clout3d, pt3d, clout3d);
    }

    cv::viz::WCloud cloudviz(clout3d);
    std::string cloudid = std::string("init_map");
    this->moViz3D->showWidget(cloudid, cloudviz);

    // Add coordinate axes
    cv::Matx33d Kl;
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            Kl(i,j) = this->mK.at<float>(i,j);
        }
    }
    //    std::cout << Kl << std::endl;

    std::string frame1_wid = "frame_1";
    this->moViz3D->showWidget(frame1_wid, cv::viz::WCameraPosition(
                                  Kl, image_1, 0.3));
    cv::Affine3f pose_1 = cv::Affine3f(
                cv::Vec3f(0, 0, 0),
                cv::Vec3f(0, 0, 0)
                );
    this->moViz3D->setWidgetPose(frame1_wid, pose_1);

    std::string frame2_wid = "frame_2";
    this->moViz3D->showWidget(frame2_wid, cv::viz::WCameraPosition(
                                  Kl, image_2, 0.3));
    cv::Affine3f pose_2 = cv::Affine3f(
                cv::Vec3f(0, 0, 0),
                cv::Vec3f(-0.1, 0, 0)
                );
    this->moViz3D->setWidgetPose(frame2_wid, pose_2);

    cv::Mat Rvec(3, 1, CV_32F);
    Rvec.at<float>(0) = -0.526415;
    Rvec.at<float>(1) = -0.589073; // ;
    Rvec.at<float>(2) = -0.304148; // CV_PI / 4;
    cv::Affine3f viewPose = cv::Affine3f(
                Rvec,
                cv::Vec3f(3.9439, -4.32466, -3.38602)
                );
    this->moViz3D->setViewerPose(pose_2 * viewPose);

    //    cv::Mat Rvec(3, 1, CV_64F);
    //    Rvec.at<double>(0) = -0.142623;
    //    Rvec.at<double>(1) = -2.18779; // ;
    //    Rvec.at<double>(2) = -0.283118; // CV_PI / 4;
    //    cv::Affine3d viewPose = cv::Affine3d(
    //                Rvec,
    //                cv::Vec3d(10.8226, -3.84858, 10.7423)
    //                );
    //    this->viz_3D_lines->setViewerPose(viewPose);

    //    std::cout << "start to plot 3D line!" << std::endl;
    //    this->moViz3D->spinOnce(1, true);
    this->moViz3D->spin();

    //    // for acquiring proper view port in debugger only!
    //    while(!this->viz_3D_lines->wasStopped()) {
    //        cv::Affine3d viewPose = this->viz_3D_lines->getViewerPose();
    //        this->viz_3D_lines->spinOnce(1, true);
    //        //
    //        std::cout << "current viewer pose: " << std::endl;
    //        //        for (int i=0; i<4; ++i) {
    //        //            for (int j=0; j<4; ++j)
    //        //                std::cout << viewPose.at<double>(i,j) << " ";
    //        //            std::cout << std::endl;
    //        //        }
    //        cv::Vec<double, 3> tvec = viewPose.translation();
    //        cv::Vec<double, 3> rvec = viewPose.rvec();
    //        std::cout << tvec[0] << " " << tvec[1] << " " << tvec[2] << "; ";
    //        std::cout << rvec[0] << " " << rvec[1] << " " << rvec[2] << std::endl;

    //    }

    this->moViz3D->removeAllWidgets();
}
#endif

} //namespace ORB_SLAM

