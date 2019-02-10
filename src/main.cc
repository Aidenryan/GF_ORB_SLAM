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
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include "Converter.h"


using namespace std;


bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "GF_ORB_SLAM");
    ros::start();

    std::cout << std::endl << "% GF-SLAM Version 1.00" << std::endl;

    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun GF_ORB_SLAM GF_ORB_SLAM path_to_vocabulary path_to_settings "
             << "ratio_good_inlier (absolute or relative to package directory)"
             << "rostopic_image_raw path_to_results" << endl;
        ros::shutdown();
        return 1;
    }

    // Load Settings and Check
    string strSettingsFile = ros::package::getPath("GF_ORB_SLAM")+"/"+argv[2];
    std::cout << "Load settings and check " << strSettingsFile << std::endl;

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    //Create Frame Publisher for image_view
    int nRows = fsSettings["Camera2.nRows"];
    int nCols = fsSettings["Camera2.nCols"];
    ORB_SLAM::FramePublisher FramePub(nRows, nCols);
    std::cout << "Set-up FramePublisher" << std::endl;
    
    // New version to load vocabulary from text file "Data/ORBvoc.txt".
    // If you have an own .yml vocabulary, use the function
    // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
    arma::wall_clock timer;
    timer.tic();
    string strVocFile = ros::package::getPath("GF_ORB_SLAM")+"/"+argv[1];
    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while." << std::endl;

    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = Vocabulary.loadFromTextFile(strVocFile);
    else
        bVocLoad = Vocabulary.loadFromBinaryFile(strVocFile);
    //    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);

    if(!bVocLoad)
    {
        std::cout << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << std::endl;
        std::cout << "Falied to open at: " << strVocFile << std::endl;
        ros::shutdown();
        return 1;
    }

    double time_loadVoc = timer.toc();
    std::cout << "Vocabulary loaded with " << time_loadVoc << " sec!" << std::endl;

    //    // load pose ground truth data
    //    string strGTPosePath = ros::package::getPath("GF_ORB_SLAM")+"/"+argv[3];
    //    cout << strGTPosePath << endl;

    //    // load regressed obs threshold data
    //    string strObsThresPath = ros::package::getPath("GF_ORB_SLAM")+"/"+argv[4];
    //    cout << strObsThresPath << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile, argv[4], atoi(argv[3]));
    //    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile, argv[4], atof(argv[3]));
    //    //Set the ratio of GF in tracker
    //    Tracker.ratio_good_inlier_predef = atof(argv[3]);
    //        std::cout << "Set ratio of Good Feature = " << Tracker.ratio_good_inlier_predef << std::endl;
    //Set the fixed number of GF in tracker
    //    Tracker.num_good_inlier_predef = atoi(argv[3]);
    std::cout << "Set number of Good Feature = " << Tracker.num_good_inlier_predef << std::endl;
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run, &Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, &LocalMapper);

    //Simply bind the tracker and mapper
    Tracker.SetLocalMapper(&LocalMapper);
    LocalMapper.SetTracker(&Tracker);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);
    //Bind the loop closure thread to tracking & mapping
    Tracker.SetLoopClosing(&LoopCloser);
    LocalMapper.SetLoopCloser(&LoopCloser);
    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    //Set the real time track writing file stream
    //    string fNameRT = ros::package::getPath("GF_ORB_SLAM")+"/"+"AllFrameTrajectory.txt";
    std::string fNameRealTimeTrack = std::string(argv[5]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    //    std::cerr << std::endl << "%Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    Tracker.SetRealTimeFileStream(fNameRealTimeTrack);

    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    ros::Rate r(fps);

    //    std::cerr << "k = 0;" << std::endl;
    while (ros::ok())
    {
        FramePub.Refresh();
        MapPub.Refresh();
        Tracker.CheckResetByPublishers();
        r.sleep();
    }



    // Save keyframe poses at the end of the execution
    vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

    std::cout << std::endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << std::endl;
    //    std::cerr << std::endl << "%Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << std::endl;

    ofstream fKeyFrameTrack;
    //    string strFile = ros::package::getPath("GF_ORB_SLAM")+"/"+"KeyFrameTrajectory.txt";
    std::string fNameKeyFrameTrack = std::string(argv[5]) + "_KeyFrameTrajectory.txt";
    //    string strFile = "/mnt/DATA/GoogleDrive/ORB_SLAM/KITTI_POSE_GF_SLAM/KeyFrameTrajectory.txt";
    fKeyFrameTrack.open(fNameKeyFrameTrack.c_str());
    fKeyFrameTrack << fixed;
    fKeyFrameTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        fKeyFrameTrack << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
                       << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
                       << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
    fKeyFrameTrack.close();

    // save the time log
    //
    //#ifdef TRACKING_TIME_LOGGING
    std::string fNameFrameTimeLog = std::string(argv[5]) + "_Log.txt";
    Tracker.SaveTimeLog(fNameFrameTimeLog);
    //#endif

#ifdef ORB_EXTRACTOR_TIME_LOGGING
    std::string fNameORBTimeLog = std::string(argv[5]) + "_LogORB.txt";
    Tracker.SaveORBLog(fNameORBTimeLog);
#endif

#ifdef LOCAL_BA_TIME_LOGGING
    std::string fNameBATimeLog = std::string(argv[5]) + "_LogBA.txt";
    LocalMapper.SaveTimeLog(fNameBATimeLog);
#endif

    //    // save the greedy debug log
    //    ofstream fFrameDebugLog;
    //    std::string fNameFrameDebugLog = std::string(argv[5]) + "_Debug.txt";
    //    fFrameDebugLog.open(fNameFrameDebugLog.c_str());
    //    fFrameDebugLog << fixed;
    //    fFrameDebugLog << "#lmk_selected   lmk_diff" << std::endl;
    //    for(size_t i=0; i<Tracker.mObsHandler->mDebugLog_.size(); i++)
    //    {
    //        fFrameDebugLog << setprecision(0)
    //                      << Tracker.mObsHandler->mDebugLog_[i].first << " "
    //                      << Tracker.mObsHandler->mDebugLog_[i].second << std::endl;
    //    }
    //    fFrameDebugLog.close();



#ifdef OBS_THRESH_SWEEP
    // Output observability threshold records
    ofstream f_Obs;
    string strFileObs = ros::package::getPath("GF_ORB_SLAM")+"/"+"FrameObsRec.txt";
    f_Obs.open(strFileObs.c_str());
    f_Obs << fixed;

    f_Obs << "#TimeStamp Init[Tx Ty Tz Qx Qy Qz Qw] NumLmkInit Opt[Tx Ty Tz Qx Qy Qz Qw] NumLmkOpt ObsThres XXX" << std::endl;
    for(size_t i=0; i<Tracker.obs_thres_arr.size(); i++)
    {
        f_Obs << setprecision(6) << Tracker.obs_thres_arr[i].time_stamp << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_init[0] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_init[1] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_init[2] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_init[0] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_init[1] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_init[2] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_init[3] << " "
              << Tracker.obs_thres_arr[i].num_lmk_init << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_best[0] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_best[1] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].T_best[2] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_best[0] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_best[1] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_best[2] << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].Q_best[3] << " "
              << Tracker.obs_thres_arr[i].num_lmk_best << " "
              << setprecision(7) << Tracker.obs_thres_arr[i].thre_obs_best << " ";

        // detailed infor for all lmks on current frame
        if (Tracker.obs_thres_arr[i].lmk_arr != NULL) {
            f_Obs << Tracker.obs_thres_arr[i].lmk_arr->size() << std::endl;
            //
#ifdef OBS_ALL_LMK_OUTPUT
            for (size_t j = 0; j < Tracker.obs_thres_arr[i].lmk_arr->size(); ++ j) {
                //
                GoodPoint tmlObsLmk = Tracker.obs_thres_arr[i].lmk_arr->at(j);
                f_Obs << tmlObsLmk.idx << " "
                      << setprecision(7) << tmlObsLmk.obs_score << " "
                      << setprecision(7) << tmlObsLmk.pI[0] << " "
                      << setprecision(7) << tmlObsLmk.pI[1] << " "
                      << setprecision(7) << tmlObsLmk.pW[0] << " "
                      << setprecision(7) << tmlObsLmk.pW[1] << " "
                      << setprecision(7) << tmlObsLmk.pW[2] << std::endl;
            }
#endif
        }
        else {
            f_Obs << "0" << std::endl;
        }
    }
    f_Obs.close();
#endif

    std::cout << "Finished saving!" << std::endl;
    //    std::cerr << "%Finished saving!" << std::endl;
    ros::shutdown();

    return 0;
}
