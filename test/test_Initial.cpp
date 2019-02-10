// Copyright 2005, Google Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// A sample program demonstrating using Google C++ testing framework.
//
// Author: wan@google.com (Zhanyong Wan)


// In this example, we use a more advanced feature of Google Test called
// test fixture.
//
// A test fixture is a place to hold objects and functions shared by
// all tests in a test case.  Using a test fixture avoids duplicating
// the test code necessary to initialize and cleanup those common
// objects for each test.  It is also useful for defining sub-routines
// that your tests need to invoke a lot.
//
// <TechnicalDetails>
//
// The tests share the test fixture in the sense of code sharing, not
// data sharing.  Each test is given its own fresh copy of the
// fixture.  You cannot expect the data modified by one test to be
// passed on to another test, which is a bad idea.
//
// The reason for this design is that tests should be independent and
// repeatable.  In particular, a test should not fail as the result of
// another test's failure.  If one test depends on info produced by
// another test, then the two tests should really be one big test.
//
// The macros for indicating the success/failure of a test
// (EXPECT_TRUE, FAIL, etc) need to know what the current test is
// (when Google Test prints the test result, it tells you which test
// each failure belongs to).  Technically, these macros invoke a
// member function of the Test class.  Therefore, you cannot use them
// in a global function.  That's why you should put test sub-routines
// in a test fixture.
//
// </TechnicalDetails>

#include "Observability.h"
#include "Tracking.h"
#include "gtest/gtest.h"
//#include "gtest/gmock.h"

using namespace ORB_SLAM;

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestInitialization : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // initialize the obs class
        //        std::cout << "start setting up" << std::endl;
        std::string name = "initialization_test";
        int argc = 0;
        ros::init(argc, NULL, name);

        std::string pathROSWorkSpace = "/home/yipuzhao/ros_workspace/package_dir";

        // Load Settings and Check
        std::string strSettingsFile = pathROSWorkSpace + "/ORB_Data/Gazebo_yaml/gazebo_params.yaml";
        std::cout << "Load settings and check " << strSettingsFile << std::endl;

        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            return ;
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
        std::string strVocFile = pathROSWorkSpace + "/ORB_Data/ORBvoc.bin";
        std::cout << std::endl << "Loading ORB Vocabulary. This could take a while." << std::endl;

        ORB_SLAM::ORBVocabulary Vocabulary;
        bool bVocLoad = false; // chose loading method based on file extension
        bVocLoad = Vocabulary.loadFromBinaryFile(strVocFile);

        if(!bVocLoad)
        {
            std::cout << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << std::endl;
            std::cout << "Falied to open at: " << strVocFile << std::endl;
            return ;
        }

        double time_loadVoc = timer.toc();
        std::cout << "Vocabulary loaded with " << time_loadVoc << " sec!" << std::endl;

        //Create the map
        ORB_SLAM::Map World;

        FramePub.SetMap(&World);

        //Create Map Publisher for Rviz
        ORB_SLAM::MapPublisher MapPub(&World);

        //Initialize the Tracking Thread and launch
        Tracker = new ORB_SLAM::Tracking(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile, "NULL", 1000);
        //        Tracker = new ORB_SLAM::Tracking(&Vocabulary, strSettingsFile, 1000);
        std::cout << "Tracker initialized!" << std::endl;

        //Initialize the Local Mapping Thread and launch
        LocalMapper = new ORB_SLAM::LocalMapping(&World);
        boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, LocalMapper);

        //Simply bind the tracker and mapper
        Tracker->SetLocalMapper(LocalMapper);
        LocalMapper->SetTracker(Tracker);

        Tracker->mvInitPoses.push_back(std::make_pair(1.0, cv::Mat::eye(4, 4, CV_32F)));
        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Tcw.at<float>(0, 3) = -0.1f;
        Tracker->mvInitPoses.push_back(std::make_pair(1.1, Tcw));

        //Load the 1st image
        im_1 = cv::imread(pathROSWorkSpace + "/GF_ORB_SLAM/test/0000000116466000000.png", CV_LOAD_IMAGE_GRAYSCALE);
        Tracker->mCurrentFrame = Frame(im_1, 1.0, Tracker->mpIniORBextractor, Tracker->mpORBVocabulary, Tracker->mK, Tracker->mDistCoef);
        Tracker->FirstInitialization(ORB_SLAM::Tracking::eInitMethod::Struct_Only);

        ///---------------------------------------

        //Load the 2nd image
        im_2 = cv::imread(pathROSWorkSpace + "/GF_ORB_SLAM/test/0000000116465000000.png", CV_LOAD_IMAGE_GRAYSCALE);
        Tracker->mCurrentFrame = Frame(im_2, 1.1, Tracker->mpIniORBextractor, Tracker->mpORBVocabulary, Tracker->mK, Tracker->mDistCoef);
        Tracker->Initialize(ORB_SLAM::Tracking::eInitMethod::Struct_Only);

        cout << "done!" << endl;
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    ORB_SLAM::Tracking * Tracker;
    ORB_SLAM::LocalMapping * LocalMapper;
    cv::Mat im_1, im_2;
};


TEST_F(TestInitialization, StructureOnly) {
    //
    Tracker->PlotInitialMap3D(im_1, im_2);

}

}  // namespace
