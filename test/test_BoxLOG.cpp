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

#include "BoxLOG.hpp"
#include "ORBextractor.h"
#include "gtest/gtest.h"
//#include "gtest/gmock.h"

using namespace ORB_SLAM;

namespace {
// To use a test fixture, derive a class from testing::Test.
class TestBoxLOG : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // load testing image
        image_ = cv::imread("./test/1403636694613555456.png", CV_LOAD_IMAGE_GRAYSCALE);
        //
        detector_ = new BoxLoGDetector();
        extractor_ = new ORBextractor();

        do_viz = false; // true; //
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    BoxLoGDetector * detector_;
    ORBextractor * extractor_;
    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat mDescriptors_;

    bool do_viz;
};


// Tests the generation of responce image.
TEST_F(TestBoxLOG, ResponceImage) {
    //
    keypoints_.clear();
    detector_->run(image_, keypoints_, 2, 20.0f);

    if (do_viz) {
        detector_->vizResponce(std::string("LoG responce"));
        detector_->vizKeypoints(std::string("Keypoints at base level"), image_, keypoints_);
    }
}

// Tests the generation of responce image.
TEST_F(TestBoxLOG, MultiLevelDetector) {
    //
    keypoints_.clear();
    (*extractor_)(image_,cv::Mat(),keypoints_,mDescriptors_);

    if (do_viz) {
        detector_->vizKeypoints(std::string("Keypoints at multi-level"), image_, keypoints_);
    }
}

}  // namespace
