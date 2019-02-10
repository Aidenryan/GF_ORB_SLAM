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
class TestRANSAC : public testing::Test {
protected:  // You should make the members protected s.t. they can be
    // accessed from sub-classes.

    // virtual void SetUp() will be called before each test is run.  You
    // should define it if you need to initialize the variables.
    // Otherwise, this can be skipped.
    virtual void SetUp() {
        // initialize the obs class
        //        std::cout << "start setting up" << std::endl;
        // EuRoC cam param
        double C2_f = 5.1369248;
        double C2_k1= 0; // -0.28340811;
        double C2_k2= 0; // 0.07395907;
        double C2_cx= 367.215;
        double C2_cy= 248.375;
        double C2_dx= 0.01123325985; // 0.0112;
        double C2_dy= 0.01123325985;
        double C2_nCols= 752;
        double C2_nRows= 480;

        //        // Matlab cam param
        //        double C2_f = 1.0;
        //        double C2_k1= 0;
        //        double C2_k2= 0;
        //        double C2_cx= 320;
        //        double C2_cy= 240;
        //        double C2_dx= 1.0/320.0;
        //        double C2_dy= 1.0/320.0;
        //        double C2_nRows= 480;
        //        double C2_nCols= 640;

        //        arma::mat C2_K;
        //        C2_K << C2_f/C2_dx << 0.0 << C2_cx << arma::endr
        //             << 0.0 << C2_f/C2_dy << C2_cy << arma::endr
        //             << 0.0 << 0.0 <<  1.0 << arma::endr;

        //        std::cout << "instantiate observability object" << std::endl;
        obs_ = new Observability(C2_f, C2_nRows, C2_nCols, C2_cx, C2_cy,
                                 C2_k1, C2_k2, C2_dx, C2_dy);

        //        std::cout << "propagate pose param" << std::endl;
        obs_->Xv << 0.3000  << -0.1000 <<   1.0000  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
                 <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
        obs_->predictPWLSVec(0.1, 2);

        //        std::cout << "fill in map and measurement variables" << std::endl;
        // fill in necessary members
        pFrame_ = new Frame();
        pFrame_->N = 600; // 1500; //
        //Scale Levels Info
        pFrame_->mnScaleLevels = 8;
        pFrame_->mfScaleFactor = 1.2;

        pFrame_->mvScaleFactors.resize(pFrame_->mnScaleLevels);
        pFrame_->mvLevelSigma2.resize(pFrame_->mnScaleLevels);

        pFrame_->mvScaleFactors[0]=1.0f;
        pFrame_->mvLevelSigma2[0]=1.0f;
        for(int i=1; i<pFrame_->mnScaleLevels; i++)
        {
            pFrame_->mvScaleFactors[i]=pFrame_->mvScaleFactors[i-1]*pFrame_->mfScaleFactor;
            pFrame_->mvLevelSigma2[i]=pFrame_->mvScaleFactors[i]*pFrame_->mvScaleFactors[i];
            //            std::cout << "sigma at level " << i << ": " << pFrame_->mvLevelSigma2[i] << std::endl;
        }

        pFrame_->mvInvLevelSigma2.resize(pFrame_->mvLevelSigma2.size());
        for(int i=0; i<pFrame_->mnScaleLevels; i++)
            pFrame_->mvInvLevelSigma2[i]=1/pFrame_->mvLevelSigma2[i];

        obs_->kinematic[1].Tcw.copyTo(pFrame_->mTcw);
        pFrame_->mK = (cv::Mat_<double>(3,3) << C2_f/C2_dx, 0.0, C2_cx, 0.0, C2_f/C2_dy, C2_cy, 0, 0, 1);

        //
        size_t i = 0;
        while (i < pFrame_->N) {
            pFrame_->mvbOutlier.push_back(false);
            pFrame_->mvbCandidate.push_back(true);
            pFrame_->mvbGoodFeature.push_back(false);
            pFrame_->mvbJacobBuilt.push_back(false);
            pFrame_->mvpMatchScore.push_back(1.0);

            ++i;
        }

        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "GF_ORB_SLAM");
        ros::start();

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = C2_k1;
        DistCoef.at<float>(1) = C2_k2;
        DistCoef.at<float>(2) = 0;
        DistCoef.at<float>(3) = 0;

        tracker_ = new Tracking(pFrame_->mK, DistCoef);
    }

    // virtual void TearDown() will be called after each test is run.
    // You should define it if there is cleanup work to do.  Otherwise,
    // you don't have to provide it.
    //
    // virtual void TearDown() {
    // }

    // Declares the variables your tests want to use.
    Tracking * tracker_;
    Observability * obs_;
    Frame * pFrame_;


    double noise_level_inl = 1.5;
    double noise_level_out = 3.0;
};


TEST_F(TestRANSAC, PnP_Solution) {
    //
    //    std::cout << "outlier rejection in " << pFrame_->N << " lmks:" << std::endl;
    std::srand(std::time(nullptr));

    cv::Mat lmkMat(3, 1, CV_32F);
    MapPoint * oriMapPoints = new MapPoint[pFrame_->N];
    vector<GoodPoint> mpVec;
    vector<double> avgTimeArr, minTimeArr, maxTimeArr, diffArr, sizeArr, maxArr, minArr;

    vector<size_t> random_idx;
    for (size_t i=0; i<pFrame_->N; ++i)
        random_idx.push_back(i);

    vector<cv::KeyPoint> mvKeys_ori;

    for (size_t outlier_rt = 5; outlier_rt <= 30; outlier_rt += 5) {
        //    for (size_t outlier_rt = 0; outlier_rt <= 0; outlier_rt += 5) {
        double avgTimeCost = 0, minTimeCost = 99999, maxTimeCost = -1, avgSubsetDiff = 0, minSubsetDiff = 99999, maxSubsetDiff = -1;

        size_t repeat_no = 200; // 5000; // 10000; //
        int num_outlier = round( double(outlier_rt) / 100.0 * double(pFrame_->N) );
        //        std::cout << "num_outlier = " << num_outlier << std::endl;
        //
        for (size_t iter = 0; iter < repeat_no; iter ++) {

            //            std::cout << "-------------------------- Round " << iter
            //                      << " --------------------------" << std::endl;
            //
            if (iter % 20 == 0) {
                // re-create the map
                pFrame_->mvpMapPoints.clear();
                pFrame_->mvKeys.clear();
                size_t i = 0;
                while (i < pFrame_->N) {
                    //
                    lmkMat.at<float>(0,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
                    lmkMat.at<float>(1,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
                    lmkMat.at<float>(2,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8;

                    float u, v;
                    arma::mat proj_jacob;
                    obs_->project_Point_To_Frame(lmkMat, obs_->kinematic[1].Tcw, u, v, proj_jacob);

                    if (u < 0 || v < 0)
                        continue ;

                    // NOTE be careful about the oct level definition !!!
                    int oct_lvl = std::round( ( float(std::rand()) / float(RAND_MAX) ) * 7 );

                    // if visible, take the random lmk
                    oriMapPoints[i].SetWorldPos(lmkMat);
                    pFrame_->mvpMapPoints.push_back(&oriMapPoints[i]);
                    pFrame_->mvKeys.push_back(cv::KeyPoint(u + (( float(std::rand()) / float(RAND_MAX) ) * 2 - 1) * noise_level_inl,
                                                           v + (( float(std::rand()) / float(RAND_MAX) ) * 2 - 1) * noise_level_inl,
                                                           1.0,
                                                           -1,
                                                           0,
                                                           oct_lvl));

                    ++i;
                }

                //
                mvKeys_ori = pFrame_->mvKeys;
                //                std::cout << "finish set up frame!" << std::endl;
            }
            //
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::shuffle(random_idx.begin(), random_idx.end(), std::default_random_engine(seed));
            //
            pFrame_->mvKeys = mvKeys_ori;
            for (size_t i=0; i<num_outlier; ++i) {
                double out_fac = float(std::rand()) / float(RAND_MAX) * 2 - 1;
                if (out_fac < 0)
                    out_fac -= 2;
                else
                    out_fac += 2;
                pFrame_->mvKeys[random_idx[i]].pt.x += out_fac * noise_level_out;
                //
                out_fac = float(std::rand()) / float(RAND_MAX) * 2 - 1;
                if (out_fac < 0)
                    out_fac -= 2;
                else
                    out_fac += 2;
                pFrame_->mvKeys[random_idx[i]].pt.y += out_fac * noise_level_out;
            }
            //            std::cout << "selection start!" << std::endl;
            mpVec.clear();
            arma::wall_clock timer;
            timer.tic();
            tracker_->RanSACMatches(const_cast<const Frame*>(pFrame_), mpVec);
            //            std::cout << "selection done!" << std::endl;
            double curTimeCost = timer.toc();
            avgTimeCost += curTimeCost;
            if (curTimeCost > maxTimeCost)
                maxTimeCost = curTimeCost;
            if (curTimeCost < minTimeCost)
                minTimeCost = curTimeCost;

            std::vector<size_t> exp_inlier_idx;
            for (size_t i=0; i<mpVec.size(); ++i) {
                exp_inlier_idx.push_back(mpVec[i].idx);
            }
            std::sort(exp_inlier_idx.begin(), exp_inlier_idx.end());

            std::vector<size_t> true_inlier_idx(&random_idx[num_outlier], &random_idx[random_idx.size()]);
            std::sort(true_inlier_idx.begin(), true_inlier_idx.end());

            // compute the set diff between repeats
            std::vector<size_t> diff_set;
            std::set_difference(true_inlier_idx.begin(), true_inlier_idx.end(), exp_inlier_idx.begin(), exp_inlier_idx.end(),
                                std::inserter(diff_set, diff_set.begin()));
            avgSubsetDiff += diff_set.size();

            if (diff_set.size() > maxSubsetDiff)
                maxSubsetDiff = diff_set.size();
            if (diff_set.size() < minSubsetDiff)
                minSubsetDiff = diff_set.size();

            //            std::cout << "Different inlier selected at current repeat: " << diff_set.size() << std::endl;
            EXPECT_NEAR(diff_set.size(), 0, ceil( true_inlier_idx.size() * 0.2 ));
        }

        avgTimeCost = avgTimeCost / double(repeat_no);
        avgSubsetDiff = avgSubsetDiff / double(repeat_no);
        //        std::cout << "Average time cost of subset selection = " << avgTimeCost << std::endl;
        avgTimeArr.push_back(avgTimeCost);
        minTimeArr.push_back(minTimeCost);
        maxTimeArr.push_back(maxTimeCost);
        diffArr.push_back(avgSubsetDiff);
        sizeArr.push_back(pFrame_->N - num_outlier);
        maxArr.push_back(maxSubsetDiff);
        minArr.push_back(minSubsetDiff);
    }

    std::cout << std::endl << "Summay: " << std::endl;
    std::cout << "Inlier / Full: " << std::endl;
    for (size_t i=0; i<sizeArr.size(); ++i)
        std::cout << sizeArr[i] << "/" << pFrame_->N << " ";
    std::cout << std::endl;

    std::cout << "Avg. Time Cost: " << std::endl;
    for (size_t i=0; i<avgTimeArr.size(); ++i)
        std::cout << avgTimeArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Max. Time Cost: " << std::endl;
    for (size_t i=0; i<maxTimeArr.size(); ++i)
        std::cout << maxTimeArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Min. Time Cost: " << std::endl;
    for (size_t i=0; i<minTimeArr.size(); ++i)
        std::cout << minTimeArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Avg. Inlier Diff: " << std::endl;
    for (size_t i=0; i<diffArr.size(); ++i)
        std::cout << diffArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Max. Inlier Diff: " << std::endl;
    for (size_t i=0; i<maxArr.size(); ++i)
        std::cout << maxArr[i] << " ";
    std::cout << std::endl;
    //
    std::cout << "Min. Inlier Diff: " << std::endl;
    for (size_t i=0; i<minArr.size(); ++i)
        std::cout << minArr[i] << " ";
    std::cout << std::endl << std::endl;

}

}  // namespace
