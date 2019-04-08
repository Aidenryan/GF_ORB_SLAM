/**
* This file is part of GF-ORB-SLAM.
*
* Copyright (C) 2019 Yipu Zhao <yipu dot zhao at gatech dot edu> 
* (Georgia Institute of Technology)
* For more information see 
* <https://sites.google.com/site/zhaoyipu/good-feature-visual-slam>
*
* GF-ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GF-ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GF-ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <iostream>
#include <fstream>
#include "Observability.h"

using namespace ORB_SLAM;

void SetUp(const size_t num_features, Observability * & obs_, Frame * & pFrame_) {

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
    double rand_xy = ((double)rand() / (RAND_MAX)) * 0.4 - 0.2,
            rand_z = ((double)rand() / (RAND_MAX)) * 0.8 - 0.4;
    obs_->Xv << 0.3000 + rand_xy << -0.1000 + rand_xy <<   1.0000 + rand_z  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
             <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
    //    obs_->Xv << 0.3000  << -0.1000 <<   1.0000  <<  0.9992  <<  0.0131  <<  0.0314 <<  -0.0209
    //             <<  3.0000 <<  -1.0000 <<  10.0000  <<  0.2618  <<  0.6283 <<  -0.4189;
    obs_->predictPWLSVec(0.1, 1);

    //        std::cout << "fill in map and measurement variables" << std::endl;
    // fill in necessary members
    pFrame_ = new Frame();
    pFrame_->N = num_features; // 200; // 160; // 280; // 100; // 80; // 240; // 120; // 60; //
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

    cv::Mat lmkMat(3, 1, CV_32F);
    MapPoint * oriMapPoints = new MapPoint[pFrame_->N];

    //
    size_t i = 0;
    while (i < pFrame_->N) {
        //
        lmkMat.at<float>(0,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
        lmkMat.at<float>(1,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8 - 4;
        lmkMat.at<float>(2,0) = ( float(std::rand()) / float(RAND_MAX) ) * 8;

        float u, v;
        arma::mat proj_jacob;
        obs_->project_Point_To_Frame(lmkMat, obs_->kinematic[0].Tcw, u, v, proj_jacob);
        if (u < 0 || v < 0)
            continue ;

        // NOTE be careful about the oct level definition !!!
        int oct_lvl = std::round( ( float(std::rand()) / float(RAND_MAX) ) * 7 );

        // if visible, take the random lmk
        oriMapPoints[i].SetWorldPos(lmkMat);
        pFrame_->mvpMapPoints.push_back(&oriMapPoints[i]);
        pFrame_->mvKeysUn.push_back(cv::KeyPoint(u + ( float(std::rand()) / float(RAND_MAX) ) * 2 - 1,
                                                 v + ( float(std::rand()) / float(RAND_MAX) ) * 2 - 1,
                                                 1.0,
                                                 -1,
                                                 0,
                                                 oct_lvl));

        //            cv::Mat lPt = pFrame_->mvpMapPoints[i]->GetWorldPos();
        //            std::cout << "Spawn map point at "
        //                      << lPt.at<float>(0,0) << ", "
        //                      << lPt.at<float>(1,0) << ", "
        //                      << lPt.at<float>(2,0) << std::endl;

        //            std::cout << "Corresponding projection at "
        //                      << pFrame_->mvKeysUn[i].pt.x << ", "
        //                      << pFrame_->mvKeysUn[i].pt.y << std::endl;

        pFrame_->mvbOutlier.push_back(false);
        pFrame_->mvbCandidate.push_back(true);
        pFrame_->mvbJacobBuilt.push_back(false);
        pFrame_->mvbGoodFeature.push_back(false);
        pFrame_->mvpMatchScore.push_back(std::round( ( float(std::rand()) / float(RAND_MAX) ) * 100 ));

        ++i;
    }

}

void GetMetricAndIdx(const vector<GoodPoint> & mpVec, double & curDet, vector<size_t> & subset_idx) {
    //
    arma::mat curMat = arma::zeros( size(mpVec[0].obs_block) );
    for (size_t i=0; i<mpVec.size(); ++i) {
        //                curMat = curMat + pFrame_->mvpMapPoints[mpVec[i].idx]->ObsMat.t() * pFrame_->mvpMapPoints[mpVec[i].idx]->ObsMat;
        curMat = curMat + mpVec[i].obs_block;
    }
    curDet = ORB_SLAM::logDet( curMat );
    //            std::cout << "-------------------------- Round " << iter
    //                      << ": " << "subset logDet = " << curDet
    //                      << " --------------------------" << std::endl;

    subset_idx.clear();
    for (size_t i=0; i<mpVec.size(); ++i) {
        subset_idx.push_back(mpVec[i].idx);
    }
    std::sort(subset_idx.begin(), subset_idx.end());
}

template <class T>
void SaveToTextFile(const T (*mat)[8][161], const int sz_1, const int sz_2, const int sz_3, std::string filename) {
    ofstream myfile (filename);
    if (myfile.is_open()) {
        for (int i=0; i<sz_1; ++i) {
            for (int j=0; j<sz_2; ++j) {
                for (int k=0; k<sz_3; ++k) {
                    //
                    myfile << mat[i][j][k] << "\n" ;
                }
            }
        }
        myfile.close();
    }
}

int main() {
    //
    std::srand(std::time(nullptr));

    std::vector<double> errBound_def {1.0, 0.5, 0.2, 0.1, 0.05, 0.025, 0.01, 0.005};

    // random iter
    size_t world_repeat_no = 100; // 20; //
    for (size_t count_world = 1; count_world <= world_repeat_no; count_world ++) {
        std::cout << "start simulating world number " << count_world << "!" << std::endl;
        double timeCost_summary[15][8][161];
        double logDet_summary[15][8][161];
        size_t diffFeat_summary[15][8][161];
        int i=0, j=0, k=0;
        // map config
        for (size_t num_feature = 100; num_feature <= 3000; num_feature += 200) {
            // create the world
            Observability * obs_ = NULL;
            Frame * pFrame_ = NULL;
            SetUp(num_feature, obs_, pFrame_);
            obs_->mKineIdx = 0;
//           std::cout << obs_->Xv << std::endl;
            // subset config
            j=0;
            for (double num_good_inlier = double(num_feature) * 0.1; num_good_inlier <= double(num_feature) * 0.8; num_good_inlier += double(num_feature) * 0.1) {
                //
                std::cout << "======================= " << num_good_inlier << " / " << num_feature << " =======================" << std::endl;
                vector<GoodPoint> mpVec;
                double logDet_base, logDet_curr;
                vector<size_t> subIdx_base, subIdx_curr;
                size_t greedy_mtd;
                arma::wall_clock timer;

                // baseline greedy
                mpVec.clear();
                obs_->lmkSelectPool.clear();
                pFrame_->mvbJacobBuilt = vector<bool>(pFrame_->N, false);
                greedy_mtd = 1;
                double time_for_select = 999; // 0.004; // 0.002; // 0.001; //
                k=0;
                //
                timer.tic();
                obs_->setSelction_Number(size_t(num_good_inlier), greedy_mtd, time_for_select, 1.0, pFrame_, &mpVec);
                double time_base = timer.toc();
                std::cout << "Time cost of base greedy = " << time_base << std::endl;
                // collect the statistics
                GetMetricAndIdx(mpVec, logDet_base, subIdx_base);
                timeCost_summary[i][j][k] = time_base;
                logDet_summary[i][j][k] = logDet_base;
                diffFeat_summary[i][j][k] = 0;
                ++ k;

                // lazier greedy repeat test
                size_t sub_repeat_no = 20; // 10; //
                for (size_t errn = 0; errn < errBound_def.size(); ++ errn) {
                    double error_bound = errBound_def[errn];
                    for (size_t iter = 0; iter < sub_repeat_no; iter ++) {
                        mpVec.clear();
                        obs_->lmkSelectPool.clear();
                        pFrame_->mvbJacobBuilt = vector<bool>(pFrame_->N, false);
                        greedy_mtd = 2; // 3; // 4; //
                        //
                        timer.tic();
                        bool flagSucc = obs_->setSelction_Number(size_t(num_good_inlier), greedy_mtd, time_for_select, error_bound, pFrame_, &mpVec);
                        if (flagSucc == false) {
                            std::cout << "selection failure! most likely due to time constraint!" << std::endl;
                            //                    iter --;
                            //                    continue ;
                        }
                        double time_curr = timer.toc();
                        std::cout << "Time cost of lazier greedy = " << time_curr << std::endl;

                        // collect the statistics
                        GetMetricAndIdx(mpVec, logDet_curr, subIdx_curr);
                        // compare the difference wrt base greedy
                        std::vector<size_t> diff_set;
                        std::set_difference(subIdx_base.begin(), subIdx_base.end(), subIdx_curr.begin(), subIdx_curr.end(),
                                            std::inserter(diff_set, diff_set.begin()));
                        timeCost_summary[i][j][k] = time_curr;
                        logDet_summary[i][j][k] = logDet_curr;
                        diffFeat_summary[i][j][k] = diff_set.size();
                        ++ k;
                    }
                }
                //
                ++ j;
            }
            //
            ++ i;
            // delete the world
            delete obs_;
            delete pFrame_;
        }

        // save the summarized results
        SaveToTextFile<double>(timeCost_summary, 15, 8, 161, std::string("./tools/simu/world_") + std::to_string(count_world) + std::string("_TimeCost.txt"));
        SaveToTextFile<double>(logDet_summary, 15, 8, 161, std::string("./tools/simu/world_") + std::to_string(count_world) + std::string("_LogDet.txt"));
        SaveToTextFile<size_t>(diffFeat_summary, 15, 8, 161, std::string("./tools/simu/world_") + std::to_string(count_world) + std::string("_DiffFeat.txt"));
    }

    return 0;
}
