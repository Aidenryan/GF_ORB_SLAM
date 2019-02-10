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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/dense/linear_solver_dense.h"

#include <Eigen/StdVector>

#include "Converter.h"


namespace ORB_SLAM
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag=NULL);
    int static PoseOptimization(Frame* pFrame);

    int static PoseOptimization(Frame* pFrame, int &N_MP_Used);
    float static PoseOptimization2(Frame* pFrame);

    float static PoseOptimizationOOMC(Frame* pFrame, const std::vector<size_t> Triplet2Use, cv::Mat &pose);
    float static PoseOptimizationOOMC_2(Frame *pFrame, const std::set<int> & PtsIdx2Use);

    int static PoseOptimization_GFSLAM(Frame *pFrame, const vector<GoodPoint> & mpSorted, const double & ObsThreshold, int & N_MP_Used);
    int static PoseOptimization_Selected(Frame *pFrame, const vector<GoodPoint> & mpSorted);
    int static PoseOptimization_RANSAC(Frame *pFrame, const vector<GoodPoint> & mpSorted);

    float static GetPoseGraph(Frame *pFrame, g2o::SparseOptimizer & optimizer, vector<g2o::EdgeSE3ProjectXYZ*>& vpEdges, int &nInitial0);
    float static EvaluateInlierRatio(Frame *pFrame, g2o::SparseOptimizer & optimizer, cv::Mat & Pose2Evaluate,
                                     vector<g2o::EdgeSE3ProjectXYZ*>& vpEdges, int & nInitialCorrespondences);


    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       std::map<KeyFrame*, set<KeyFrame*> > &LoopConnections);


    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2 = 10);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
