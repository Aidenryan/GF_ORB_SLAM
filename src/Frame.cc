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

#include "Frame.h"
#include "Converter.h"

#include <ros/ros.h>

namespace ORB_SLAM
{
long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy;
int Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor), im(frame.im.clone()), mTimeStamp(frame.mTimeStamp),
     mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()), N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn),
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), mDescriptors(frame.mDescriptors.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier),
     mfGridElementWidthInv(frame.mfGridElementWidthInv), mfGridElementHeightInv(frame.mfGridElementHeightInv),mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels), mfScaleFactor(frame.mfScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        mTcw = frame.mTcw.clone();
}


Frame::Frame(cv::Mat &im_, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef)
    :mpORBvocabulary(voc),mpORBextractor(extractor), im(im_),mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone())
{
    // Exctract ORB  
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);

    N = mvKeys.size();

    if(mvKeys.empty()) {
        std::cout << "no key points being detected!" << std::endl;
        return;
    }

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mvpMatchScore = vector<int>(N,static_cast<int>(999));

    //
    // TODO
    // replace the default opencv distortion model with kilabr compatable impl, as found from here:
    // https://github.com/schneith/undistorter
    //
    UndistortKeyPoints();


    // This is done for the first created Frame
    if(mbInitialComputations)
    {
        ComputeImageBounds();

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);

        mbInitialComputations=false;
    }


    mnId=nNextId++;    

    //Scale Levels Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();

    mvScaleFactors.resize(mnScaleLevels);
    mvLevelSigma2.resize(mnScaleLevels);
    mvScaleFactors[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<mnScaleLevels; i++)
    {
        mvScaleFactors[i]=mvScaleFactors[i-1]*mfScaleFactor;        
        mvLevelSigma2[i]=mvScaleFactors[i]*mvScaleFactors[i];
    }

    mvInvLevelSigma2.resize(mvLevelSigma2.size());
    for(int i=0; i<mnScaleLevels; i++)
        mvInvLevelSigma2[i]=1/mvLevelSigma2[i];

    // Assign Features to Grid Cells
    int nReserve = 0.5*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);


    for(size_t i=0;i<mvKeysUn.size();i++)
    {
        cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }


    mvbOutlier = vector<bool>(N,false);

    mvbCandidate = vector<bool>(N,true);

    mvbJacobBuilt = vector<bool>(N,false);

    //
    mvbGoodFeature = vector<bool>(N,false);
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

//
cv::Mat Frame::getTwc() {
    //
    cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*mTcw.rowRange(0,3).col(3);
    //
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);

    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));

    return Twc;
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float PcX = Pc.at<float>(0);
    const float PcY = Pc.at<float>(1);
    const float PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale level acording to the distance
    float ratio = dist/minDistance;

    vector<float>::iterator it = lower_bound(mvScaleFactors.begin(), mvScaleFactors.end(), ratio);
    int nPredictedLevel = it-mvScaleFactors.begin();

    if(nPredictedLevel>=mnScaleLevels)
        nPredictedLevel=mnScaleLevels-1;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}


void Frame::getProjectError(MapPoint * pt3D, cv::KeyPoint * pt2D, float & d_u, float & d_v) {

        // init error variable
        d_u = FLT_MAX;
        d_v = FLT_MAX;
    //    std::cout << "image coord = "
    //              << pt2D->pt.x << ", "
    //              << pt2D->pt.y << std::endl;

        // 3D in absolute coordinates
        cv::Mat P = pt3D->GetWorldPos();
    //    std::cout << "world coord = "
    //              << P.at<float>(0) << ", "
    //              << P.at<float>(1) << ", "
    //              << P.at<float>(2) << std::endl;

        // 3D in camera coordinates
        cv::Mat mRcw_tmp = this->mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat mtcw_tmp = this->mTcw.rowRange(0,3).col(3);

        const cv::Mat Pc = mRcw_tmp * P + mtcw_tmp;
        const float PcX = Pc.at<float>(0);
        const float PcY = Pc.at<float>(1);
        const float PcZ = Pc.at<float>(2);
    //    std::cout << "cam coord = "
    //              << PcX << ", "
    //              << PcY << ", "
    //              << PcZ << std::endl;

        // Check positive depth
        if(PcZ<0.0)
            return ;

        // Project in image and check it is not outside
        const float invz = 1.0/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return ;
        if(v<mnMinY || v>mnMaxY)
            return ;

    //    // Check distance is in the scale invariance region of the MapPoint
    //    const float maxDistance = pMP->GetMaxDistanceInvariance();
    //    const float minDistance = pMP->GetMinDistanceInvariance();
    //    const cv::Mat PO = P-mOw;
    //    const float dist = cv::norm(PO);

    //    if(dist<minDistance || dist>maxDistance)
    //        return ;

    //   // Check viewing angle
    //    cv::Mat Pn = pMP->GetNormal();

    //    float viewCos = PO.dot(Pn)/dist;

    //    if(viewCos<0.5)
    //        return ;

        // Compute the distance between pixel <u,v> & the input pt2D
        //    pErr = sqrt(pow(pt2D->pt.x - u, 2) + pow(pt2D->pt.y - v, 2));
        d_u = pt2D->pt.x - u;
        d_v = pt2D->pt.y - v;
    //    std::cout << "proj pixel = [" << pt2D->pt.x << ", " << pt2D->pt.y << "]; meas. pixel = [" << u << ", " << v << "]" << std::endl;

        return ;
    }


vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, int minLevel, int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = min(FRAME_GRID_COLS-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(FRAME_GRID_ROWS-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    bool bCheckLevels=true;
    bool bSameLevel=false;
    if(minLevel==-1 && maxLevel==-1)
        bCheckLevels=false;
    else
        if(minLevel==maxLevel)
            bSameLevel=true;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels && !bSameLevel)
                {
                    if(kpUn.octave<minLevel || kpUn.octave>maxLevel)
                        continue;
                }
                else if(bSameLevel)
                {
                    if(kpUn.octave!=minLevel)
                        continue;
                }

                if(abs(kpUn.pt.x-x)>r || abs(kpUn.pt.y-y)>r)
                    continue;

                vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;

}

bool Frame::PosInGrid(cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(mvKeys.size(),2,CV_32F);
    for(unsigned int i=0; i<mvKeys.size(); i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(mvKeys.size());
    for(unsigned int i=0; i<mvKeys.size(); i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;

//        std::cout << "undist pt " << i
//                  << " = < " << mvKeysUn[i].pt.x << ", "
//                  << mvKeysUn[i].pt.y << " >" << std::endl;
    }
}

void Frame::ComputeImageBounds()
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
//        cv::Mat mat(4,2,CV_32F);
//        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
//        mat.at<float>(1,0)=im.cols; mat.at<float>(1,1)=0.0;
//        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=im.rows;
//        mat.at<float>(3,0)=im.cols; mat.at<float>(3,1)=im.rows;

//        // Undistort corners
//        mat=mat.reshape(2);
//        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
//        mat=mat.reshape(1);

//        mnMinX = min(floor(mat.at<float>(0,0)),floor(mat.at<float>(2,0)));
//        mnMaxX = max(ceil(mat.at<float>(1,0)),ceil(mat.at<float>(3,0)));
//        mnMinY = min(floor(mat.at<float>(0,1)),floor(mat.at<float>(1,1)));
//        mnMaxY = max(ceil(mat.at<float>(2,1)),ceil(mat.at<float>(3,1)));

        cv::Mat mat(8,2,CV_32F);
        mat.at<float>(0,0)=0.0;
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=im.cols;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0;
        mat.at<float>(2,1)=im.rows;
        mat.at<float>(3,0)=im.cols;
        mat.at<float>(3,1)=im.rows;
        //
        mat.at<float>(4,0)=0.0;
        mat.at<float>(4,1)=float(im.rows) / 2.0;
        mat.at<float>(5,0)=float(im.cols) / 2.0;
        mat.at<float>(5,1)=0.0;
        mat.at<float>(6,0)=float(im.cols) / 2.0;
        mat.at<float>(6,1)=im.rows;
        mat.at<float>(7,0)=im.cols;
        mat.at<float>(7,1)=float(im.rows) / 2.0;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = INT_MAX;
        mnMaxX = INT_MIN;
        mnMinY = INT_MAX;
        mnMaxY = INT_MIN;
        //
        for (int i=0; i<mat.rows; ++i) {
            if (mnMinX > mat.at<float>(i,0))
                mnMinX = floor( mat.at<float>(i,0) );
            if (mnMinY > mat.at<float>(i,1))
                mnMinY = floor( mat.at<float>(i,1) );
            //
            if (mnMaxX < mat.at<float>(i,0))
                mnMaxX = ceil( mat.at<float>(i,0) );
            if (mnMaxY < mat.at<float>(i,1))
                mnMaxY = ceil( mat.at<float>(i,1) );
        }

    }
    else
    {
        mnMinX = 0;
        mnMaxX = im.cols;
        mnMinY = 0;
        mnMaxY = im.rows;
    }

    std::cout << "func ComputeImageBounds: image bound after undistortion = [ "
              << mnMinX << ", "
              << mnMaxX << ", "
              << mnMinY << ", "
              << mnMaxY << " ]" << std::endl;
}

} //namespace ORB_SLAM
