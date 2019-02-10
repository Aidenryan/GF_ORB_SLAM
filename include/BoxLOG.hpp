#ifndef BOXLOG_HPP
#define BOXLOG_HPP

#include <exception>
#include <cmath>

// OpenCV

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>

// Armadillo
#define ARMA_NO_DEBUG
#include "armadillo"


#define USE_INTEGRAL_IMAGE


namespace ORB_SLAM
{

/*=========================== BoxLoG Detector ==========================*/
/**
 * @brief The class for performing BoxLoG filtering and pattern dot detection.
 */
class BoxLoGDetector {
public:
    /**
     * Main functionality
     *    Input:
     *          img0: input gray-scale image
     *          radLoG: the corresponding real LoG radius
     *
     *    Output:
     *          img: BoxLoG response
     *
     * */
    /*================================= run ================================*/
    /*
      Apply the BoxLoG filter to an image.
      TODO: Does not use integral images, but rather OpenCV's boxFilter
      TODO:   implementation.  Would be nice to move to box filter to
      TODO:   integral image formulation.  At least if it improves speed.
      TODO:   Needs to be tested out.
    */
    inline void run(const cv::Mat & rawimg_,
                    std::vector<cv::KeyPoint> &keypoints_,
                    const double radLoG = 2,
                    const float threshold = 20.0f) {

        int blurKernelSize = 1;
        //        float sigma = sqrt( std::max(1.6*1.6 - 0.5f*0.5f, 0.01f) );

        if (rawimg_.depth() != CV_32F) {
            // THIS CV_32F CONVERTION is VITAL !!
            rawimg_.convertTo(mGray_, CV_32F);

            // Gaussian blur for anti-aliasing
            //    double floorRadLoG = static_cast<double>(static_cast<int>(radLoG));
            //    int blurKernelSize = static_cast<int>( ((radLoG - floorRadLoG) < 0.5)? (floorRadLoG + 0.1) : (floorRadLoG + 1.1));
            cv::GaussianBlur(mGray_, mGray_, cv::Size(blurKernelSize, blurKernelSize), 0, 0);
        }
        else {
            cv::GaussianBlur(rawimg_, mGray_, cv::Size(blurKernelSize, blurKernelSize), 0, 0);
        }

        //        // For small/large dots, invert the image to get pos response @ blobs.
        //        std::cout << "BoxLoG: track level = " << trackLevel_ << std::endl;
        //        if (trackLevel_ ==  DOT_SMALL|| trackLevel_ == DOT_LARGE)
        //            img0 = cv::Scalar::all(255) - img0;

        // Compute BoxLoG parameters
        float flL1, flL2, flL3, warpR;
        int R1, R2;
        //        arma::mat blkSum, blkSumBL;
        //        getRealLoG_(radLoG,blkSum,warpR,flL1,flL2,flL3,R1,R2,blkSumBL);
        getLoGCoeff_(radLoG,flL1,flL2,flL3,R1,R2,warpR);
        int bdRng = std::max(R1, R2);

#ifdef USE_INTEGRAL_IMAGE
        // compute integral image
        cv::integral(mGray_, mInt_, CV_32F);
        //
        //        float p1 = flL1-2*flL2+flL3, p2 = flL2-2*flL3;
        responceFromIntegral(flL1-flL2, flL2-flL3, flL3, R1, R2, warpR, bdRng);
#else
        // Compute BoxLoG response
        cv::Mat imgbf1, imgbf2, imgbf3;
        cv::boxFilter(mGray_, imgbf1,  -1 , cv::Size(2*R1+1,2*R1+1),      cv::Point( -1, -1 ), false);
        cv::boxFilter(mGray_, imgbf2,  -1 , cv::Size(2*R2+1,2*R2+1),      cv::Point( -1, -1 ), false);
        cv::boxFilter(mGray_, imgbf3,  -1 , cv::Size(warpR*2+1,warpR*2+1),cv::Point( -1, -1 ), false);
        mResponce_ = imgbf1*(flL1-flL2)+imgbf2*(flL2-flL3)+imgbf3*flL3;
#endif

        //        // Detect Local Maximum Pixels
        //        //                detectFromResp_(keypoints_, toPatchDistThreshold, nonMaxSuppThreshold);

        findScaleSpaceExtrema(threshold, bdRng, keypoints_);

        //        cv::KeyPointsFilter::removeDuplicatedSorted( keypoints_ );

        //        if( nfeatures > 0 )
        //            cv::KeyPointsFilter::retainBest(keypoints_, nfeatures);
    }



    /*============================== imageSC2 ==============================*/
    /*
      Output an image to a named window.
    */
    void vizResponce(const std::string responceWindow)
    {
        // draw responce image
        cv::Mat cavans_1;
        mResponce_.copyTo(cavans_1);
        cv::namedWindow(responceWindow, 1);
        cv::imshow(responceWindow, cavans_1);

        cv::waitKey();
    }


    void vizKeypoints(const std::string keypointWindow,
                      const cv::Mat &rawimg_,
                      const std::vector<cv::KeyPoint> &keypoints_)
    {
        // draw keypoint image
        cv::Mat cavans_2;
        rawimg_.copyTo(cavans_2);
        cv::cvtColor(cavans_2, cavans_2, cv::COLOR_GRAY2BGR);
        for (int i=0; i<keypoints_.size(); ++i) {
            cv::circle(cavans_2, keypoints_[i].pt, 2.0, cv::Scalar(0,255,0));
        }
        cv::namedWindow(keypointWindow, 1);
        cv::imshow(keypointWindow, cavans_2);

        cv::waitKey();
    }



private:


    cv::Mat mGray_;
    cv::Mat mResponce_;

#ifdef USE_INTEGRAL_IMAGE
    cv::Mat mInt_;
#endif

    /*============================= getRealLoG =============================*/
    /*
      Not sure.  Either gets the real LoG filter, or computes the best
      piecewise approximation for a 3 piece model.
    */
    inline void getRealLoG_(const double rad,
                            arma::mat & blkSum,
                            float & warpR,
                            float & L1,
                            float & L2,
                            float & L3,
                            int & R,
                            int & R2,
                            arma::mat blkSumBL) {

        double sigma = rad/sqrt(2);
        warpR= ceil(3*sigma)+1;
        int N=int(2*warpR+1);

        arma::mat xc=arma::linspace(-warpR, warpR, N);
        //    xc.t().print("xc=");

        arma::mat klog=arma::zeros(N,N);
        double rsq=0;
        double sigma2=sigma*sigma;
        for ( int i=0;i<N;++i)
        {
            for ( int j=0; j<N; ++j)
            {
                rsq = xc(i)*xc(i)+xc(j)*xc(j);
                klog(i,j) = -1*((rsq-2*sigma2)/(2*3.1415*sigma2*sigma2))
                        * exp(-1*rsq/(2*sigma2));
            }
        }

        blkSum=arma::zeros(warpR+1,1);
        for ( uint sumR =0; sumR<warpR+1; ++sumR)
            blkSum(sumR,0) = sum(sum(klog.submat(arma::span((warpR-sumR),(warpR+sumR)),
                                                 arma::span((warpR-sumR),(warpR+sumR)))));

        //--------------------------------------------------------
        // Get the BoxLoG Filter
        int maxIdx=0; // cannot directly use fix(rad) because some dicretize artifacts
        for ( uint i=1; i<warpR+1;++i)
            maxIdx=(blkSum(maxIdx)<blkSum(i)? i:maxIdx);
        maxIdx++;

        int i_cand=round((3*maxIdx+1)/4);

        // build BoxLoG
        R=i_cand-1;
        int i2=2*maxIdx-i_cand;
        R2=i2-1;
        double a1=(R*2+1)*(R*2+1);
        double a2=(R2*2+1)*(R2*2+1);

        //! outer band
        arma::mat BoxLoG(N,N);
        L3=arma::as_scalar(-blkSum(i2-1)/((2*warpR+1)*(2*warpR+1)-a2));
        BoxLoG.fill(L3);

        //! middle band
        L2=arma::as_scalar(blkSum(i2-1)-blkSum(i_cand-1))/(a2-a1);
        BoxLoG.submat(arma::span((warpR-R2), (warpR+R2)),arma::span((warpR-R2),(warpR+R2))).fill(L2);

        //! inner square
        L1=arma::as_scalar(blkSum(i_cand-1)/a1);
        BoxLoG.submat(arma::span((warpR-R),(warpR+R)),arma::span((warpR-R),(warpR+R))).fill(L1);

        blkSumBL=arma::zeros(warpR+1,1);
        for ( uint sumR =0; sumR<warpR+1; ++sumR)
            blkSumBL(sumR,0)=sum(sum(BoxLoG.submat(arma::span((warpR-sumR),(warpR+sumR)),arma::span((warpR-sumR),(warpR+sumR)))));

    }

    //
    inline void getLoGCoeff_(const double rad,
                             float & L1,
                             float & L2,
                             float & L3,
                             int & R1,
                             int & R2,
                             float & warpR) {

        double sigma = rad/sqrt(2);
        warpR= ceil(3*sigma)+1;
        int N=int(2*warpR+1);

        arma::mat xc=arma::linspace(-warpR, warpR, N);

        arma::mat klog=arma::zeros(N,N);
        double rsq=0;
        double sigma2=sigma*sigma;
        for ( int i=0;i<N;++i)
        {
            for ( int j=0; j<N; ++j)
            {
                rsq = xc(i)*xc(i)+xc(j)*xc(j);
                klog(i,j) = -1*((rsq-2*sigma2)/(2*3.1415*sigma2*sigma2))
                        * exp(-1*rsq/(2*sigma2));
            }
        }

        arma::mat blkSum=arma::zeros(warpR+1,1);
        for ( uint sumR =0; sumR<warpR+1; ++sumR)
            blkSum(sumR,0) = sum(sum(klog.submat(arma::span((warpR-sumR),(warpR+sumR)),
                                                 arma::span((warpR-sumR),(warpR+sumR)))));

        //--------------------------------------------------------
        // Get the BoxLoG Filter
        int maxIdx=0; // cannot directly use fix(rad) because some dicretize artifacts
        for ( uint i=1; i<warpR+1;++i)
            maxIdx=(blkSum(maxIdx)<blkSum(i)? i:maxIdx);
        maxIdx++;

        int i_cand=round((3*maxIdx+1)/4);

        // build BoxLoG
        R1=i_cand-1;
        int i2=2*maxIdx-i_cand;
        R2=i2-1;
        double a1=(R1*2+1)*(R1*2+1);
        double a2=(R2*2+1)*(R2*2+1);

        //! outer band
        L3=arma::as_scalar(-blkSum(i2-1)/((2*warpR+1)*(2*warpR+1)-a2));

        //! middle band
        L2=arma::as_scalar(blkSum(i2-1)-blkSum(i_cand-1))/(a2-a1);

        //! inner square
        L1=arma::as_scalar(blkSum(i_cand-1)/a1);

    }


    //
    inline void findScaleSpaceExtrema(const float threshold,
                                      const int border_range,
                                      std::vector<cv::KeyPoint> &keypoints_) {

        //        const int BORDER_RANGE = 3;
        const int step = (int)mResponce_.step1();

        cv::KeyPoint kpt;
        for( int r = border_range; r < mResponce_.rows - border_range; r++)
        {
            const float* currptr = mResponce_.ptr<float>(r);

            for( int c = border_range; c < mResponce_.cols - border_range; c++)
            {
                float val = currptr[c];

                // find local extrema with pixel accuracy
                if( std::fabs(val) > threshold &&
                        ((val > 0 && val >= currptr[c-1] && val >= currptr[c+1] &&
                          val >= currptr[c-step-1] && val >= currptr[c-step] && val >= currptr[c-step+1] &&
                          val >= currptr[c+step-1] && val >= currptr[c+step] && val >= currptr[c+step+1] ) ||
                         (val < 0 && val <= currptr[c-1] && val <= currptr[c+1] &&
                          val <= currptr[c-step-1] && val <= currptr[c-step] && val <= currptr[c-step+1] &&
                          val <= currptr[c+step-1] && val <= currptr[c+step] && val <= currptr[c+step+1] )))
                {
                    kpt.pt.x = c;
                    kpt.pt.y = r;
                    kpt.response = std::fabs(val);

                    keypoints_.push_back(kpt);
                }
            }
        }
    }


#ifdef USE_INTEGRAL_IMAGE
    inline void responceFromIntegral(const float p1,
                                     const float p2,
                                     const float p3,
                                     const int R1,
                                     const int R2,
                                     const int R3,
                                     const int border_range) {

        mResponce_ = cv::Mat::zeros(mInt_.rows, mInt_.cols, CV_32F);

        //        const float norm_fac = float((2*R1+1) * (2*R1+1) + (2*R2+1) * (2*R2+1)) * 2;
        const int step = (int)mInt_.step1();
        for( int r = border_range; r < mInt_.rows - border_range; r++) {
            const float* currptr = mInt_.ptr<float>(r);
            float* resptr = mResponce_.ptr<float>(r);
            for( int c = border_range; c < mInt_.cols - border_range; c++) {
                //
                resptr[c] = (
                            (p1 * currptr[c + R1 + (R1) * step] +
                            p2 * currptr[c + R2 + (R2) * step] +
                        p3 * currptr[c + R3 + (R3) * step]) -
                        (p1 * currptr[c + R1 + (- R1) * step] +
                        p2 * currptr[c + R2 + (- R2) * step] +
                        p3 * currptr[c + R3 + (-R3) * step]) -
                        (p1 * currptr[c - R1 + (R1) * step] +
                        p2 * currptr[c - R2 + (R2) * step] +
                        p3 * currptr[c - R3 + (R3) * step]) +
                        (p1 * currptr[c - R1 + (- R1) * step] +
                        p2 * currptr[c - R2 + (- R2) * step] +
                        p3 * currptr[c - R3 + (- R3) * step])
                        ); // / norm_fac;
            }
        }

        //        double min_, max_;
        //        cv::minMaxLoc(mResponce_, &min_, &max_);
        //        mResponce_ = mResponce_ / float(std::max(fabs(min_), fabs(max_))) * 255.0f;
        //        cv::normalize(mResponce_, mResponce_, 255.0, 0, cv::NORM_MINMAX);

        //        for( int r = border_range; r < mInt_.rows - border_range; r++) {
        //            for( int c = border_range; c < mInt_.cols - border_range; c++) {
        //                //
        //                mResponce_.at<CV_32F>(r, c) =
        //                        (p1 * mInt_.at<CV_32F>(c + R1, r + R1) +
        //                         p2 * mInt_.at<CV_32F>(c + R2, r + R2)) -
        //                        (p1 * mInt_.at<CV_32F>(c + R1, r - R1) +
        //                         p2 * mInt_.at<CV_32F>(c + R2, r - R2)) -
        //                        (p1 * mInt_.at<CV_32F>(c - R1, r + R1) +
        //                         p2 * mInt_.at<CV_32F>(c - R2, r + R2)) +
        //                        (p1 * mInt_.at<CV_32F>(c - R1, r - R1) +
        //                         p2 * mInt_.at<CV_32F>(c - R2, r - R2));
        //            }
        //        }
    }
#endif

};

}

#endif // BOXLOG_HPP
