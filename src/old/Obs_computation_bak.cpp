#include "Obs_computation.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <iosfwd>

//#define TIME_TMPRUPDATE_SORE

#define EPS 1e-6


namespace ORB_SLAM {

void cpp11()
{
    std::cout<<"C++11 MULTITHREADING\n";
}


void CallableObj(Frame* pFrame, double d, std::string const& s)
{

    std::cout<< s << " = " << d << std::endl;
    const int N = pFrame->mvpMapPoints.size();
    std::cout << "pFrame size: " << N << std::endl;

}

void Obs_computer::CallableObj_inside(Frame* pFrame, double d, std::string const& s)
{

    std::cout<< s << " = " << d << std::endl;
    const int N = pFrame->mvpMapPoints.size();
    std::cout << "pFrame size: " << N << std::endl;

}


/// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/// !!!!!
/// !!!
/// NEVER use reference as input for multi-threading kernels!!!!
void Obs_computer::testing_function(Frame* pFrame, int start_idx, int end_idx, double dt, int tmprLen,
                                     arma::mat Xv, arma::mat F_Q, arma::mat F_Omg)
{
    arma::mat H13;
    arma::mat H47;

    for(int i= start_idx; i<end_idx; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];

        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            // Feature position
            arma::mat Y = arma::zeros<arma::mat>(3,1);
            cv::Mat featPos = pMP->GetWorldPos();
            Y(0,0) = featPos.at<float>(0);
            Y(1,0) = featPos.at<float>(1);
            Y(2,0) = featPos.at<float>(2);

            // Measurement
            arma::mat Z = arma::zeros<arma::mat>(1,2);
            cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
            Z(0,0) = kpUn.pt.x;
            Z(0,1) = kpUn.pt.y;

            // Compute the H matrix

            compute_H_subblock(Xv, Y, Z, H13, H47);

            //std::cout << "[OBS_COMPUTOR]  LinObsMat" << std::endl;
            arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);
            // Copy first 3 columns
            LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
            // Columns 7~9
            LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
            if(fabs(dt - 1.0) > 1e-6) {
                LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
            }
            // First segment in linObsMat: just H:
            LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

            // Segment 2 to 13
            arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
            arma::mat rollingQFac = F_Q;  // Q^n
            for (int j = 1; j < 13; j++) {
                    // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                    // 2nd block:  (H47 * Q^n)
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                    // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                    // Update
                    rollingQAdd = rollingQAdd + rollingQFac;
                    rollingQFac = rollingQFac * F_Q;
            }

            // Update the SOM
            arma::mat SOM_old = pMP->ObsMat;
            int Len_SOM_old = static_cast<int>(SOM_old.n_rows);

            assert(Len_SOM_old% 26 == 0);
            if(Len_SOM_old == 0) {
                /// Case 1: first time observed, the SOM_old is an empty matrix
                pMP->ObsMat  = LinObsMat;
                pMP->ObsScore= 0.0;
                pMP->ObsRank = arma::rank(LinObsMat);
            }
            else if(Len_SOM_old == 26*tmprLen) {
                /// Case 2: phase 4, "fully loaded"
                // SOM block order: row 0 -- oldest, row End -- latest.

                arma::mat SOM = arma::zeros<arma::mat>(SOM_old.n_rows, SOM_old.n_cols);
                SOM.rows(0, 26*(tmprLen-1)-1) = SOM_old.rows(26, 26*tmprLen-1);
                SOM.rows(26*(tmprLen-1), 26*tmprLen-1) = LinObsMat;

                // Save
                arma::vec s = arma::svd(SOM);
                pMP->ObsMat  = SOM;
                pMP->ObsScore= s(12);
                pMP->ObsRank = arma::rank(SOM);
            }
            else if(Len_SOM_old< 26*tmprLen) {
                /// Case 3: phase 2/3, "NOT fully loaded, but has been initialized"
                arma::mat SOM = arma::join_vert(SOM_old, LinObsMat);
                pMP->ObsMat  = SOM;
                pMP->ObsRank = arma::rank(SOM);


                if (SOM.n_rows >= 32) {
                    // Full rank
                    arma::vec s = arma::svd(SOM);
                     pMP->ObsScore= s(12);
                } else {
                     pMP->ObsScore = 0.0;
                }
            }

        } //Perform regular temporal obs update!
     } // For: all pMP

}


// ==================================================================================
void Obs_computer::compute_F(const arma::mat & Xv, const double & dt, arma::mat & df_dxv) {

    // Update the quaternion
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // dq3_by_dq2
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);

    arma::mat dq3_by_dq2;
    dq3_by_dq2 << R << -X << -Y << -Z << arma::endr
               << X << R  <<  Z << -Y << arma::endr
               << Y << -Z <<  R <<  X << arma::endr
               << Z <<  Y << -X <<  R << arma::endr;

    // dq3_by_dq1
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);

    arma::mat dq3_by_dq1;

    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;

    // df_dxv
    df_dxv = arma::eye<arma::mat>(13, 13);
    df_dxv.submat(0, 7, 2, 9) = arma::eye(3,3) * dt;
    df_dxv.submat(3, 3, 6, 6) = dq3_by_dq2;
    df_dxv.submat(3, 10, 6, 12) = dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    return;
}

// ==================================================================================
arma::mat Obs_computer::DCM2QUAT(const cv::Mat& a){

    arma::mat q = arma::zeros<arma::mat>(4, 1);

    double trace = a.at<double>(0,0) + a.at<double>(1,1) + a.at<double>(2,2);
    if( trace > 0 ) {// I changed M_EPSILON to 0
      //std::cout<<"Condition 1\n";
      double s = 0.5f / sqrtf(trace+ 1.0f);
      q(0,0) = 0.25f / s;
      q(1,0) = ( a.at<double>(2,1) - a.at<double>(1,2) ) * s;
      q(2,0) = ( a.at<double>(0,2) - a.at<double>(2,0) ) * s;
      q(3,0) = ( a.at<double>(1,0) - a.at<double>(0,1) ) * s;
    } else {
      if ( a.at<double>(0,0) > a.at<double>(1,1) && a.at<double>(0,0) > a.at<double>(2,2) ) {
          //std::cout<<"Condition 2\n";
          double s = 2.0f * sqrtf( 1.0f + a.at<double>(0,0) - a.at<double>(1,1) - a.at<double>(2,2));
          q(0,0) = (a.at<double>(2,1) - a.at<double>(1,2) ) / s;
          q(1,0) = 0.25f * s;
          q(2,0) = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
          q(3,0) = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
        } else if (a.at<double>(1,1) > a.at<double>(2,2)) {
          //std::cout<<"Condition 3\n";
          double s = 2.0f * sqrtf( 1.0f + a.at<double>(1,1) - a.at<double>(0,0) - a.at<double>(2,2));
          q(0,0) = (a.at<double>(0,2) - a.at<double>(2,0) ) / s;
          q(1,0) = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
          q(2,0) = 0.25f * s;
          q(3,0) = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
        } else {
          //std::cout<<"Condition 4\n";
          double s = 2.0f * sqrtf( 1.0f + a.at<double>(2,2) - a.at<double>(0,0) - a.at<double>(1,1) );
          q(0,0) = (a.at<double>(1,0) - a.at<double>(0,1) ) / s;
          q(1,0) = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
          q(2,0) = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
          q(3,0) = 0.25f * s;
        }
    }

    if(q(0,0)<0)
    q=-1.0*q;

    return q;

}


arma::mat Obs_computer::DCM2QUAT_float(const cv::Mat& a){

    arma::mat q = arma::zeros<arma::mat>(4, 1);

    float trace = a.at<float>(0,0) + a.at<float>(1,1) + a.at<float>(2,2);
    if( trace > 0 ) {// I changed M_EPSILON to 0
      //std::cout<<"Condition 1\n";
      float s = 0.5f / sqrtf(trace+ 1.0f);
      q(0,0) = 0.25f / s;
      q(1,0) = ( a.at<float>(2,1) - a.at<float>(1,2) ) * s;
      q(2,0) = ( a.at<float>(0,2) - a.at<float>(2,0) ) * s;
      q(3,0) = ( a.at<float>(1,0) - a.at<float>(0,1) ) * s;
    } else {
      if ( a.at<float>(0,0) > a.at<float>(1,1) && a.at<float>(0,0) > a.at<float>(2,2) ) {
          //std::cout<<"Condition 2\n";
          float s = 2.0f * sqrtf( 1.0f + a.at<float>(0,0) - a.at<float>(1,1) - a.at<float>(2,2));
          q(0,0) = (a.at<float>(2,1) - a.at<float>(1,2) ) / s;
          q(1,0) = 0.25f * s;
          q(2,0) = (a.at<float>(0,1) + a.at<float>(1,0) ) / s;
          q(3,0) = (a.at<float>(0,2) + a.at<float>(2,0) ) / s;
        } else if (a.at<float>(1,1) > a.at<float>(2,2)) {
          //std::cout<<"Condition 3\n";
          float s = 2.0f * sqrtf( 1.0f + a.at<float>(1,1) - a.at<float>(0,0) - a.at<float>(2,2));
          q(0,0) = (a.at<float>(0,2) - a.at<float>(2,0) ) / s;
          q(1,0) = (a.at<float>(0,1) + a.at<float>(1,0) ) / s;
          q(2,0) = 0.25f * s;
          q(3,0) = (a.at<float>(1,2) + a.at<float>(2,1) ) / s;
        } else {
          //std::cout<<"Condition 4\n";
          float s = 2.0f * sqrtf( 1.0f + a.at<float>(2,2) - a.at<float>(0,0) - a.at<float>(1,1) );
          q(0,0) = (a.at<float>(1,0) - a.at<float>(0,1) ) / s;
          q(1,0) = (a.at<float>(0,2) + a.at<float>(2,0) ) / s;
          q(2,0) = (a.at<float>(1,2) + a.at<float>(2,1) ) / s;
          q(3,0) = 0.25f * s;
        }
    }

    if(q(0,0)<0)
    q=-1.0*q;

    return q;

}

// ==================================================================================
arma::mat Obs_computer::get_angular_velocity_with_H(const cv::Mat& Hprev, const cv::Mat& Hcur){

    cv::Mat Rprev = Hprev(cv::Range(0,3), cv::Range(0,3));
    cv::Mat Rcur  = Hcur(cv::Range(0,3), cv::Range(0,3));
    cv::Mat Omega_x = (Rcur - Rprev) * Rprev.t();
    Omega_x = (Omega_x - Omega_x.t());

    arma::mat omega;
    omega << 0.5 * Omega_x.at<float>(2,1) << arma::endr
          << 0.5 * Omega_x.at<float>(0,2) << arma::endr
          << 0.5 * Omega_x.at<float>(1,0) << arma::endr;

    return omega;

}

arma::mat Obs_computer::get_angular_velocity_with_R(const cv::Mat& Rprev, const cv::Mat& Rcur){

    cv::Mat Omega_x = (Rcur - Rprev) * Rprev.t();
    Omega_x = (Omega_x - Omega_x.t());

    arma::mat omega;
    omega << 0.5 * Omega_x.at<double>(2,1) << arma::endr
          << 0.5 * Omega_x.at<double>(0,2) << arma::endr
          << 0.5 * Omega_x.at<double>(1,0) << arma::endr;

    return omega;

}


// FIXME: may be let Xv be an input reference?
arma::mat Obs_computer::get_Camera_State_Predicted (const cv::Mat& Hprev, const cv::Mat& Hcur){

    arma::mat Xv = arma::zeros<arma::mat>(13, 1);

    // r
    Xv(0,0) = Hcur.at<float>(0, 3);
    Xv(1,0) = Hcur.at<float>(1, 3);
    Xv(2,0) = Hcur.at<float>(2, 3);

    // q
    Xv.rows(3, 6) = DCM2QUAT_float(Hcur.rowRange(0,3).colRange(0,3));

    // v
    Xv(7,0) = (Hcur.at<float>(0, 3) - Hprev.at<float>(0, 3));
    Xv(8,0) = (Hcur.at<float>(1, 3) - Hprev.at<float>(1, 3));
    Xv(9,0) = (Hcur.at<float>(2, 3) - Hprev.at<float>(2, 3));

    // omega
    Xv.rows(10, 12) = get_angular_velocity_with_H(Hprev, Hcur);

    return Xv;
}

// ==================================================================================
void Obs_computer::compute_H (const arma::mat & Xv, const arma::mat & yi, const arma::mat & zi,
                 arma::mat & dh_dxv, arma::mat & dh_dy){

    assert(yi.n_rows == 3);

    // --------------------------------------
    arma::mat rw = Xv.rows(0,2);
    arma::mat qwr = Xv.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(qwr));
    arma::mat RelRw = yi - rw;

    // --------------------------------------
    // Compute the dh_dxv
    // --------------------------------------
    // dhd_dhu
    double ud = zi(0,0);
    double vd = zi(0,1);
    double xd = (zi(0,0) - Cx) * dx;
    double yd = (zi(0,1) - Cy) * dy;
    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;
    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
    arma::mat dhd_dhu = J_undistor.i();

    // dhu_dhrl
    arma::mat hrl = Rrw * RelRw;
    arma::mat dhu_dhrl;
    if( fabs(hrl(2,0)) < 1e-6 ) {
        dhu_dhrl  = arma::zeros<arma::mat>(2,3);
    } else {
    dhu_dhrl << f*ku/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*f*ku/( std::pow(hrl(2,0), 2.0)) << arma::endr
                    << 0.0    <<  f*kv/(hrl(2,0))    <<  -hrl(1,0)*f*kv/( std::pow( hrl(2,0), 2.0)) << arma::endr;
    }

    // dh_dhrl
    arma::mat dh_dhrl = dhd_dhu *dhu_dhrl;

    // dh_drw
    //    arma::mat dhrl_drw = - Rrw;
    //    arma::mat dh_drw = dh_dhrl * dhrl_drw;
    arma::mat dh_drw = -1.0 * (dh_dhrl *  Rrw);

    // dhrl_dqwr
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq( qwr_conj ,  RelRw) * dqbar_by_dq;

     // dh_dqwr
    arma::mat dh_dqwr = dh_dhrl * dhrl_dqwr;

    // dh_dxv
    arma::mat Tmp;
    Tmp = arma::join_horiz(dh_drw, dh_dqwr);
    dh_dxv = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6));

    //----------------------------------------
    // compute dh_dy
    //----------------------------------------
    dh_dy = dh_dhrl * Rrw;
}

// ==================================================================================
void Obs_computer::compute_H (const arma::mat & Xv, const arma::mat & yi, const arma::mat & zi,
                 arma::mat & dh_dxv){

    assert(yi.n_rows == 3);

    // --------------------------------------
    arma::mat rw = Xv.rows(0,2);
    arma::mat qwr = Xv.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(qwr));
    arma::mat RelRw = yi - rw;

    // --------------------------------------
    // Compute the dh_dxv
    // --------------------------------------
    // dhd_dhu
    double ud = zi(0,0);
    double vd = zi(0,1);
    double xd = (zi(0,0) - Cx) * dx;
    double yd = (zi(0,1) - Cy) * dy;
    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;
    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
    arma::mat dhd_dhu = J_undistor.i();

    // dhu_dhrl
    arma::mat hrl = Rrw * RelRw;
    arma::mat dhu_dhrl;
    if( fabs(hrl(2,0)) < 1e-6 ) {
        dhu_dhrl  = arma::zeros<arma::mat>(2,3);
    } else {
    dhu_dhrl << f*ku/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*f*ku/( std::pow(hrl(2,0), 2.0)) << arma::endr
                    << 0.0    <<  f*kv/(hrl(2,0))    <<  -hrl(1,0)*f*kv/( std::pow( hrl(2,0), 2.0)) << arma::endr;
    }

    // dh_dhrl
    arma::mat dh_dhrl = dhd_dhu *dhu_dhrl;

    // dh_drw
    //    arma::mat dhrl_drw = - Rrw;
    //    arma::mat dh_drw = dh_dhrl * dhrl_drw;
    arma::mat dh_drw = -1.0 * (dh_dhrl *  Rrw);

    // dhrl_dqwr
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq( qwr_conj ,  RelRw) * dqbar_by_dq;

     // dh_dqwr
    arma::mat dh_dqwr = dh_dhrl * dhrl_dqwr;

    // dh_dxv
    arma::mat Tmp;
    Tmp = arma::join_horiz(dh_drw, dh_dqwr);
    dh_dxv = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6));

}

// ==================================================================================
void Obs_computer::compute_H_invseDepth(const arma::mat & Xv, const arma::mat & yi, const arma::mat & zi,
               arma::mat & dh_dxv, arma::mat & dh_dy) {

     assert(yi.n_rows == 6);

    // --------------------------------------
    arma::mat rw = Xv.rows(0,2);
    arma::mat qwr = Xv.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(qwr));

    arma::mat pts = yi.rows(0,2);
    double lambda = yi(5,0),  phi = yi(4,0), theta = yi(3,0);

    arma::colvec mi(3);
    mi << std::cos(phi)*std::sin(theta) << -std::sin(phi) << std::cos(phi)*std::cos(theta) << arma::endr;

    arma::mat right = ((pts - rw)*lambda + mi);

    // --------------------------------------
    // Compute the dh_dxv
    // --------------------------------------

    // dhd_dhu
    double ud = zi(0,0);
    double vd = zi(0,1);
    double xd = (zi(0,0) - Cx) * dx;
    double yd = (zi(0,1) - Cy) * dy;
    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;
    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
    arma::mat dhd_dhu = J_undistor.i();

    // dhu_dhrl
    arma::mat hc = Rrw*right;
    arma::mat dhu_dhrl;
    if( fabs(hc(2,0)) < 1e-6 ) {
        dhu_dhrl  = arma::zeros<arma::mat>(2,3);
    }  else {
        dhu_dhrl << f*ku/(hc(2,0))  << 0.0  <<  -hc(0,0)*f*ku/(std::pow(hc(2,0), 2.0)) << arma::endr
          << 0.0  << f*kv/(hc(2,0))  << -hc(1,0)*f*kv/(std::pow(hc(2,0), 2.0)) << arma::endr;
    }

    // dhrl_drw
    arma::mat dhrl_drw = -1.0 * Rrw * yi(5,0);
  
    // dh_dhrl
    arma::mat dh_dhrl = dhd_dhu * dhu_dhrl;
    
    // dh_drw 
    arma::mat dh_drw = dh_dhrl * dhrl_drw;

    // dhrl_dqwr
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq(qwr_conj, right) * dqbar_by_dq;

    // dh_dqwr
    arma::mat dh_dqwr = dh_dhrl * dhrl_dqwr;

    // dh_dxv
    arma::mat Tmp;
    Tmp = arma::join_horiz(dh_drw, dh_dqwr);
    dh_dxv = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6)); 

    //----------------------------------------
    // compute dh_dy
    //----------------------------------------

    arma::mat tmp1(3,1), tmp2(3,1);
    tmp1 << std::cos(phi)*std::cos(theta) << arma::endr
         << 0.0 << arma::endr
         << -std::cos(phi)*std::sin(theta) << arma::endr;

    arma::mat dmi_dthetai = Rrw * tmp1;

    tmp2 << -std::sin(phi)*std::sin(theta) << arma::endr
        << -cos(phi) << arma::endr
        << -std::sin(phi)*std::cos(theta) << arma::endr;
    arma::mat dmi_dphii = Rrw*tmp2;

    arma::mat dhrl_dy = arma::join_horiz(lambda*Rrw, dmi_dthetai);
    dhrl_dy = arma::join_horiz(dhrl_dy, dmi_dphii);
    dhrl_dy = arma::join_horiz(dhrl_dy, Rrw * (pts - rw));

    dh_dy = dh_dhrl * dhrl_dy;

    return;
}


// ==================================================================================
void Obs_computer::compute_H_invseDepth(const arma::mat & Xv, const arma::mat & yi, const arma::mat & zi,
               arma::mat & dh_dxv) {

     assert(yi.n_rows == 6);

    // --------------------------------------
    arma::mat rw = Xv.rows(0,2);
    arma::mat qwr = Xv.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(qwr));

    arma::mat pts = yi.rows(0,2);
    double lambda = yi(5,0),  phi = yi(4,0), theta = yi(3,0);

    arma::colvec mi(3);
    mi << std::cos(phi)*std::sin(theta) << -std::sin(phi) << std::cos(phi)*std::cos(theta) << arma::endr;

    arma::mat right = ((pts - rw)*lambda + mi);

    // --------------------------------------
    // Compute the dh_dxv
    // --------------------------------------

    // dhd_dhu
    double ud = zi(0,0);
    double vd = zi(0,1);
    double xd = (zi(0,0) - Cx) * dx;
    double yd = (zi(0,1) - Cy) * dy;
    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;
    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
    arma::mat dhd_dhu = J_undistor.i();

    // dhu_dhrl
    arma::mat hc = Rrw*right;
    arma::mat dhu_dhrl;
    if( fabs(hc(2,0)) < 1e-6 ) {
        dhu_dhrl  = arma::zeros<arma::mat>(2,3);
    }  else {
        dhu_dhrl << f*ku/(hc(2,0))  << 0.0  <<  -hc(0,0)*f*ku/(std::pow(hc(2,0), 2.0)) << arma::endr
          << 0.0  << f*kv/(hc(2,0))  << -hc(1,0)*f*kv/(std::pow(hc(2,0), 2.0)) << arma::endr;
    }

    // dhrl_drw
    arma::mat dhrl_drw = -1.0 * Rrw * yi(5,0);

    // dh_dhrl
    arma::mat dh_dhrl = dhd_dhu * dhu_dhrl;

    // dh_drw
    arma::mat dh_drw = dh_dhrl * dhrl_drw;

    // dhrl_dqwr
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq(qwr_conj, right) * dqbar_by_dq;

    // dh_dqwr
    arma::mat dh_dqwr = dh_dhrl * dhrl_dqwr;

    // dh_dxv
    arma::mat Tmp;
    Tmp = arma::join_horiz(dh_drw, dh_dqwr);
    dh_dxv = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6));

    return;
}

// ==================================================================================
void Obs_computer::inst_ObsScore_indi_feat(const arma::mat & Xv,
                                           const std::vector<std::pair<arma::mat, arma::mat> > &YZ,
                                           const double & dt,
                                           std::vector<arma::mat> & indObsMat,
                                           std::vector<double> & indObsScore) {
    // Compute the F series
    ////std::cout << "Computing F series" << std::endl;

    arma::mat F;
    compute_F(Xv, dt, F);
    std::vector<arma::mat> F_mul;
    F_mul.push_back(F);
    for (int i = 1; i < 12; i++) {
        F_mul.push_back(F_mul.back() * F);
    }

    // Compute the individual linear observability matrix
    ////std::cout << "Compute the individual linear observability matrix" << std::endl;

    arma::mat H_x;
    arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);
    for (size_t i = 0; i < YZ.size(); i++) {
        compute_H(Xv, YZ[i].first, YZ[i].second, H_x);
        LinObsMat.rows(0,1) = H_x;
        for (int j = 1; j < 13; j++) {
               LinObsMat.rows( j*2, ((j+1)*2-1)) = H_x * F_mul[ j-1 ];
        }

        indObsMat.push_back(LinObsMat);
        arma::vec s = arma::svd(LinObsMat);
        indObsScore.push_back(s(12));
    }
}

// ==================================================================================
void Obs_computer::inst_ObsScore_indi_feat_Efficient(const arma::mat & Xv,
                                                     const std::vector<std::pair<arma::mat, arma::mat> > &YZ,
                                                     const double & dt,
                                                     std::vector<arma::mat> & indObsMat,
                                                     std::vector<double> & indObsScore) {

    //---------------------------------------------------------------------------------
    // Compute F

    // Update the quaternion
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;
    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

   //-------------------------------------------------------------------------------
   // Compute the individual linear observability matrix

    arma::mat H13;
    arma::mat H47;
    for (size_t i = 0; i < YZ.size(); i++) {
        compute_H_subblock(Xv, YZ[i].first, YZ[i].second, H13, H47);
        arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);

        // Copy first 3 columns
        LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
        // Columns 7~9
        LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
        if(fabs(dt - 1.0) > 1e-6) {
            LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
        }
        // First segment in linObsMat: just H:
        LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

        // Segment 2 to 13
        arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
        arma::mat rollingQFac = F_Q;  // Q^n
        for (int j = 1; j < 13; j++) {
                // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                // 2nd block:  (H47 * Q^n)
                LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                // 3rd block:  (H13 * n* dt)
                //LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(7,9))
                //   = static_cast<double>(j) * dt * H13;

                // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                // Update
                rollingQAdd = rollingQAdd + rollingQFac;
                rollingQFac = rollingQFac * F_Q;
        }

        indObsMat.push_back(LinObsMat);
        arma::vec s = arma::svd(LinObsMat);
        indObsScore.push_back(s(12));
    }
}

// ==================================================================================
void Obs_computer::inst_ObsScore_triplet(const arma::mat & Xv,
                                         const std::vector<Triplet > &tripletsYZ,
                                         const double & dt,
                                         std::vector<arma::mat> & triObsMat,
                                         std::vector<double> & triObsScore,
                                         std::vector<int> & triObsRank){


    //-----------------------------------------------------------------------------
    // Compute F

    // Update the quaternion
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;
    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    //---------------------------------------------------------------------------------
    // Compute the individual linear observability matrix
    // Note that the triplet ObsMat:
    //   ObsMat([H1; H2; H3], F) = [ObsMat(H1, F), ObsMat(H2, F), ObsMat(H3, F)]

    arma::mat H13;
    arma::mat H47;

    std::vector<std::pair<arma::mat, arma::mat> > triplet;
    std::pair<arma::mat, arma::mat> YZ;
    for (size_t tripletIdx = 0; tripletIdx < tripletsYZ.size(); tripletIdx++) {

        triplet = tripletsYZ[tripletIdx];
        assert(triplet.size() == 3);

        // Compute the ObsMat and ObsScore for each triplet
        arma::mat TripletObsMat = arma::zeros<arma::mat>(26*3, 13);
        for (size_t featIdx = 0; featIdx < triplet.size(); featIdx++) {

            YZ = triplet[featIdx];

            compute_H_subblock(Xv, YZ.first, YZ.second, H13, H47);
            arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);

            // Copy first 3 columns
            LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
            // Columns 7~9
            LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
            if(fabs(dt - 1.0) > 1e-6) {
                LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
            }
            // First segment in linObsMat: just H:
            LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

            // Segment 2 to 13
            arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
            arma::mat rollingQFac = F_Q;  // Q^n
            for (int j = 1; j < 13; j++) {
                    // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                    // 2nd block:  (H47 * Q^n)
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                    // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                    // Update
                    rollingQAdd = rollingQAdd + rollingQFac;
                    rollingQFac = rollingQFac * F_Q;
            }

            TripletObsMat.rows(featIdx*26, (featIdx*26+25) ) = LinObsMat;
        }

        triObsMat.push_back(TripletObsMat);
        triObsRank.push_back(arma::rank(TripletObsMat));
        arma::vec s = arma::svd(TripletObsMat);
        triObsScore.push_back(s(12));
    }
}


// ==================================================================================
void Obs_computer::inst_ObsScore_triplet(const arma::mat & Xv,
                                         const std::vector<Triplet > &tripletsYZ,
                                         const double & dt,
                                         std::vector<std::pair<size_t, double> >  & triObsScore){


    triObsScore.reserve(tripletsYZ.size());

    // Compute F

    // Update the quaternion
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;
    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    //---------------------------------------------------------------------------------
    // Compute the individual linear observability matrix
    // Note that the triplet ObsMat:
    //   ObsMat([H1; H2; H3], F) = [ObsMat(H1, F), ObsMat(H2, F), ObsMat(H3, F)]

    arma::mat H13;
    arma::mat H47;

    std::vector<std::pair<arma::mat, arma::mat> > triplet;
    std::pair<arma::mat, arma::mat> YZ;
    for (size_t tripletIdx = 0; tripletIdx < tripletsYZ.size(); tripletIdx++) {

        triplet = tripletsYZ[tripletIdx];
        assert(triplet.size() == 3);

        // Compute the ObsMat and ObsScore for each triplet
        arma::mat TripletObsMat = arma::zeros<arma::mat>(26*3, 13);
        for (size_t featIdx = 0; featIdx < triplet.size(); featIdx++) {

            YZ = triplet[featIdx];

            compute_H_subblock(Xv, YZ.first, YZ.second, H13, H47);
            arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);

            // Copy first 3 columns
            LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
            // Columns 7~9
            LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
            if(fabs(dt - 1.0) > 1e-6) {
                LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
            }
            // First segment in linObsMat: just H:
            LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

            // Segment 2 to 13
            arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
            arma::mat rollingQFac = F_Q;  // Q^n
            for (int j = 1; j < 13; j++) {
                    // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                    // 2nd block:  (H47 * Q^n)
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                    // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                    // Update
                    rollingQAdd = rollingQAdd + rollingQFac;
                    rollingQFac = rollingQFac * F_Q;
            }

            TripletObsMat.rows(featIdx*26, (featIdx*26+25) ) = LinObsMat;
        }

        arma::vec s = arma::svd(TripletObsMat);
        triObsScore.push_back(std::make_pair( tripletIdx, s(12)));
    }
}



// ==================================================================================
void Obs_computer::temporal_ObsScore_indi_feat(const DQ_State & Xv_Q,
                                               const Vec_DQ_FeatYZ & V_Q_featYZ,
                                               const std::deque<double> & dt_Q,
                                               std::vector<arma::mat> & tmprObsMat,
                                               std::vector<double> & tmprObsScore,
                                               std::vector<int> & tmprObsRank) {

    assert(!V_Q_featYZ.empty());
    assert(Xv_Q.size() == V_Q_featYZ[0].size() && Xv_Q.size() == dt_Q.size());
    int timeSegNum = Xv_Q.size();

    // Compute the SOM and tmprObsScore for each feature across time segments
    // The key is, the F matrices are the same in the same time segment for different features
    // Thus, we collect the F matrices first.

    std::vector<std::pair<arma::mat, arma::mat> > F_vec(timeSegNum);
    for(int timeIdx = 0; timeIdx < timeSegNum; timeIdx++) {

        arma::mat Xv = Xv_Q[timeIdx];
        double dt = dt_Q[timeIdx];

        //----------------------------
        // Compute F

        // Update the quaternion
        arma::mat omegaOld = Xv.rows(10, 12);
        arma::mat qOld = Xv.rows(3, 6);
        arma::mat v = omegaOld * dt;
        double theta = arma::norm(v, 2);
        arma::mat q;
        if(theta < 1e-6) {
            q << 1 << 0 << 0 << 0 << arma::endr;
        } else {
            arma::mat v_n = v / theta;
            arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
            q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
        }

        // F matrix subblock:  F_Q
        double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
        arma::mat F_Q;
        F_Q  << R << -X << -Y << -Z << arma::endr
               << X << R  <<  Z << -Y << arma::endr
               << Y << -Z <<  R <<  X << arma::endr
               << Z <<  Y << -X <<  R << arma::endr;

        // F matrix subblock:   F_Omg
        R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
        arma::mat  dq3_by_dq1;
        dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                    << X <<  R << -Z <<  Y << arma::endr
                    << Y <<  Z <<  R << -X << arma::endr
                    << Z << -Y <<  X <<  R << arma::endr;
        arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

        F_vec[timeIdx] = std::make_pair(F_Q, F_Omg);
    }

    assert(static_cast<int>(F_vec.size()) == timeSegNum);

    // Compute the SOM for each feature by going through the temporal queue of each feature.
    // The computation is very similar to triplet Obs computation, but F matrix changes at each time segment
    std::pair<arma::mat, arma::mat> YZ;
    arma::mat H13;
    arma::mat H47;

    for (size_t featIdx = 0; featIdx < V_Q_featYZ.size(); featIdx++) {

        assert(static_cast<int>(V_Q_featYZ[featIdx].size()) == timeSegNum);

        arma::mat SOM = arma::zeros<arma::mat>(26*timeSegNum, 13);
        for (int timeIdx = 0; timeIdx < timeSegNum; timeIdx++) {

            // For this feature, grap measurements, featEstimate, F matrices in each time segment.
            YZ = V_Q_featYZ[featIdx][timeIdx];
            arma::mat F_Q = F_vec[timeIdx].first;
            arma::mat F_Omg = F_vec[timeIdx].second;
            double dt = dt_Q[timeIdx];

            compute_H_subblock(Xv_Q[timeIdx], YZ.first, YZ.second, H13, H47);
            arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);

            // Copy first 3 columns
            LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
            // Columns 7~9
            LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
            if(fabs(dt - 1.0) > 1e-6) {
                LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
            }
            // First segment in linObsMat: just H:
            LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

            // Segment 2 to 13
            arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
            arma::mat rollingQFac = F_Q;  // Q^n
            for (int j = 1; j < 13; j++) {
                    // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                    // 2nd block:  (H47 * Q^n)
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                    // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                    LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                    // Update
                    rollingQAdd = rollingQAdd + rollingQFac;
                    rollingQFac = rollingQFac * F_Q;
            }

            SOM.rows(timeIdx*26, (timeIdx*26+25) ) = LinObsMat;
        }

        tmprObsMat.push_back(SOM);
        tmprObsRank.push_back(arma::rank(SOM));
        arma::vec s = arma::svd(SOM);
        tmprObsScore.push_back(s(12));
    }

    return;
}


// ==================================================================================
void Obs_computer::update_temporal_ObsScore_indi_feat(const arma::mat & Xv,
                                        const std::vector<std::pair<arma::mat, arma::mat> > &YZ,
                                        const double & dt,
                                        const int & tmprLen,
                                        std::vector<arma::mat> & tmprObsMat2Update,
                                        std::vector<double> & tmprObsScore2Update,
                                        std::vector<int> & tmprObsRank2Update) {

    // This function is to update the SOM of a set of features given the previous SOM, the
    // desired temporal length, and the current state + measurement

    assert(YZ.size() == tmprObsMat2Update.size());
    assert(YZ.size() == tmprObsScore2Update.size());
    assert(YZ.size() == tmprObsRank2Update.size());
    assert(tmprLen > 1);

    //-----------------------------------------------------------------------------
    // Compute F

    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;
    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    //-----------------------------------------------------------------------------
    arma::mat H13;
    arma::mat H47;
    for(size_t featIdx = 0; featIdx < YZ.size(); featIdx++) {

        // Compute the H matrix
        compute_H_subblock(Xv, YZ[featIdx].first, YZ[featIdx].second, H13, H47);
        arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);

        // Copy first 3 columns
        LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
        // Columns 7~9
        LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
        if(fabs(dt - 1.0) > 1e-6) {
            LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
        }
        // First segment in linObsMat: just H:
        LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

        // Segment 2 to 13
        arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
        arma::mat rollingQFac = F_Q;  // Q^n
        for (int j = 1; j < 13; j++) {
                // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                // 2nd block:  (H47 * Q^n)
                LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12))
                        = H47 * (rollingQAdd * F_Omg);

                // Update
                rollingQAdd = rollingQAdd + rollingQFac;
                rollingQFac = rollingQFac * F_Q;
        }


        // Update the SOM
        // TODO: how to initalize for the first time instance of feature measurement?
        arma::mat SOM_old = tmprObsMat2Update[featIdx];
        int Len_SOM_old = static_cast<int>(SOM_old.n_rows);

        assert(Len_SOM_old% 26 == 0);
        if(Len_SOM_old == 0) {
            /// Case 1: first time observed, the SOM_old is an empty matrix
            tmprObsMat2Update[featIdx] = LinObsMat;
            tmprObsScore2Update[featIdx] = 0.0;
            tmprObsRank2Update[featIdx] = arma::rank(LinObsMat);

        }
        else if(Len_SOM_old == 26*tmprLen) {
            /// Case 2: phase 4, "fully loaded"
            // SOM block order: row 0 -- oldest, row End -- latest.

            arma::mat SOM = arma::zeros<arma::mat>(SOM_old.n_rows, SOM_old.n_cols);
            SOM.rows(0, 26*(tmprLen-1)-1) = SOM_old.rows(26, 26*tmprLen-1);
            SOM.rows(26*(tmprLen-1), 26*tmprLen-1) = LinObsMat;

            // Save
            tmprObsMat2Update[featIdx] = SOM;
            arma::vec s = arma::svd(SOM);
            tmprObsScore2Update[featIdx] = s(12);
            tmprObsRank2Update[featIdx] = arma::rank(SOM);

        }
        else if(Len_SOM_old< 26*tmprLen) {
            /// Case 3: phase 2/3, "NOT fully loaded, but has been initialized"
            arma::mat SOM = arma::join_vert(SOM_old, LinObsMat);
            tmprObsMat2Update[featIdx] = SOM;
            tmprObsRank2Update[featIdx] = arma::rank(SOM);

            if (SOM.n_rows >= 32) {
                // Full rank
                arma::vec s = arma::svd(SOM);
                tmprObsScore2Update[featIdx] = s(12);
            } else {
                tmprObsScore2Update[featIdx] = 0.0;
            }
        }

    } // end for(featIdx)

    return;
}

// ==================================================================================
void Obs_computer::update_temporal_ObsScore_indi_feat(
                                        Frame *pFrame,    const arma::mat & Xv,
                                        const int & tmprLen, const double & dt  ) {

    const int N = pFrame->mvpMapPoints.size();

    //-----------------------------------------------------------------------------
    // Compute F
    //std::cout << "[OBS_COMPUTOR]  Get F" << std::endl;
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;

    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);


    //-----------------------------------------------------------------------------
    // Compute H and update Obs information for each feature

    //std::cout << "[OBS_COMPUTOR]  Get H" << std::endl;

    arma::mat H13;
    arma::mat H47;
    for(int i=0; i<N; i++)  {
            MapPoint* pMP = pFrame->mvpMapPoints[i];

            // If the current Map point is not matched:
            // Reset the Obs information
            if(pMP) {
                //std::cout << "[OBS_COMPUTOR]  Valid  pMP" << std::endl;
                // Feature position
                arma::mat Y = arma::zeros<arma::mat>(3,1);
                cv::Mat featPos = pMP->GetWorldPos();
                Y(0,0) = featPos.at<float>(0);
                Y(1,0) = featPos.at<float>(1);
                Y(2,0) = featPos.at<float>(2);

                // Measurement
                arma::mat Z = arma::zeros<arma::mat>(1,2);
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                Z(0,0) = kpUn.pt.x;
                Z(0,1) = kpUn.pt.y;

                // Compute the H matrix
                //std::cout << "[OBS_COMPUTOR]  compute_H_subblock" << std::endl;
                compute_H_subblock(Xv, Y, Z, H13, H47);

                //std::cout << "[OBS_COMPUTOR]  LinObsMat" << std::endl;
                arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);
                // Copy first 3 columns
                LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
                // Columns 7~9
                LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
                if(fabs(dt - 1.0) > 1e-6) {
                    LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
                }
                // First segment in linObsMat: just H:
                LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

                // Segment 2 to 13
                arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
                arma::mat rollingQFac = F_Q;  // Q^n
                for (int j = 1; j < 13; j++) {
                        // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                        // 2nd block:  (H47 * Q^n)
                        LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                        // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                        LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                        // Update
                        rollingQAdd = rollingQAdd + rollingQFac;
                        rollingQFac = rollingQFac * F_Q;
                }

                //std::cout << "[OBS_COMPUTOR]  Update SOM" << std::endl;
                // Update the SOM
                arma::mat SOM_old = pMP->ObsMat;
                int Len_SOM_old = static_cast<int>(SOM_old.n_rows);

                assert(Len_SOM_old% 26 == 0);
                if(Len_SOM_old == 0) {
                    //std::cout << "[OBS_COMPUTOR]  Case 1" << std::endl;
                    /// Case 1: first time observed, the SOM_old is an empty matrix
                    pMP->ObsMat  = LinObsMat;
                    pMP->ObsScore= 0.0;
//                    pMP->ObsRank = arma::rank(LinObsMat);
                }
                else if(Len_SOM_old == 26*tmprLen) {
                    //std::cout << "[OBS_COMPUTOR]  Case 2" << std::endl;
                    /// Case 2: phase 4, "fully loaded"
                    // SOM block order: row 0 -- oldest, row End -- latest.

                    arma::mat SOM = arma::zeros<arma::mat>(SOM_old.n_rows, SOM_old.n_cols);
                    SOM.rows(0, 26*(tmprLen-1)-1) = SOM_old.rows(26, 26*tmprLen-1);
                    SOM.rows(26*(tmprLen-1), 26*tmprLen-1) = LinObsMat;

                    // Save
                    arma::vec s = arma::svd(SOM);
                    pMP->ObsMat  = SOM;
                    pMP->ObsScore= s(12);
//                    pMP->ObsRank = arma::rank(SOM);

                }
                else if(Len_SOM_old< 26*tmprLen) {
                    //std::cout << "[OBS_COMPUTOR]  Case 3" << std::endl;
                    /// Case 3: phase 2/3, "NOT fully loaded, but has been initialized"
                    arma::mat SOM = arma::join_vert(SOM_old, LinObsMat);
                    pMP->ObsMat  = SOM;
//                    pMP->ObsRank = arma::rank(SOM);


                    if (SOM.n_rows >= 32) {
                        // Full rank
                        arma::vec s = arma::svd(SOM);
                         pMP->ObsScore= s(12);
                    } else {
                         pMP->ObsScore = 0.0;
                    }
                }
            } // else: Perform regular temporal obs update!
         } // For: all pMP
    //std::cout << "[OBS_COMPUTOR]  Done" << std::endl;

    return;
}

// ==================================================================================
MP_sorted_tmprObs Obs_computer::update_sort_temporal_ObsScore_indi_feat(
                                                    Frame *pFrame,    const arma::mat & Xv,
                                                    const int & tmprLen, const double & dt  ) {

#ifdef TIME_TMPRUPDATE_SORE
    arma::wall_clock timer;
    timer.tic();
#endif

    MP_sorted_tmprObs  mpSorted;
    double meanObsScore = 0.0;  // we simply put this to the end of the mpSorted; !!

    const int N = pFrame->mvpMapPoints.size();

    //-----------------------------------------------------------------------------
    // Compute F
    //std::cout << "[OBS_COMPUTOR]  Get F" << std::endl;
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;

    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

#ifdef TIME_TMPRUPDATE_SORE
    std::cout << "[OBS_COMPUTOR] Time up to F computation: " << timer.toc() << std::endl;
#endif


    //-----------------------------------------------------------------------------
    // Compute H and update Obs information for each feature
    //std::cout << "[OBS_COMPUTOR]  Get H" << std::endl;

    arma::mat H13;
    arma::mat H47;
    for(int i=0; i<N; i++)  {
            MapPoint* pMP = pFrame->mvpMapPoints[i];

            // If the current Map point is not matched:
            // Reset the Obs information
            if(pMP) {
                //std::cout << "[OBS_COMPUTOR]  Valid  pMP" << std::endl;
                // Feature position
                arma::mat Y = arma::zeros<arma::mat>(3,1);
                cv::Mat featPos = pMP->GetWorldPos();
                Y(0,0) = featPos.at<float>(0);
                Y(1,0) = featPos.at<float>(1);
                Y(2,0) = featPos.at<float>(2);

                // Measurement
                arma::mat Z = arma::zeros<arma::mat>(1,2);
                cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
                Z(0,0) = kpUn.pt.x;
                Z(0,1) = kpUn.pt.y;

                // Compute the H matrix
                //std::cout << "[OBS_COMPUTOR]  compute_H_subblock" << std::endl;
                compute_H_subblock(Xv, Y, Z, H13, H47);

                //std::cout << "[OBS_COMPUTOR]  LinObsMat" << std::endl;
                arma::mat LinObsMat = arma::zeros<arma::mat>(26, 13);
                // Copy first 3 columns
                LinObsMat.cols(0, 2) = arma::repmat(H13, 13, 1);
                // Columns 7~9
                LinObsMat.cols(7,9) = LinObsMat.cols(0, 2) % H13_mul_fac;
                if(fabs(dt - 1.0) > 1e-6) {
                    LinObsMat.cols(7,9) = LinObsMat.cols(7,9) * dt;
                }
                // First segment in linObsMat: just H:
                LinObsMat(arma::span(0,1), arma::span(3,6))= H47;

                // Segment 2 to 13
                arma::mat rollingQAdd= arma::eye<arma::mat>(4, 4);  // (Q^(n-1) + Q^(n-2) +... + I)
                arma::mat rollingQFac = F_Q;  // Q^n
                for (int j = 1; j < 13; j++) {
                        // [0,1,2,   3,4,5,6,   7,8,9,   10,11,12 ]

                        // 2nd block:  (H47 * Q^n)
                        LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(3,6)) = H47 * rollingQFac;

                        // 4th block: Q^(n-1) + Q^(n-2) +... + I) * Omega
                        LinObsMat( arma::span(j*2, ((j+1)*2-1)), arma::span(10,12)) = H47 * (rollingQAdd * F_Omg);

                        // Update
                        rollingQAdd = rollingQAdd + rollingQFac;
                        rollingQFac = rollingQFac * F_Q;
                }

                //std::cout << "[OBS_COMPUTOR]  Update SOM" << std::endl;
                // Update the SOM
                arma::mat SOM_old = pMP->ObsMat;
                int Len_SOM_old = static_cast<int>(SOM_old.n_rows);

                assert(Len_SOM_old% 26 == 0);
                if(Len_SOM_old == 0) {
                    //std::cout << "[OBS_COMPUTOR]  Case 1" << std::endl;
                    /// Case 1: first time observed, the SOM_old is an empty matrix
                    pMP->ObsMat  = LinObsMat;
                    pMP->ObsScore= 0.0;
                    pMP->ObsRank = arma::rank(LinObsMat);
                }
                else if(Len_SOM_old == 26*tmprLen) {
                    //std::cout << "[OBS_COMPUTOR]  Case 2" << std::endl;
                    /// Case 2: phase 4, "fully loaded"
                    // SOM block order: row 0 -- oldest, row End -- latest.

                    arma::mat SOM = arma::zeros<arma::mat>(SOM_old.n_rows, SOM_old.n_cols);
                    SOM.rows(0, 26*(tmprLen-1)-1) = SOM_old.rows(26, 26*tmprLen-1);
                    SOM.rows(26*(tmprLen-1), 26*tmprLen-1) = LinObsMat;

                    // Save
                    arma::vec s = arma::svd(SOM);
                    pMP->ObsMat  = SOM;
                    pMP->ObsScore= s(12);
                    pMP->ObsRank = arma::rank(SOM);

                    // add to the sorting list
                   mpSorted.push_back(std::make_pair(static_cast<size_t>(i), pMP->ObsScore ));
                   meanObsScore += pMP->ObsScore;

                }
                else if(Len_SOM_old< 26*tmprLen) {
                    //std::cout << "[OBS_COMPUTOR]  Case 3" << std::endl;
                    /// Case 3: phase 2/3, "NOT fully loaded, but has been initialized"
                    arma::mat SOM = arma::join_vert(SOM_old, LinObsMat);
                    pMP->ObsMat  = SOM;
                    pMP->ObsRank = arma::rank(SOM);


                    if (SOM.n_rows >= 32) {
                        // Full rank
                        arma::vec s = arma::svd(SOM);
                         pMP->ObsScore= s(12);

                         // add to the sorting list
                        mpSorted.push_back(std::make_pair(static_cast<size_t>(i), pMP->ObsScore ));
                        meanObsScore += pMP->ObsScore;

                    } else {
                         pMP->ObsScore = 0.0;

                    }
                }
            } //Perform regular temporal obs update!
         } // For: all pMP

#ifdef TIME_TMPRUPDATE_SORE
    std::cout << "[OBS_COMPUTOR] Time up to all SOM computation: " << timer.toc() << std::endl;
#endif

     // Sort features according to the tmprObs score
     std::sort(mpSorted.begin(), mpSorted.end(), tmprObsScore_comparator);

#ifdef TIME_TMPRUPDATE_SORE
    std::cout << "[OBS_COMPUTOR] Time up to sorting: " << timer.toc() << std::endl;
#endif


    meanObsScore = meanObsScore / static_cast<double>(mpSorted.size());
    mpSorted.push_back(std::make_pair(static_cast<size_t>(10000), meanObsScore));

    //std::cout << "[OBS_COMPUTOR]  Done" << std::endl;
    return mpSorted;
}



// ==================================================================================
MP_sorted_tmprObs Obs_computer::update_sort_temporal_ObsScore_indi_feat_multiThreading(
                                                    Frame *pFrame,    const arma::mat & Xv,
                                                    const int & tmprLen, const double & dt  ) {

    MP_sorted_tmprObs  mpSorted;
    double meanObsScore = 0.0;  // we simply put this to the end of the mpSorted; !!

    const int N = pFrame->mvpMapPoints.size();

    //-----------------------------------------------------------------------------
    // Compute F
    //std::cout << "[OBS_COMPUTOR]  Get F" << std::endl;
    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);
    arma::mat v = omegaOld * dt;
    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 1e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  std::sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << std::cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }

    // F matrix subblock:  F_Q
    double R = q(0,0), X = q(0,1), Y = q(0,2), Z = q(0,3);
    arma::mat F_Q;
    F_Q  << R << -X << -Y << -Z << arma::endr
           << X << R  <<  Z << -Y << arma::endr
           << Y << -Z <<  R <<  X << arma::endr
           << Z <<  Y << -X <<  R << arma::endr;

    // F matrix subblock:   F_Omg
    R = qOld(0,0), X = qOld(1,0), Y = qOld(2,0), Z = qOld(3,0);
    arma::mat  dq3_by_dq1;
    dq3_by_dq1 << R << -X << -Y << -Z << arma::endr
                << X <<  R << -Z <<  Y << arma::endr
                << Y <<  Z <<  R << -X << arma::endr
                << Z << -Y <<  X <<  R << arma::endr;

    arma::mat F_Omg= dq3_by_dq1 * dqomegadt_by_domega(omegaOld,dt);

    arma::wall_clock timer;
    timer.tic();
//    //-----------------------------------------------------------------------------
//    // Compute H and update Obs information for each feature


     int N_Threads = 20;
     int grainSize = static_cast<double>(N)/static_cast<double>(N_Threads);

    std::vector<std::thread> threadVector;
    threadVector.reserve(N_Threads);
    for (int i = 0; i < N_Threads-1; i++) {
        threadVector.push_back(std::thread(&Obs_computer::testing_function, this, pFrame, i*grainSize, (i+1)*grainSize, dt, tmprLen, Xv, F_Q, F_Omg));
    }
    threadVector.push_back(std::thread(&Obs_computer::testing_function, this, pFrame, (N_Threads-1)*grainSize, N, dt, tmprLen, Xv, F_Q, F_Omg));

//    std::cerr << "% ~~~ inside timer 1: " << timer.toc() << std::endl;

    for (size_t i = 0; i < threadVector.size(); i++) {
        threadVector[i].join();
    }

    //-----------------------------------------------------------------------------

    // Collect the mpSorted
    for(int i= 0; i<N; i++)  {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        // If the current Map point is not matched:
        // Reset the Obs information
        if(pMP) {
            if(pMP->ObsScore > EPS) {
                meanObsScore += pMP->ObsScore;
                mpSorted.push_back(std::make_pair(static_cast<size_t>(i), pMP->ObsScore) );
            }
        }
    }


//        // Collect the mpSorted (Test38)
//        for(int i= 0; i<N; i++)  {
//            MapPoint* pMP = pFrame->mvpMapPoints[i];
//            // If the current Map point is not matched:
//            // Reset the Obs information
//            if(pMP) {
//                    meanObsScore += pMP->ObsScore;
//                    mpSorted.push_back(std::make_pair(static_cast<size_t>(i), pMP->ObsScore) );
//            }
//        }

    // Sort features according to the tmprObs score
    std::sort(mpSorted.begin(), mpSorted.end(), tmprObsScore_comparator);


    meanObsScore = meanObsScore / static_cast<double>(mpSorted.size());
    mpSorted.push_back(std::make_pair(static_cast<size_t>(10000), meanObsScore));



    //std::cout << "[OBS_COMPUTOR]  Done" << std::endl;
    return mpSorted;
}



// ==================================================================================
void Obs_computer::compute_H_subblock (const arma::mat & Xv,
                                       const arma::mat & yi,
                                       const arma::mat & zi,
                                       arma::mat & H13, arma::mat & H47) {

    arma::mat rw = Xv.rows(0,2);
    arma::mat qwr = Xv.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(qwr));
    arma::mat RelRw = yi - rw;

    // dhd_dhu
    double ud = zi(0,0);
    double vd = zi(0,1);
    double xd = (zi(0,0) - Cx) * dx;
    double yd = (zi(0,1) - Cy) * dy;
    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;
    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;
    arma::mat dhd_dhu = J_undistor.i();

    // dhu_dhrl
    arma::mat hrl = Rrw * RelRw;
    arma::mat dhu_dhrl;
    if ( fabs(hrl(2,0)) < 1e-6 ) {
        dhu_dhrl  = arma::zeros<arma::mat>(2,3);
    } else {
    dhu_dhrl << f*ku/(hrl(2,0))   <<   0.0  <<   -hrl(0,0)*f*ku/( std::pow(hrl(2,0), 2.0)) << arma::endr
                    << 0.0    <<  f*kv/(hrl(2,0))   <<  -hrl(1,0)*f*kv/( std::pow( hrl(2,0), 2.0)) << arma::endr;
    }

    arma::mat dh_dhrl = dhd_dhu *dhu_dhrl;
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq( qwr_conj ,  RelRw) * dqbar_by_dq;

    // H matrix subblock (cols 1~3): H13
     H13 = -1.0 * (dh_dhrl *  Rrw);

    // H matrix subblock (cols 4~7): H47
     H47 = dh_dhrl * dhrl_dqwr;

    return;
}


arma::mat Obs_computer::dRq_times_a_by_dq(arma::mat & q,  arma::mat& aMat) {

    assert(aMat.n_rows == 3 && aMat.n_cols == 1);

    double &q0 = q(0,0);
    double &qx = q(1,0);
    double &qy = q(2,0);
    double &qz = q(3,0);

    arma::mat dR_by_dq0(3,3), dR_by_dqx(3,3), dR_by_dqy(3,3), dR_by_dqz(3,3);
    dR_by_dq0 << 2.0*q0 << -2.0*qz << 2.0*qy << arma::endr
              << 2.0*qz << 2.0*q0  << -2.0*qx << arma::endr
              << -2.0*qy << 2.0*qx << 2.0*q0 << arma::endr;

    dR_by_dqx << 2.0*qx << 2.0*qy << 2.0*qz << arma::endr
              << 2.0*qy << -2.0*qx << -2.0*q0 << arma::endr
              << 2.0*qz << 2.0*q0 << -2.0*qx << arma::endr;

    dR_by_dqy << -2.0*qy << 2.0*qx << 2.0*q0 << arma::endr
              << 2.0*qx << 2.0*qy  << 2.0*qz << arma::endr
              << -2.0*q0 << 2.0*qz << -2.0*qy << arma::endr;

    dR_by_dqz << -2.0*qz << -2.0*q0 << 2.0*qx << arma::endr
              << 2.0*q0 << -2.0*qz  << 2.0*qy << arma::endr
              << 2.0*qx << 2.0*qy << 2.0*qz << arma::endr;

    arma::mat RES = arma::zeros<arma::mat>(3,4);
    RES(arma::span(0,2), arma::span(0,0)) = dR_by_dq0 * aMat;
    RES(arma::span(0,2), arma::span(1,1)) = dR_by_dqx * aMat;
    RES(arma::span(0,2), arma::span(2,2)) = dR_by_dqy * aMat;
    RES(arma::span(0,2), arma::span(3,3)) = dR_by_dqz * aMat;

    return RES;
}

arma::mat Obs_computer::dqomegadt_by_domega(const arma::mat & omega, const double & delta_t) {

    double omegamod = arma::norm(omega, 2);
    if(fabs(omegamod) < 1e-8) {
        return arma::zeros<arma::mat>(4,3);
    }

    arma::mat RES;

    RES << dq0_by_domegaA(omega(0,0), omegamod, delta_t)
        << dq0_by_domegaA(omega(1,0), omegamod, delta_t)
        << dq0_by_domegaA(omega(2,0), omegamod, delta_t)
        << arma::endr

       << dqA_by_domegaA(omega(0,0), omegamod, delta_t)
       << dqA_by_domegaB(omega(0,0), omega(1,0), omegamod, delta_t)
       << dqA_by_domegaB(omega(0,0), omega(2,0), omegamod, delta_t)
       << arma::endr

       << dqA_by_domegaB(omega(1,0), omega(0,0), omegamod, delta_t)
       << dqA_by_domegaA(omega(1,0), omegamod, delta_t)
       << dqA_by_domegaB(omega(1,0), omega(2,0), omegamod, delta_t)
       << arma::endr

       << dqA_by_domegaB(omega(2,0), omega(0,0), omegamod, delta_t)
       << dqA_by_domegaB(omega(2,0), omega(1,0), omegamod, delta_t)
       << dqA_by_domegaA(omega(2,0), omegamod, delta_t)
       << arma::endr;

    return RES;
}

}

