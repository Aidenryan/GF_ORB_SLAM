#include "H_class.h"


namespace OBS {

void H_invDepth::compute_H(const arma::mat & Xv, const arma::mat & yi, const arma::mat & zi,
               arma::mat & dh_dxv, arma::mat & dh_dy) {

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
    dhu_dhrl << f*ku/(hc(2,0))  << 0.0  <<  -hc(0,0)*f*ku/(std::pow(hc(2,0), 2.0)) << arma::endr
      << 0.0  << f*kv/(hc(2,0))  << -hc(1,0)*f*kv/(std::pow(hc(2,0), 2.0)) << arma::endr;

    // dhrl_drw
    arma::mat dhrl_drw = -1.0 * Rrw * yi(5,0);
  
    // dh_dhrl
    arma::mat dh_dhrl = dhd_dhu * dhu_dhrl;
    
    // dh_drw 
    arma::mat dh_drw = dh_dhrl * dhrl_drw;

    // dh_dqwr
    // dhrl_dqwr
    arma::mat qwr_conj = qconj(qwr);
    arma::mat dhrl_dqwr = dRq_times_a_by_dq(qwr_conj, right) * dqbar_by_dq;
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

arma::mat H_invDepth::dRq_times_a_by_dq(arma::mat & q,  arma::mat& aMat) {

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

//
//arma::mat H_invDepth::dh_dxv( arma::mat & Xv,
//                   arma::mat& yi,  arma::mat& zi) {
//    arma::mat Tmp;
//    Tmp = arma::join_horiz(dh_drw(Xv, yi, zi), dh_dqwr(Xv, yi, zi));
//    arma::mat Hi1 = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6));
//
//    return Hi1;
//
//}


//
//arma::mat H_invDepth::dh_dy( arma::mat& Xv,
//                   arma::mat& yi,  arma::mat& zi) {
//    arma::mat dh_dhrlRES = dhd_dhu(zi ) * dhu_dhrl(Xv, yi);
//
//    arma::mat rw = Xv.rows(0,2);
//
//    arma::mat q = Xv.rows(3,6);
//    arma::mat Rrw = arma::inv(q2r(q));
//
//    double &lambda = yi(5,0);
//    double &phi = yi(4,0);
//    double &theta = yi(3,0);
//
//    arma::mat tmp1(3,1), tmp2(3,1);
//    tmp1 << std::cos(phi)*std::cos(theta) << arma::endr
//         << 0.0 << arma::endr
//         << -std::cos(phi)*std::sin(theta) << arma::endr;
//
//    arma::mat dmi_dthetai = Rrw * tmp1;
//
//    tmp2 << -std::sin(phi)*std::sin(theta) << arma::endr
//        << -cos(phi) << arma::endr
//        << -std::sin(phi)*std::cos(theta) << arma::endr;
//    arma::mat dmi_dphii = Rrw*tmp2;
//
//    arma::mat dhrl_dy = arma::join_horiz(lambda*Rrw, dmi_dthetai);
//    dhrl_dy = arma::join_horiz(dhrl_dy, dmi_dphii);
//    dhrl_dy = arma::join_horiz(dhrl_dy, Rrw*(yi.rows(0,2)-rw));
//
//    arma::mat Hii = (dh_dhrlRES * dhrl_dy);
//
//    return Hii;
//}

}


