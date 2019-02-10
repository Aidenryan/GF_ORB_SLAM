#include "H_comp.h"

arma::mat jacob_undistor_fm(Camera& camera,  arma::mat& uvd) {

    double &Cx = camera.Cx;
    double &Cy = camera.Cy;
    double &k1 = camera.k1;
    double &k2 = camera.k2;
    double &dx = camera.dx;
    double &dy = camera.dy;

    double ud = uvd(0,0);
    double vd = uvd(0,1);
    double xd = (uvd(0,0) - Cx) * dx;
    double yd = (uvd(0,1) - Cy) * dy;

    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;

    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);

    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;

    return J_undistor;
}


arma::mat dhd_dhu(Camera& camera,  arma::mat& uvd) {
    double &Cx = camera.Cx;
    double &Cy = camera.Cy;
    double &k1 = camera.k1;
    double &k2 = camera.k2;
    double &dx = camera.dx;
    double &dy = camera.dy;

    double ud = uvd(0,0);
    double vd = uvd(0,1);
    double xd = (uvd(0,0) - Cx) * dx;
    double yd = (uvd(0,1) - Cy) * dy;

    double rd2 = (xd * xd) + (yd * yd);
    double rd4 = rd2 * rd2;

    double uu_ud = (1+k1*rd2+k2*rd4)+(ud-Cx)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);
    double vu_vd = (1+k1*rd2+k2*rd4)+(vd-Cy)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);

    double uu_vd = (ud-Cx)*(k1+2*k2*rd2)*(2*(vd-Cy)*dy*dy);
    double vu_ud = (vd-Cy)*(k1+2*k2*rd2)*(2*(ud-Cx)*dx*dx);

    arma::mat J_undistor;
    J_undistor << uu_ud << uu_vd << arma::endr << vu_ud << vu_vd << arma::endr;

    return J_undistor.i();
}


arma::mat  dhu_dhrl(Camera& camera,   arma::mat& Xv_km1_k,  arma::mat& yi ){

    double &f = camera.f;
    double ku = 1.0 / camera.dx;
    double kv = 1.0 / camera.dy;
    arma::mat rw = Xv_km1_k.rows(0,2);
    arma::mat quat = Xv_km1_k.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(quat));

    double theta = yi(3,0);
    double phi = yi(4,0);
    double rho = yi(5,0);
    arma::colvec mi(3);
    mi << std::cos(phi)*std::sin(theta) << -std::sin(phi) << std::cos(phi)*std::cos(theta) << arma::endr;

    arma::mat hc = Rrw*( (yi.rows(0,2) - rw)*rho + mi );
    double hcx = hc(0,0);
    double hcy = hc(1,0);
    double hcz = hc(2,0);

    arma::mat a;
    a << f*ku/(hcz)  << 0.0  <<  -hcx*f*ku/(std::pow(hcz, 2.0)) << arma::endr
      << 0.0  << f*kv/(hcz)  << -hcy*f*kv/(std::pow(hcz, 2.0)) << arma::endr;

    return a;
}


// TODO: convert all vectors to arma::row/colvec types
arma::mat dh_drw( Camera& camera,  arma::mat & Xv_km1_k,
                   arma::mat & yi,  arma::mat & zi) {
    arma::mat dh_dhrl = dhd_dhu(camera, zi) * dhu_dhrl(camera, Xv_km1_k, yi);
    return dh_dhrl * dhrl_drw(Xv_km1_k, yi);
}


arma::mat dRq_times_a_by_dq(arma::mat & q,  arma::mat& aMat) {

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


arma::mat dhrl_dqwr( arma::mat& Xv_km1_k,  arma::mat & yi){

    arma::colvec v1; v1 << 1.0 << -1.0 << -1.0 << -1.0 << arma::endr;
    arma::mat dqbar_by_dq = arma::diagmat(v1);

    arma::mat rw = Xv_km1_k.rows(0,2);
    arma::mat qwr = Xv_km1_k.rows(3,6);
    double lambda = yi(5,0);
    double phi = yi(4,0);
    double theta = yi(3,0);
    arma::colvec mi(3);
    mi << std::cos(phi)*std::sin(theta) << -std::sin(phi) << std::cos(phi)*std::cos(theta) << arma::endr;

    // FIXME
    arma::mat pts = yi.rows(0,2);
    arma::mat right = ((pts - rw)*lambda + mi);
    arma::mat left = qconj(qwr);
    return dRq_times_a_by_dq( left, right )*dqbar_by_dq;
}


arma::mat dh_dqwr(Camera & camera,  arma::mat& Xv_km1_k,
                   arma::mat& yi,  arma::mat& zi) {

    arma::mat dh_dhrl = dhd_dhu(camera, zi) * dhu_dhrl(camera, Xv_km1_k, yi);
    arma::mat Hi12 = dh_dhrl * dhrl_dqwr(Xv_km1_k, yi);

    return Hi12;

}

arma::mat dh_dxv( Camera & camera,  arma::mat & Xv_km1_k,
                   arma::mat& yi,  arma::mat& zi) {
    arma::mat Tmp;
    Tmp = arma::join_horiz(dh_drw(camera, Xv_km1_k, yi, zi), dh_dqwr(camera, Xv_km1_k, yi, zi));
    arma::mat Hi1 = arma::join_horiz(Tmp, arma::zeros<arma::mat>(2,6));

    return Hi1;

}

arma::mat dh_dy(Camera & camera,  arma::mat& Xv_km1_k,
                   arma::mat& yi,  arma::mat& zi) {
    arma::mat dh_dhrlRES = dhd_dhu( camera, zi ) * dhu_dhrl( camera, Xv_km1_k, yi);

    arma::mat rw = Xv_km1_k.rows(0,2);

    arma::mat q = Xv_km1_k.rows(3,6);
    arma::mat Rrw = arma::inv(q2r(q));

    double &lambda = yi(5,0);
    double &phi = yi(4,0);
    double &theta = yi(3,0);

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
    dhrl_dy = arma::join_horiz(dhrl_dy, Rrw*(yi.rows(0,2)-rw));

    arma::mat Hii = (dh_dhrlRES * dhrl_dy);

    return Hii;
}




