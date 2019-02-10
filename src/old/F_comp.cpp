#include "F_comp.h"

//#define DEBUG_MODE

arma::mat dfv_by_dxv(arma::mat Xv, double dt){

    arma::mat omegaOld = Xv.rows(10, 12);
    arma::mat qOld = Xv.rows(3, 6);

    arma::mat dfv_by_dxvRES= arma::eye<arma::mat>(13, 13);
    arma::mat qwt = v2q(omegaOld * dt); // 1x4

    // TODO: remove the dq3_by_dq2 function
    dfv_by_dxvRES.submat(3, 3, 6, 6) = dq3_by_dq2(qwt);

    // constant velocity model
    dfv_by_dxvRES.submat(0, 7, 2, 9) = arma::eye(3,3) * dt;
    dfv_by_dxvRES.submat(3, 10, 6, 12) = dq3_by_dq1(qOld) * dqomegadt_by_domega(omegaOld,dt);

    return dfv_by_dxvRES;
}



arma::mat v2q(arma::mat v){

    double theta = arma::norm(v, 2);
    arma::mat q;
    if(theta < 10e-6) {
        q << 1 << 0 << 0 << 0 << arma::endr;
    } else {
        arma::mat v_n = v / theta;
        arma::mat v_n_reshape =  sin(theta / 2.0) * (arma::reshape(v_n, 1,3) / arma::norm(v_n, 2));
        q << cos(theta / 2.0) << v_n_reshape(0,0) << v_n_reshape(0,1) << v_n_reshape(0,2);
    }
    return q;
}


arma::mat dq3_by_dq1(arma::mat q) {

    q.print("q = ");

    double R = q(0,0);
    double X = q(1,0);
    double Y = q(2,0);
    double Z = q(3,0);

    arma::mat dq3_by_dq1RES;

    dq3_by_dq1RES
            << R << -X << -Y << -Z << arma::endr
            << X <<  R << -Z <<  Y << arma::endr
            << Y <<  Z <<  R << -X << arma::endr
            << Z << -Y <<  X <<  R << arma::endr;

    return dq3_by_dq1RES;
}


arma::mat dq3_by_dq2(arma::mat q) {

    double R = q(0,0);
    double X = q(0,1);
    double Y = q(0,2);
    double Z = q(0,3);

    arma::mat dq3_by_dq2RES;

    dq3_by_dq2RES
            << R << -X << -Y << -Z << arma::endr
            << X << R  <<  Z << -Y << arma::endr
            << Y << -Z <<  R <<  X << arma::endr
            << Z <<  Y << -X <<  R << arma::endr;
#ifdef DEBUG_MODE
    dq3_by_dq2RES.print("dq3_by_dq2RES() = ");
#endif

    return dq3_by_dq2RES;
}

arma::mat dqomegadt_by_domega(arma::mat omega, double delta_t) {

    double omegamod = arma::norm(omega, 2);

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

#ifdef DEBUG_MODE
    RES.print("dqomegadt_by_domega() = ");
#endif

    return RES;
}
