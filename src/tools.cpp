#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rms(4);
    rms << 0, 0, 0, 0;

    for (int i = 0; i < estimations.size(); i++){
        const VectorXd& estimate = estimations[i];
        const VectorXd& truth = ground_truth[i];

        rms = (estimate - truth).array()  * (estimate - truth).array();
    }

    rms /= estimations.size();
    VectorXd result = rms.array().sqrt();

    return result;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    float tol = 0.0001;
    MatrixXd Hj(3, 4);

    float px= x_state(0), py = x_state(1), vx = x_state(2), vy = x_state(3);

    if (abs(px) < tol || abs(py) < tol){
        std::cout << "divide by 0 => returning uninitialized H"<< std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    double t1 = px / sqrt(px * px + py * py);
    double t2 = py / sqrt(px * px + py * py);
    double de = pow(px * px + py * py, 3/2.);
    Hj << t1, t2, 0, 0,
            -py / (px * px + py * py), px /(px * px + py * py), 0 , 0,
            py * (vx*py - vy*px)/de, px * (vy * px - vx * py)/de, t1, t2;

    return Hj;
}
