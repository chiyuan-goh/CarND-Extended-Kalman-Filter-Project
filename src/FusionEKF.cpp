#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    VectorXd x(4);
    MatrixXd Pin = MatrixXd::Zero(4, 4);
    MatrixXd Fin = MatrixXd::Identity(4,4);
    MatrixXd Hin = MatrixXd::Zero(2, 4);
    MatrixXd Rin = MatrixXd::Zero(2, 2);
    MatrixXd Qin = MatrixXd::Zero(4, 4);
    ekf_.Init(x, Pin, Fin, Hin, Rin, Qin); //this is just placeholders.

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            float r  = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
//            float rv = measurement_pack.raw_measurements_(2);

            ekf_.x_(0) = r * cos(phi);
            ekf_.x_(1) = -r * sin(phi);

            //TODO: again not sure whther to adjust speed. need to set P'

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            const VectorXd& m = measurement_pack.raw_measurements_;
            ekf_.x_(0) = m(0);
            ekf_.x_(1) = m(1);
            //TODO: not too sure whether I should set the speed yet. need to set P'
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
    float noise_ax = 9., noise_ay = 9.;
    ekf_.Q_ = MatrixXd::Zero(4, 4);
    ekf_.Q_(0, 0) = pow(dt, 4)/4. * noise_ax;
    ekf_.Q_(0, 2) = pow(dt, 3)/2. * noise_ax;
    ekf_.Q_(1, 1) = pow(dt, 4)/4. * noise_ay;
    ekf_.Q_(1, 3) = pow(dt, 3)/2. * noise_ay;
    ekf_.Q_(2, 0) = pow(dt, 3)/2. * noise_ax;
    ekf_.Q_(2, 2) = pow(dt, 2) * noise_ax;
    ekf_.Q_(3, 1) = pow(dt, 3)/2. * noise_ay;
    ekf_.Q_(3, 3) = pow(dt, 2) * noise_ay;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        //needs to calculate the jacobian
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.Update(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
