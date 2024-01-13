#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>   
using namespace Eigen;  // Eigen related statement; simplifies syntax for declaration of matrices

class KalmanFilter {
public:
    KalmanFilter(const Eigen::MatrixXf& A, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q, 
                 const Eigen::MatrixXf& R, const Eigen::MatrixXf& xk, const Eigen::MatrixXf& P)
        : A(A), H(H), Q(Q), R(R), xk(xk), P(P) {}

    Eigen::MatrixXf update(const Eigen::MatrixXf& zk) {
        // Prediction Step
        Eigen::MatrixXf xp = A * xk;
        Eigen::MatrixXf Pp = A * P * A.transpose() + Q;

        // Update Step
        Eigen::MatrixXf X = H * Pp * H.transpose() + R;
        Eigen::MatrixXf K = Pp * H.transpose() * X.inverse();
        xk = xp + K * (zk - H * xp);
        P = Pp - K * H * Pp;

        return xk;
    }

private:
    Eigen::MatrixXf A; // State transition matrix
    Eigen::MatrixXf H; // State to measurement matrix
    Eigen::MatrixXf Q; // Covariance matrix of process noise
    Eigen::MatrixXf R; // Covariance matrix of measurement noise
    Eigen::MatrixXf xk; // State variable
    Eigen::MatrixXf P;  // State covariance
};

#endif  // KALMAN_FILTER_H
