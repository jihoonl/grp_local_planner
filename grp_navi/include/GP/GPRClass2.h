#include <Eigen/Dense>
#include <cublas_v2.h>
#include <cuda_runtime.h>
#include <iostream>
#include <stdio.h>
#include <cusolverDn.h>

using namespace Eigen;

class GPRClass{
public:
    //on CPU
    MatrixXd x;
    MatrixXd y;
    VectorXd y_mean;
    MatrixXd y_meanzero;
    MatrixXd K;
    MatrixXd K_star;
    MatrixXd kvec_invK;
    int x_dim;
    int y_dim;
    int data_size;
    double hyp[3];

    //on GPU
    double* invK;
    double* invKy_meanzero;

    ~GPRClass();
    GPRClass(MatrixXd x_,MatrixXd y_, double* hyp_);

    double kernel_se(VectorXd x1, VectorXd x2);

    MatrixXd kernel(MatrixXd x1, MatrixXd x2);

    MatrixXd kernel_speedup(MatrixXd x1, MatrixXd x2);

    void initCUDA();

    MatrixXd GPRmean(MatrixXd x_star);
    MatrixXd GPRvar(MatrixXd x_star);

    MatrixXd Multi(MatrixXd A, MatrixXd B);
    MatrixXd mrdivide(Eigen::MatrixXd A, Eigen::MatrixXd B);
    
};
