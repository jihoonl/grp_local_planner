#include <GP/GPRClass2.h>
#include <time.h>

using namespace Eigen;

GPRClass::~GPRClass(){
    cudaFree(invK);
    cudaFree(invKy_meanzero);
}

double GPRClass::kernel_se(VectorXd x1, VectorXd x2){
    double diff2 = (x1 - x2).dot(x1 - x2);

    double tmp = std::exp(-diff2/hyp[1])* hyp[0];
    double var = (diff2 == 0.0)?hyp[2]:0.0;

    tmp += var;

    return tmp + var;
}

MatrixXd GPRClass::kernel(MatrixXd x1, MatrixXd x2){
    MatrixXd KernelMtx = MatrixXd(x1.rows(),x2.rows());
    for(int i = 0; i < x1.rows(); i++){
        for(int j = 0; j < x2.rows(); j++){
            KernelMtx(i,j) = kernel_se(VectorXd(x1.row(i)),VectorXd(x2.row(j)));
        }
    }

    return KernelMtx;
}


MatrixXd GPRClass::kernel_speedup(MatrixXd x1, MatrixXd x2){
    MatrixXd KernelMtx = MatrixXd(x1.rows(), x2.rows());
    
	MatrixXd pmag, qmag;

    double A = 1/hyp[1];
    pmag = A*(x1.cwiseAbs2());
    qmag = (A*(x2.cwiseAbs2())).transpose();
    
    KernelMtx = qmag.replicate(x1.rows(), 1) + pmag.replicate(1, x2.rows()) - 2 * A* Multi(x1, x2.transpose());

    KernelMtx = -KernelMtx;
    KernelMtx = KernelMtx.array().exp().matrix();
    
    KernelMtx = hyp[0]*KernelMtx;
    
    return KernelMtx;
}

void GPRClass::initCUDA(){
    
}

GPRClass::GPRClass(MatrixXd x_, MatrixXd y_, double* hyp_){
    x = x_;
    y = y_;

    x_dim = x.cols();
    y_dim = y.cols();
    data_size = x.rows();

    hyp[0] = hyp_[0];
    hyp[1] = hyp_[1];
    hyp[2] = hyp_[2];

    y_mean = VectorXd(y.colwise().mean());
    y_meanzero = MatrixXd(data_size, y_dim);
    for(int i = 0; i < data_size; i++){
        for(int j = 0; j < y_dim; j++){
            y_meanzero(i,j) = y(i,j) - y_mean(j);
        }
    }

    K = kernel_speedup(x, x);

}

MatrixXd GPRClass::GPRmean(MatrixXd x_star){

    int test_size = x_star.rows();

    K_star = kernel_speedup(x_star,x);
    MatrixXd meanMtx(test_size,y_dim);
    for(int i = 0; i < test_size; i++){
        meanMtx.row(i) = y_mean.transpose();
    }

    cublasHandle_t handle;
    cublasCreate(&handle);

    MatrixXd y_star(test_size,y_dim);
    double* y_mz_d, *y_star_d;
    cudaMalloc((void**)&y_mz_d,sizeof(double)*data_size*y_dim);
    cudaMemcpy(y_mz_d,(double*)y_meanzero.data(),sizeof(double)*data_size* y_dim,cudaMemcpyHostToDevice);
    cudaMalloc((void**)&y_star_d,sizeof(double)*test_size*y_dim);

    kvec_invK = mrdivide(K_star, K );

    y_star = Multi(kvec_invK, y_meanzero);

    return meanMtx + y_star;
}


MatrixXd GPRClass::GPRvar(MatrixXd x_star){
    clock_t begin = clock();
    MatrixXd Cov_star(x_star.rows(),x_star.rows());
    MatrixXd K_starstar = kernel_speedup(x_star,x_star);
    
    MatrixXd K_starT = K_star.transpose();
    
	Cov_star = Multi(kvec_invK, K_starT);
    
    K_starstar = K_starstar - Cov_star;

    return K_starstar;
}



MatrixXd GPRClass::Multi(MatrixXd A, MatrixXd B){
    MatrixXd C(A.rows(),B.cols());

    //allocate GPU memory
    double *A_d, *B_d, *C_d;
    cudaMalloc((void **) &A_d, sizeof(double)*A.rows()*A.cols());
    cudaMalloc((void **) &B_d, sizeof(double)*B.rows()*B.cols());
    cudaMalloc((void **) &C_d, sizeof(double)*C.rows()*C.cols());

    //copy matrix from CPU to GPU
    cudaMemcpy(A_d, (double*)A.data(), sizeof(double)*A.rows()*A.cols(), cudaMemcpyHostToDevice);
    cudaMemcpy(B_d, (double*)B.data(), sizeof(double)*B.rows()*B.cols(), cudaMemcpyHostToDevice);

    //create handler (don't know where it is used)
    cublasHandle_t handle;
    cublasCreate(&handle);

    //calculate
    const double alpha = 1.0f;
    const double beta  = 0.0f;
    //cublasDgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, B->cols(), A->rows(), A->cols(), &alpha, B_d, B->cols(), A_d, A->cols(), &beta, C_d, C->cols());
    cublasDgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, A.rows(), B.cols(), A.cols(), &alpha, A_d, A.rows(), B_d, B.rows(), &beta, C_d, C.rows());


    //copy result from GPU to CPU
    cudaMemcpy((double*)C.data(), C_d, sizeof(double)*C.rows()*C.cols(), cudaMemcpyDeviceToHost);
    //cudaMemcpy((double*)y_star.data(),y_star_d,sizeof(double)*test_size*y_dim,cudaMemcpyDeviceToHost);

    //free memory
    cudaFree(A_d); cudaFree(B_d); cudaFree(C_d);
    return C;
}


MatrixXd GPRClass::mrdivide(Eigen::MatrixXd A, Eigen::MatrixXd B){
    
    return ((B.transpose()).fullPivHouseholderQr().solve(A.transpose())).transpose();
    
}

