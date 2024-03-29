#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
  int N = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器
  vector<double> x_data, y_data;               // 存储N个观测数据点
  // 生成x, y的真值
  for (int i = 0; i < N; i++) {
    // 【步骤 1】给定初始值 x_0
    double x = i / 100.0;
    x_data.push_back(x); 
    // y值通过二次曲线公式加上高斯分布噪声产生
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  // 开始Gauss-Newton高斯牛顿迭代
  int iterations = 100;    // 迭代次数
  double cost = 0, lastCost = 0;  // 本次迭代的误差cost和上一次迭代的误差cost

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); // 计时用
  // 开始高斯牛顿法迭代
  for (int iter = 0; iter < iterations; iter++) {
    Matrix3d H = Matrix3d::Zero();// 初始化海森矩阵H为零
    Vector3d b = Vector3d::Zero();// 初始化 bias
    cost = 0;// 每次迭代开始，误差置零
    
    // 【步骤 2】对于第k次迭代，求出当前 N 个数据点的雅可比矩阵 J(x_k) 和误差 f(x_k)
    for (int i = 0; i < N; i++) {
      double xi = x_data[i], yi = y_data[i];  // 第i个数据点的观测值x_i, y_i
      double error = yi - exp(ae * xi * xi + be * xi + ce); //第i个数据点的误差 e_i，对应公式(6.39)
      Vector3d J; // 雅可比矩阵
      // 每个误差项对于状态变量a,b,c的偏导数，对应公式(6.40)
      J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
      J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
      J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc

      // 对应公式(6.41)高斯牛顿法增量方程
      H += inv_sigma * inv_sigma * J * J.transpose(); // H_i = J_i * w_sigma^{-1} * J_i^T
      b += -inv_sigma * inv_sigma * error * J; // b_i = - w_sigma^{-1} * e_i * J_i

      // 误差的平方，对应公式(6.38)
      cost += error * error;
    }

    // 【步骤 3】用Eigen矩阵的ldlt().solve()函数求解线性方程 Hx=b
    Vector3d dx = H.ldlt().solve(b);
    // 判断结果是否有效
    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    // 判断当前误差是否大于上次迭代的误差，迭代到误差无法继续减小时停止
    if (iter > 0 && cost >= lastCost) {
      cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
      break;
    }
    // 用求解估计出的增量\Delta x 更新a,b,c的值
    ae += dx[0];    be += dx[1];    ce += dx[2];

    lastCost = cost;

    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<"\t\testimated params: " << ae << "," << be << "," << ce << endl;
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
  cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
  return 0;
}