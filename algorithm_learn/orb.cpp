#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

int main() {
    // 加载图像
    cv::Mat img = cv::imread("/home/sbim/SLAM_LEARN/algorithm_learn/picture/test1.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cout << "Error: Image cannot be loaded." << std::endl;
        return -1;
    }

    // 创建ORB检测器
    int nfeatures = 500; // 最大特征点数量
    float scaleFactor = 1.2f;
    int nlevels = 8;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(nfeatures, scaleFactor, nlevels);

    // 检测关键点
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    // jisuanORB特征描述符
    cv::Mat descriptors;
    detector->compute(img, keypoints, descriptors);

    // 绘制关键点
    cv::Mat img_keypoints;
    drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    // 显示图像
    cv::imshow("ORB Features", img_keypoints);
    cv::waitKey(0);

    return 0;
}