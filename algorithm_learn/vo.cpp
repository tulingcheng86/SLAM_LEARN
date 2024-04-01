#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 主函数
int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "Usage: visual_odometry img1 img2" << endl;
        return -1;
    }

    // 读取图像
    Mat img1 = imread(argv[1], IMREAD_GRAYSCALE);
    Mat img2 = imread(argv[2], IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()) {
        cout << "Could not open or find the images!" << endl;
        return -1;
    }

    // 初始化ORB检测器
    Ptr<ORB> orb = ORB::create();

    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    // 检测ORB特征并计算描述子
    orb->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, Mat(), keypoints2, descriptors2);

    // 匹配描述子
    BFMatcher matcher(NORM_HAMMING);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // 绘制匹配结果
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);

    // 显示匹配结果
    imshow("Matches", img_matches);
    waitKey(0);

    return 0;
}

