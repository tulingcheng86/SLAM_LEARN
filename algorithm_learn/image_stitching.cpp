#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    // 检查参数
    if (argc != 3) {
        cout << "Usage: image_stitching img1 img2" << endl;
        return -1;
    }

    // 读取图像
    Mat img1 = imread(argv[1]);
    Mat img2 = imread(argv[2]);

    if (img1.empty() || img2.empty()) {
        cout << "Could not open or find the images!" << endl;
        return -1;
    }

    // 存储两个图像
    vector<Mat> imgs;
    imgs.push_back(img1);
    imgs.push_back(img2);

    // 创建拼接器
    Stitcher::Mode mode = Stitcher::PANORAMA;
    Ptr<Stitcher> stitcher = Stitcher::create(mode);

    // 拼接图像
    Mat pano;
    Stitcher::Status status = stitcher->stitch(imgs, pano);

    if (status != Stitcher::OK) {
        cout << "Can't stitch images, error code = " << int(status) << endl;
        return -1;
    }

    // 显示和保存结果
    imshow("Panorama", pano);
    imwrite("panorama.jpg", pano);
    waitKey(0);

    return 0;
}

