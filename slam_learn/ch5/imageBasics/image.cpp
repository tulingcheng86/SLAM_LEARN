#include<iostream>
#include<chrono>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    cv::Mat image;
    image =cv::imread (argv[1]);//程序从 argv[1]，也就是命令行的第一个参数中读取图像位置。

    if(image.data==nullptr)
    {
        cout<<"bucunzai"<<endl;
        return 0;
    }
    cout<<image.cols<<image.rows<<image.channels()<<endl;
    cv::imshow("image",image);
    cv::waitKey(0);
}