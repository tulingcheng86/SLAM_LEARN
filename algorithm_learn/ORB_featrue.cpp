#include <librealsense2/rs.hpp> // 包含RealSense头文件
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>


int main() {
    try {
        rs2::pipeline p;
        rs2::config cfg;
        // 配置管道以获取彩色图像
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        // 开始流
        p.start(cfg);
        // 假设的相机内参
        double fx = 615.0; // 相机的焦距x
        double fy = 615.0; // 相机的焦距y
        double cx = 320.0; // 主点x
        double cy = 240.0; // 主点y
        cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Mat distCoeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // 假设没有畸变
        // 定义ORB检测器
        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        // 定义匹配器
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);

        // 存储旧帧、关键点和描述符
        cv::Mat old_frame, old_gray, old_descriptors;
        std::vector<cv::KeyPoint> old_keypoints;

        // 等待几帧让相机暖机
        for(int i = 0; i < 5; i++) p.wait_for_frames();

        // 获取第一帧
        rs2::frameset frames = p.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        old_frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);

        // 检测特征点并计算描述符
        orb->detectAndCompute(old_gray, cv::Mat(), old_keypoints, old_descriptors);

        while (true) {
            frames = p.wait_for_frames();
            color_frame = frames.get_color_frame();
            cv::Mat frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat frame_gray;
            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

            // 检测特征点并计算描述符
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb->detectAndCompute(frame_gray, cv::Mat(), keypoints, descriptors);

            if (!old_descriptors.empty() && !descriptors.empty() && old_descriptors.type() == descriptors.type())
            {
            // 匹配描述符
            std::vector<cv::DMatch> matches;
            matcher->match(old_descriptors, descriptors, matches);

            // 选择用于运动估计的匹配点
            std::vector<cv::Point2f> pts_old, pts_new;
            for(size_t i = 0; i < matches.size(); i++) {
                pts_old.push_back(old_keypoints[matches[i].queryIdx].pt);
                pts_new.push_back(keypoints[matches[i].trainIdx].pt);
            }

            // 计算本质矩阵
            cv::Mat E = cv::findEssentialMat(pts_new, pts_old, K, cv::RANSAC, 0.999, 1.0, cv::noArray());

            // 从本质矩阵恢复相对姿态
            cv::Mat R, t;
            cv::recoverPose(E, pts_new, pts_old, K, R, t);

            // 此处 R 和 t 表示从上一帧到当前帧的相机旋转和平移
            // 可以将 R 和 t 应用到相机位姿的估计或更新地图点位置等

            // 绘制匹配结果
            cv::Mat img_matches;
            cv::drawMatches(old_frame, old_keypoints, frame, keypoints, matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            cv::imshow("ORB Feature Matches", img_matches);
            }
            else 
            {
                std::cout << "Descriptors are empty or mismatched types." << std::endl;
            }
            int keyboard = cv::waitKey(30);
            if (keyboard == 'q' || keyboard == 27)
                break;

            // 为下一次迭代准备
            old_frame = frame.clone();
            old_gray = frame_gray.clone();
            old_keypoints = keypoints;
            old_descriptors = descriptors.clone();
        }
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}