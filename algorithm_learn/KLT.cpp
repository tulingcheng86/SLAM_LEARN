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

        // 定义旧帧和特征点
        cv::Mat old_frame, old_gray;
        std::vector<cv::Point2f> p0, p1;

        // 等待几帧让相机暖机
        for(int i = 0; i < 5; i++) p.wait_for_frames();

        // 获取第一帧
        rs2::frameset frames = p.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        old_frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(old_frame, old_gray, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

        // 创建随机颜色
        std::vector<cv::Scalar> colors;
        cv::RNG rng;
        for(int i = 0; i < 100; i++) {
            int r = rng.uniform(0, 256);
            int g = rng.uniform(0, 256);
            int b = rng.uniform(0, 256);
            colors.push_back(cv::Scalar(r, g, b));
        }

        while (true) {
            frames = p.wait_for_frames();
            color_frame = frames.get_color_frame();
            cv::Mat frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat frame_gray;
            cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
	    // 如果p0为空，重新提取特征点
    	    if (p0.empty()) 
    	    {
		cv::goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
		if (p0.empty()) continue; // 如果还是没有特征点，跳过这一帧
            }
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err);

            std::vector<cv::Point2f> good_new;
            for(size_t i = 0; i < p0.size(); i++) {
                if(status[i] == 1) {
                    good_new.push_back(p1[i]);
                    cv::line(frame, p1[i], p0[i], colors[i], 2);
                    cv::circle(frame, p1[i], 5, colors[i], -1);
                }
            }

            cv::imshow("RealSense Color Stream", frame);
            int keyboard = cv::waitKey(30);
            if (keyboard == 'q' || keyboard == 27)
                break;

            old_gray = frame_gray.clone();
            p0 = good_new;
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

