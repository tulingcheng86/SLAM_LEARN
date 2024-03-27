import pyrealsense2 as rs
import numpy as np
import cv2

# 配置深度和彩色流
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 开始流
pipeline.start(config)

try:
    while True:
        # 等待一对连续的帧：深度和彩色
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 将图像转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 应用颜色映射到深度图像
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # 显示图像
        cv2.imshow('RealSense', np.hstack((color_image, depth_colormap)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 停止流
    pipeline.stop()

cv2.destroyAllWindows()