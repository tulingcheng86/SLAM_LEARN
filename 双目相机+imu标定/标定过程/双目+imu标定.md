**参考**

[双目相机与IMU联合标定_vins-fusion双目+imu标定-CSDN博客](https://blog.csdn.net/qq_34935373/article/details/122563824#t2)



# 相机和IMU联合标定
把IMU和相机固定在一起录制bag 包，录制的时候需要充分激励IMU的各个轴，绕3个轴旋转和3个方向的平移

```
rosbag record /cmaera/image_1 /camera/image_2 /imu -o camera_imu.bag
```



## 标定
```
rosrun kalibr kalibr_calibrate_imu_camera --target target_6x7.yaml --bag camera_imu.bag --cam camchain.yaml --imu imu.yaml --show-extraction --bag-from-to 5 45
```

**运行时间会比较长，结果生成yaml文件**

