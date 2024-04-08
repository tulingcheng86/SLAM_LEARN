![image-20240327190957451](/home/sbim/.config/Typora/typora-user-images/image-20240327190957451.png)

状态估计问题的求解，与两个方程的具体形式，以及噪声服从哪种分布有关。我们按照**运动和观测方程是否为线性**，**噪声是否服从高斯分布**进行分类，分为线性/非线性和高斯/非高斯系统。



# 本书结构

对六自由度的位姿，如何表达它，如何优化它，都需要一定篇幅来介绍，这将是**第三讲和第四讲**的主要内容。

空间中的路标点是如何投影到一张照片上的。这需要解释相机的成像模型，我们将在**第五章**介绍。

我们知道了这些信息，怎么求解上述方程？这需要非线性优化的知识，则是**第六讲**的内容。

我们会在工作点处把系统线性化，并以预测——更新两大步骤进行求解（见**第九讲**）。

我们认为优化技术已经明显优于滤波器技术，只要计算资源允许，我们通常都偏向于使用优化方法（见**第十、十一讲**）。



**整本书公式**

https://zhuanlan.zhihu.com/p/276194541



# 第二讲 编程基础

**学会构建cmakelists.txt，用cmake来编译cpp**



**学会使用库**

**add_library（）**



#  **slam问题基本方程**

![image-20240328203146629](/home/sbim/.config/Typora/typora-user-images/image-20240328203146629.png)





# 第三讲

## **欧式变换**

描述两个坐标系之间的旋转关系，再加上平移，统称为坐标系之间的变换关系



相机视野中某个向量 p，它的坐标为 pc ，而从世界坐标系下看，它的坐标 pw 。这两个坐标之间是如何转换的呢？这时，就需要先得到该点针对机器人坐标系坐标值，再根据机器人位姿转换到世界坐标系中，这个转换关系由一个矩阵 T 来描述。

![image-20240327194850740](/home/sbim/.config/Typora/typora-user-images/image-20240327194850740.png)





## **旋转矩阵**

![image-20240327200325063](/home/sbim/.config/Typora/typora-user-images/image-20240327200325063.png)

![image-20240327200340119](/home/sbim/.config/Typora/typora-user-images/image-20240327200340119.png)







## 齐次坐标

![image-20240327232201106](/home/sbim/.config/Typora/typora-user-images/image-20240327232201106.png)





## 特殊欧式群

![image-20240327232411883](/home/sbim/.config/Typora/typora-user-images/image-20240327232411883.png)

## Eigen



```cpp
// Eigen 以矩阵为基本数据单元。它是一个模板类。它的前三个参数为：数据类型，行，列

// 声明一个 2*3 的 float 矩阵

Eigen::Matrix<float, 2, 3> matrix_23;
// 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是 Eigen::Matrix
```



## 罗德里戈公式 旋转矩阵朝旋转向量转换



**一个旋转轴为 n，角度为 θ 的旋转，显然，它对应的旋转向量为 θn。**



![image-20240327234850777](/home/sbim/.config/Typora/typora-user-images/image-20240327234850777.png)

![image-20240327235215913](/home/sbim/.config/Typora/typora-user-images/image-20240327235215913.png)



## 欧拉角

![image-20240327235316730](/home/sbim/.config/Typora/typora-user-images/image-20240327235316730.png)



## 四元数

四元数（Quaternion）



一个四元数 q 拥有一个实部和三个虚部。

![image-20240327235519967](/home/sbim/.config/Typora/typora-user-images/image-20240327235519967.png)



**我们能用单位四元数表示三维空间中任意一个旋转**

我们知道单位四元数能够表达三维空间的旋转。这种表达方式和旋转矩阵、旋转向量有什么关系呢



![image-20240327235801607](/home/sbim/.config/Typora/typora-user-images/image-20240327235801607.png)

![image-20240327235825517](/home/sbim/.config/Typora/typora-user-images/image-20240327235825517.png)



## 用四元数表示旋转

![image-20240328000134889](/home/sbim/.config/Typora/typora-user-images/image-20240328000134889.png)



**四元数到旋转矩阵的转换**

![image-20240328000248742](/home/sbim/.config/Typora/typora-user-images/image-20240328000248742.png)



## 安装Pangolin/可视化



安装pangolin  git clone --recursive https://github.com/stevenlovegrove/Pangolin.git 

![image-20240328002230162](/home/sbim/.config/Typora/typora-user-images/image-20240328002230162.png)

![image-20240328002502064](/home/sbim/.config/Typora/typora-user-images/image-20240328002502064.png)





```cpp
set(CMAKE_CXX_FLAGS "-std=c++14")
```



# 第四讲 李群李变换

## 群

**群（Group）是一种集合加上一种运算的代数结构**

![image-20240328102623159](/home/sbim/.config/Typora/typora-user-images/image-20240328102623159.png)



![image-20240328103437702](/home/sbim/.config/Typora/typora-user-images/image-20240328103437702.png)



**如果上式成立，那么给定某时刻的 R，我们就能求得一个 ϕ，它描述了 R 在局部的导数关系。与 R 对应的 ϕ 有什么含义呢？后面会看到，ϕ 正是对应到 SO(3) 上的李代数 so(3)**



## 李代数

![image-20240328105830206](/home/sbim/.config/Typora/typora-user-images/image-20240328105830206.png)

![image-20240328110516648](/home/sbim/.config/Typora/typora-user-images/image-20240328110516648.png)





![image-20240328110901771](/home/sbim/.config/Typora/typora-user-images/image-20240328110901771.png)



## 对应关系

![image-20240328112116877](/home/sbim/.config/Typora/typora-user-images/image-20240328112116877.png)



![在这里插入图片描述](https://img-blog.csdnimg.cn/20210507104315102.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2hic3lkc3k=,size_16,color_FFFFFF,t_70#pic_center)



## Sophus

Sophus 安装报错

https://blog.csdn.net/weixin_52402390/article/details/122300882

https://blog.csdn.net/enjoyollb/article/details/104536742







# 第五讲 相机与图像

## 针孔相机模型

![image-20240328171121600](/home/sbim/.config/Typora/typora-user-images/image-20240328171121600.png)





## 内参数矩阵 K

![image-20240328171632354](/home/sbim/.config/Typora/typora-user-images/image-20240328171632354.png)





![image-20240328171650743](/home/sbim/.config/Typora/typora-user-images/image-20240328171650743.png)





![image-20240328171617180](/home/sbim/.config/Typora/typora-user-images/image-20240328171617180.png)







![image-20240328171911091](/home/sbim/.config/Typora/typora-user-images/image-20240328171911091.png)

![image-20240328172751976](/home/sbim/.config/Typora/typora-user-images/image-20240328172751976.png)





## PCL

```
sudo apt-get install libpcl-dev
```

**CMakeLists.txt**要修改

**用c++ 14**

```cpp
cmake_minimum_required( VERSION 2.8 )
project( joinMap )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#添加头文件
include_directories("/usr/include/eigen3")

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )
```



```shell
sbim@sbim-IdeaCentre-GeekPro-17IAB:~/SLAM_LEARN/slam_learn/ch5/joinMap$ pcl_viewer map.pcd 
2024-03-28 19:43:16.931 (   0.008s) [        1F9F2880] vtkContextDevice2D.cxx:32    WARN| Error: no override found for 'vtkContextDevice2D'.
The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.
> Loading map.pcd [PCLVisualizer::setUseVbos] Has no effect when OpenGL version is ≥ 2
[done, 351.117 ms : 1081843 points]
Available dimensions: x y z rgb
```

![image-20240328200736985](/home/sbim/.config/Typora/typora-user-images/image-20240328200736985.png)





### **计算像素在世界坐标系下的位置**

![image-20240328201259469](/home/sbim/.config/Typora/typora-user-images/image-20240328201259469.png)

![image-20240328201306959](/home/sbim/.config/Typora/typora-user-images/image-20240328201306959.png)

```cpp
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
```



简单来说算法的流程就是：

1、**读取每一张图像，并且遍历每一个像素计算**

2、**计算每一张图像中每一个像素在相机坐标系下的坐标**



# 第六讲 非线性优化

经典SLAM模型由一个**运动方程**和一个**观测方程**构成：



![image-20240329144106024](/home/sbim/.config/Typora/typora-user-images/image-20240329144106024.png)



![image-20240329144153464](/home/sbim/.config/Typora/typora-user-images/image-20240329144153464.png)





![image-20240329144309391](/home/sbim/.config/Typora/typora-user-images/image-20240329144309391.png)

![image-20240329144523348](/home/sbim/.config/Typora/typora-user-images/image-20240329144523348.png)

![image-20240329144654183](/home/sbim/.config/Typora/typora-user-images/image-20240329144654183.png)



**最大似然估计的物理意义就是“在什么样的状态 x,y 下，最有可能产生现在观测到的数据 z,u**

![image-20240329144939907](/home/sbim/.config/Typora/typora-user-images/image-20240329144939907.png)



**根据误差方程，再根据标准式。得到以下   J(x，y)**

![image-20240329145113358](/home/sbim/.config/Typora/typora-user-images/image-20240329145113358.png)







## 高斯牛顿法

![image-20240329151514769](/home/sbim/.config/Typora/typora-user-images/image-20240329151514769.png)

https://blog.csdn.net/weixin_45634390/article/details/137146605?csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22137146605%22%2C%22source%22%3A%22weixin_45634390%22%7D









## cere





## G2o

未编译成功









# 第七讲 前端里程计







## 实践：特征提取和匹配





## 对极几何--对极约束求解相机运动

原理
https://blog.csdn.net/weixin_45634390/article/details/137154084?csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22137154084%22%2C%22source%22%3A%22weixin_45634390%22%7D#t9



### 原理

**通过特征匹配找到了两幅图像中对应的特征点，然后通过对极约束和基础矩阵/本质矩阵的计算，得到了相机的旋转矩阵R和平移向量t。**



## 尺度不确定性

![image-20240329191900350](/home/sbim/.config/Typora/typora-user-images/image-20240329191900350.png)



**单目初始化不能只有纯旋转，必须要有一定程度的平移**



## 三角测量-求深度

![image-20240329195459818](/home/sbim/.config/Typora/typora-user-images/image-20240329195459818.png)

![image-20240329195521171](/home/sbim/.config/Typora/typora-user-images/image-20240329195521171.png)









## PnP







![image-20240401114752634](/home/sbim/.config/Typora/typora-user-images/image-20240401114752634.png)







## P3P

P3P 是一种经典的相机定位算法，通常用于解决相机位姿估计问题。P3P 是 Perspective-Three-Point 的缩写，指的是使用至少三个已知的 3D 点和它们在图像中对应的 2D 点，来计算相机的位姿（平移和旋转）的算法。



## 前端里程计 直接法

https://zhuanlan.zhihu.com/p/61883169

## 光流

![image-20240401145811315](/home/sbim/.config/Typora/typora-user-images/image-20240401145811315.png)





## LK光流法

![image-20240401152100694](/home/sbim/.config/Typora/typora-user-images/image-20240401152100694.png)

![image-20240401152402558](/home/sbim/.config/Typora/typora-user-images/image-20240401152402558.png)



**LK 光流跟踪法避免了描述子的计算与匹配，但本身也需要一定的计算量。在我们的计算平台上，**
**使用 LK 光流能够节省一定的计算量，但在具体 SLAM 系统中使用光流还是匹配描述子，**
**最好是亲自做实验测试一下。**



![image-20240401152455075](/home/sbim/.config/Typora/typora-user-images/image-20240401152455075.png)











# 第九讲 实践：设计前端













# 第十讲 后端1

**BA，Bundle Adjustment，是指从视觉重建中提炼出最优的3D模型和相机参数（内参数和外参数）。**









## 非线性优化（BA与图优化）

https://zhuanlan.zhihu.com/p/65666168







# 第十二讲 回环检测

**回环检测模块能够给出除了相邻帧之外的一些时隔更加久远的约束。**

**在某些时候，我们把仅有前端和局部后端的系统称为VO，而把带有回环检测和全局后端的系统称为SLAM。**

## 准确率和召回率

![img](https://zhoush210.github.io/img/in_post/SLAM14/12/1.png)



假阳性又称**感知偏差**，假阴性又称**感知变异**。

**准确率**：Precision=TP/(TP+FP)，描述算法提取的**所有回环**中确实是**真实回环**的概率。

**召回率**：Recall=TP/(TP+FN)，描述在所有**真实回环**中被**正确检测**出来的概率。





## 词袋模型

![image-20240403185608369](/home/sbim/.config/Typora/typora-user-images/image-20240403185608369.png)



## 实践 

https://blog.csdn.net/weixin_45080292/article/details/123225578







# 第十三讲 建图

![image-20240408102110918](/home/sbim/.config/Typora/typora-user-images/image-20240408102110918.png)

![image-20240408102624081](/home/sbim/.config/Typora/typora-user-images/image-20240408102624081.png)
