# 张正友相机标定

# 基本原理

首先用一张图来描述小孔成像的基本整个过程

![img](https://pic1.zhimg.com/80/v2-665648ff84735e54ea26e34ed9096ba8_720w.jpg)

- 世界坐标系
  - UVW
- 相机坐标系
  - xyz
- 图像坐标系
  - uv

相机一般在世界坐标系中存在一个点，同时描述一个相机坐标不仅仅需要相机的世界坐标一般来说还需要fov角度，宽高比，和成像平面的深度吗，还有相机的指向和相机头顶的指向向量

- 在计算机图形学中我们可以将世界坐标系进行一系列线性变换使得相机坐标头顶向量指向z轴正方向

- 实际中CCD和CMOS传感器的u轴和v轴可能不是垂直的，所以我们需要使用相机标定进行矫正


## 内参矩阵和外参矩阵

在小孔成像模型下，世界坐标$P(X,Y,Z)$想要变成二维图像的$p(u,v)$需要进行下列变换

- 将$P$从世界坐标系通过刚体变换(旋转和平移)变换到相机坐标系，这个变换过程使用的是相机间的相对位姿，也就是**相机的外参数**。
- 从相机坐标系，通过透视投影变换到相机的成像平面上的像点$p(x,y)$
- 将像点$p$从成像坐标系，通过缩放和平移变换到像素坐标系上点$p(μ,ν)$

$\begin{aligned}
s\left(\begin{array}{l}
\mu \\
\nu \\
1
\end{array}\right) & =\left[\begin{array}{lll}
\alpha & 0 & c_{x} \\
0 & \beta & c_{y} \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{llll}
f & 0 & 0 & 0 \\
0 & f & 0 & 0 \\
0 & 0 & 1 & 0
\end{array}\right]\left[\begin{array}{ll}
R & t \\
0^{T} & 1
\end{array}\right]\left(\begin{array}{l}
X \\
Y \\
Z \\
1
\end{array}\right) \\
& =\left[\begin{array}{cccc}
f_{x} & 0 & c_{x} & 0 \\
0 & f_{y} & c_{y} & 0 \\
0 & 0 & 1 & 0
\end{array}\right]\left[\begin{array}{ll}
R & t \\
0^{T} & 1
\end{array}\right]\left(\begin{array}{l}
X \\
Y \\
Z \\
1
\end{array}\right)
\end{aligned}$

$K=\left[\begin{array}{cccc}
f_{x} & 0 & c_{x} & 0 \\
0 & f_{y} & c_{y} & 0 \\
0 & 0 & 1 & 0
\end{array}\right]$称为内参数

$\left[\begin{array}{ll}
R & t \\
0^{T} & 1
\end{array}\right]$一般称为外参数

## 单应矩阵

单应矩阵Homography实在射影几何中的概念，又称为射影变换-可以理解为一种线性变换或者基变换

![img](https://images2017.cnblogs.com/blog/439761/201801/439761-20180115123854881-40244505.png)

来源:[SLAM入门之视觉里程计(5)：单应矩阵 - Brook_icv - 博客园 (cnblogs.com)](https://www.cnblogs.com/wangguchangqing/p/8287585.html)

定义：

- $N,\pi平面的法向量$
- $m,m^{'} 两个小孔成像模型的成像平面$
- $X_1,X_2分别是第一二相机坐标系下的坐标$
- $d为\pi平面到m摄像机的距离$

则有$N^TX_1 = d$

同时$X_2 = RX_1+T$

则联立有

$X_{2}=R X_{1}+T \frac{1}{d} N^{T} X_{1}=\left(R+T \frac{1}{d} N^{T}\right) X_{1}=H^{\prime} X_{1}$

求解$H^{'}$

则

$H^{'}=R+T\frac{1}{d}N^T$

## 张氏标定法

通过介绍了单应矩阵后，我们知道了它可以表示两个平面之间的映射，那么我们可以运用这个方法到标定中来，通过小孔成像模型，我们可以得到

$P棋盘上的世界坐标,p图像平面的坐标$

则有

$p = K[R|t]P$

则单应矩阵

$H=K[R|t]$

则棋盘格平面的世界坐标已知，图像坐标也已知，则可以拍摄多张照片联立方程组，通过最小二乘法或者最大似然估计进行求解矩阵，即可得到对应的H再求解内参矩阵和外参矩阵

# 实验过程和步骤

## 实验准备

使用WSL2+OpenCV+VsCode+CMake+CPP

OpenCV在windows下实在难装，已经尝试过cmake+minGW编译器，编译过程中报错，换cmake+TDM-GCC，编译通过，但是运行代码时缺失dll动态链接库，无法执行程序，最后使用WSL2+GUI解决

### WSL2

WSL2是Windows Subsystem for Linux的第二代版本，是一种在Windows 10上运行Linux二进制文件的兼容层。WSL2使用虚拟化技术，因此它比WSL1更快，更强大。它还提供了一个完整的Linux内核，可以在WSL2中运行Linux发行版。WSL2还支持Docker和GPU加速，这使得它成为开发人员的理想选择。

最关键的是linux对原生C和C++的支持更好，同时WSL2已经原生支持GUI驱动，可以直接显示图形化界面，不需要xfce4这种远程的linux桌面软件,但是也有很多坑，WSL2经过我的测试对OpenCV支持还不错能够显示，但是对于OPENGL难以显示图形化界面，但是官网说的已经支持OPENGL，一开始以为是我的开发环境问题，故升级到win11和windows内部版本10.0.22621.1555最新版本，但是还是难以支持OpenGL，现觉得可能是AMD核显笔记本和一些显卡驱动的问题，WSL通过ln链接到display:0，可能需要linux的显卡驱动支持。

### OpenCV

OpenCV于1999年由Intel建立，如今由Willow Garage提供支持。OpenCV是一个基于BSD许可发行的跨平台计算机视觉库，可以运行在Linux、Windows和Mac OS操作系统上。它轻量级而且高效――由一系列 C 函数和少量 C++ 类构成，同时提供了Python、Ruby、MATLAB等语言的接口，实现了图像处理和计算机视觉方面的很多通用算法。

## 实验代码解析

### main.cpp

```c++
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main()
{
    ifstream fin("../images.txt"); 
    ofstream fout("../result.txt");  
    //读取每一幅图像，从中提取出角点
    cout<<"开始提取角点………………";
    int image_count=0;  
    Size image_size;  
    // 标定板上每行、列的角点数
    Size board_size = Size(4,6);
    // 缓存角点
    vector<Point2f> image_points_buf;  
    vector<vector<Point2f>> image_points_seq;
    string filename;
    //用于存储角点个数。
    int count= -1 ;
    while (getline(fin,filename))
    {
        image_count++;
        // 用于观察检验输出
        cout<<"image_count = "<<image_count<<endl;
        cout<<"-->count = "<<count;
        Mat imageInput=imread(filename);
        if (image_count == 1)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height =imageInput.rows;
            // 输出图像的长宽
            cout<<"image_size.width = "<<image_size.width<<endl;
            cout<<"image_size.height = "<<image_size.height<<endl;
        }

        // 提取角点,这边需要注意角点是按照里面的棋盘格开始计算
        if (0 == findChessboardCorners(imageInput,board_size,image_points_buf))
        {
            //找不到角点
            cout<<"can not find chessboard corners!\n"; 
            exit(1);
        }
        else
        {

            Mat view_gray;
            // 转换图片为灰度图
            cvtColor(imageInput,view_gray,COLOR_RGB2GRAY);
            //对角点进行精确化
            find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); 
            //保存角点
            image_points_seq.push_back(image_points_buf);  
            //在图像上显示角点位置
            drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
            //显示图片
            imshow("Camera Calibration",view_gray);
            //暂停展示
            waitKey(2000);
        }
    }
    //每张图片上总的角点数
    int total = image_points_seq.size();
    cout<<"total = "<<total<<endl;
    int CornerNum=board_size.width*board_size.height;  
    for (int ii=0 ; ii<total ;ii++)
    {
        if (0 == ii%CornerNum)// 24 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看
        {
            int i = -1;
            i = ii/CornerNum;
            int j=i+1;
            cout<<"--> 第 "<<j <<"图片的数据 --> : "<<endl;
        }
        if (0 == ii%3)    // 此判断语句，格式化输出，便于控制台查看
        {
            cout<<endl;
        }
        else
        {
            cout.width(10);
        }
        //输出所有的角点
        cout<<" -->"<<image_points_seq[ii][0].x;
        cout<<" -->"<<image_points_seq[ii][0].y;
    }
    cout<<"角点提取完成！\n";

    //以下是摄像机标定
    cout<<"开始标定………………";
    /*棋盘三维信息*/
    // 实际测量得到的标定板上每个棋盘格的大小
    Size square_size = Size(10,10);  /* */
    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
    /*内外参数*/
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 每幅图像的平移向量 */
    vector<Mat> rvecsMat; /* 每幅图像的旋转向量 */
    /* 初始化标定板上角点的三维坐标 */
    int i,j,t;
    for (t=0;t<image_count;t++)
    {
        vector<Point3f> tempPointSet;
        for (i=0;i<board_size.height;i++)
        {
            for (j=0;j<board_size.width;j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i=0;i<image_count;i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* 开始标定 */
    calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout<<"标定完成！\n";
    //对标定结果进行评价
    cout<<"开始评价标定结果………………\n";
    double total_err = 0.0; /* 所有图像的平均误差的总和 */
    double err = 0.0; /* 每幅图像的平均误差 */
    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
    cout<<"\t每幅图像的标定误差：\n";
    fout<<"每幅图像的标定误差：\n";
    for (i=0;i<image_count;i++)
    {
        vector<Point3f> tempPointSet=object_points[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);
        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = image_points_seq[i];
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/=  point_counts[i];
        std::cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;
        fout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;
    }
    std::cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;
    fout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl<<endl;
    std::cout<<"评价完成！"<<endl;
    //保存定标结果
    std::cout<<"开始保存定标结果………………"<<endl;
    Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout<<"相机内参数矩阵："<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"畸变系数：\n";
    fout<<distCoeffs<<endl<<endl<<endl;
    for (int i=0; i<image_count; i++)
    {
        fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;
        fout<<rvecsMat[i]<<endl;
        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i],rotation_matrix);
        fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;
        fout<<rotation_matrix<<endl;
        fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;
        fout<<tvecsMat[i]<<endl<<endl;
    }
    std::cout<<"完成保存"<<endl;
    fout<<endl;

    //显示定标结果

    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    std::cout<<"保存矫正图像"<<endl;
    string imageFileName;
    std::stringstream StrStm;
    for (int i = 0 ; i != image_count ; i++)
    {
        std::cout<<"Frame #"<<i+1<<"..."<<endl;
        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
        StrStm.clear();
        imageFileName.clear();
        string filePath="chess";
        StrStm<<i+1;
        StrStm>>imageFileName;
        filePath+=imageFileName;
        filePath+=".bmp";
        Mat imageSource = imread(filePath);
        Mat newimage = imageSource.clone();
        //另一种不需要转换矩阵的方式
        //undistort(imageSource,newimage,cameraMatrix,distCoeffs);
        remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);
        StrStm.clear();
        filePath.clear();
        StrStm<<i+1;
        StrStm>>imageFileName;
        imageFileName += "_d.jpg";
        //显示图片
        imshow("Camera Calibration",newimage);
        //暂停展示
        waitKey(2000);
        // imwrite(imageFileName,newimage);
    }
    std::cout<<"保存结束"<<endl;
    return 0;
}

```

### CMakeLists.txt

使用了opencv中cmake_sample修改而来，我已经做了完整的注释

```cmake
# cmake needs this line
# cmake 最低版本需求
cmake_minimum_required(VERSION 3.1)

# Define project name
# 项目名称
project(Demo)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
# 找到Opencv的package
# 这里是wsl所以可以直接寻找因为make install会安装到/usr/local/bin,而这个目录已在环境变量中
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
# message语法可有可无 用于打印输出一些信息
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
# 添加可执行文件
add_executable(pro main.cpp)

# Link your application with OpenCV libraries
# 将opencv动态库链接到可执行项目pro
target_link_libraries(pro PRIVATE ${OpenCV_LIBS})

```

# 实验结果展示



# 分析和讨论

对标定中的square_size不是很理解





# 参考文献

[SLAM入门之视觉里程计(6)：相机标定 张正友经典标定法详解 - Brook_icv - 博客园 (cnblogs.com)](https://www.cnblogs.com/wangguchangqing/p/8335131.html)