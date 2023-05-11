#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
// 初始化参数
int image_show_time = 5000;
int image_count=10;  
string read_filePath = "../images/2/";
Size image_size;  
Size board_size = Size(4,6);

// 缓存角点
vector<Point2f> image_points_vec;  
vector<vector<Point2f>> image_points_vectors;



int main()
{
    // 辅助变量，主要是C++中更安全的字符串数字转换
    std::stringstream StrStm;
    // 输出流
    ofstream fout("../result.txt");  
    
    //用于存储角点个数。
    int count= -1 ;
    cout<<"---------开始提取角点---------"<<endl;
    for(int i = 1 ;i<=image_count; i++)
    {
        // 零时使用变量
        string readImagesFileName;
        string readImagesFilePath;
        // 拼接图片路径和名字，Str在C++中的拼接更安全
        StrStm<<i;
        StrStm>>readImagesFileName;
        readImagesFileName+=".jpg";
        // 读取图片
        Mat readImageSource = imread(read_filePath+readImagesFileName);
        // 读入第一张图片的时候进行一些处理
        if(i==1)
        {
            image_size.width = readImageSource.cols;
            image_size.height =readImageSource.rows;
            // 输出图像的长宽
            cout<<"image_size.width = "<<image_size.width<<endl;
            cout<<"image_size.height = "<<image_size.height<<endl;
        }
        cout<<"image_count-----"<<i<<endl;
        // 提取角点,这边需要注意角点是按照里面的棋盘格开始计算
        if (findChessboardCorners(readImageSource,board_size,image_points_vec)==0)
        {
            //找不到角点
            cout<<"找不到角点!"<<endl; 
            exit(1);
        }
        else
        {
            Mat view_gray;
            // 转换图片为灰度图
            cvtColor(readImageSource,view_gray,COLOR_RGB2GRAY);
            //对角点进行精确化
            find4QuadCornerSubpix(view_gray,image_points_vec,Size(5,5)); 
            //保存角点
            image_points_vectors.push_back(image_points_vec);  
            //在图像上显示角点位置
            drawChessboardCorners(view_gray,board_size,image_points_vec,false); //用于在图片中标记角点
            //显示图片
            imshow("Camera Calibration",view_gray);
            //暂停展示
            waitKey(image_show_time);
        }
        // 清空一些参数
        readImagesFileName.clear();
        StrStm.clear();
    }


    // 角点输出的样例，输出第一张图片的角点
    for (int i=0 ; i<image_count ;i++)
    {
        cout<<" -->"<<image_points_vectors[i][0].x;
        cout<<" -->"<<image_points_vectors[i][0].y;
    }
    cout<<"---------角点提取完成！---------\n";

    //以下是摄像机标定
    cout<<"---------开始标定---------";
    /*棋盘三维信息*/
    // 实际测量得到的标定板上每个棋盘格的大小
    Size square_size = Size(10,10);  
    // 保存标定板上角点的三维坐标 
    vector<vector<Point3f>> object_points; 
    //内外参数
    //摄像机内参数矩阵 
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); 
    // 每幅图像中角点的数量
    vector<int> point_counts;  
    // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); 
    // 每幅图像的平移向量 
    vector<Mat> tvecsMat;  
    // 每幅图像的旋转向量 
    vector<Mat> rvecsMat; 
    //初始化标定板上角点的三维坐标
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
    calibrateCamera(object_points,image_points_vectors,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
    cout<<"---------标定完成---------\n";


    //对标定结果进行评价
    cout<<"---------标定结果---------\n";
    double total_err = 0.0; /* 所有图像的平均误差的总和 */
    double err = 0.0; /* 每幅图像的平均误差 */
    vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
    cout<<"---------每幅图像的标定误差---------\n";
    for (i=0;i<image_count;i++)
    {
        vector<Point3f> tempPointSet=object_points[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet,rvecsMat[i],tvecsMat[i],cameraMatrix,distCoeffs,image_points2);
        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = image_points_vectors[i];
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
    }
    std::cout<<"---------总体平均误差---------"<<total_err/image_count<<"像素"<<endl;
    std::cout<<"---------评价完成---------"<<endl;
    //保存定标结果
    std::cout<<"---------保存定标结果---------"<<endl;
    //保存每幅图像的旋转矩阵 
    Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); 
    fout<<"---------相机内参数矩阵---------"<<endl;
    fout<<cameraMatrix<<endl<<endl;
    fout<<"---------畸变系数---------\n";
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
    fout<<endl;
    std::cout<<"---------完成保存---------"<<endl;
    

    //显示定标结果
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);


    std::cout<<"---------保存矫正图像---------"<<endl;
    string outputFilePath = "../images/output/";
    string imageFileName;
    // std::stringstream StrStm;
    for (int i = 0 ; i != image_count ; i++)
    {
        initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
        StrStm.clear();
        imageFileName.clear();
    
        StrStm<<i+1;
        StrStm>>imageFileName;
        imageFileName+=".jpg";
        cout<<"读取"<<read_filePath+imageFileName<<endl;
        Mat imageSource = imread(read_filePath+imageFileName);
        Mat newimage = imageSource.clone();
        remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);
        StrStm.clear();
        imageFileName.clear();
        StrStm<<i+1;
        StrStm>>imageFileName;
        imageFileName += "_d.jpg";
        //显示图片
        imshow("Camera Calibration",newimage);
        cout<<imageFileName<<endl;
        //暂停展示
        waitKey(image_show_time);
        cout<<"保存"<<outputFilePath+imageFileName<<endl;
        imwrite(outputFilePath+imageFileName,newimage);
    }
    std::cout<<"保存结束"<<endl;
    return 0;
}
