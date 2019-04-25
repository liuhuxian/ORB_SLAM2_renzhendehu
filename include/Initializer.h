/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:


    /**
    * Initializer构造函数
    * @param ReferenceFrame 输入Initializer参考帧
    * @param sigma  //计算单应矩阵H和基础矩阵得分F时候一个参数
    * @param iterations  RANSAC迭代次数
    */
    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);


    /**
    * 开启初始化
    * @param CurrentFrame 当前帧
    * @param vMatches12 orbmatcher计算的初匹配
    * @param R21
    * @param t21
    * @param vP3D
    * @param vbTriangulated 
    */
    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:


    /**
    * 计算homograpy及其得分
    * @param vbMatchesInliers 匹配点中哪些可以通过H21重投影成功
    * @param score 输出H21得分
    */
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    // 计算fundamental及其得分
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);
    //视觉slam十四讲P147,7.3.3.单应矩阵
    //通过vP1，vP2求得单应矩阵并返回
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    //通过vP1，vP2求得基础矩阵并返回
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);


    /**
    * 计算单应矩阵得分，判断哪些点重投影成功
    * @param vbMatchesInliers 通过H21，H12，匹配点重投影成功情况
    * @param sigma 计算得分时需要的参数
    * @return 单应矩阵得分
    */
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    /**
    * 计算基础得分，判断哪些匹配点重投影成功
    * @param vbMatchesInliers 针对输入的单应矩阵F，匹配点重投影成功情况
    * @return 基础矩阵得分
    */
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    /**
    * 通过输入的F21计算Rt
    * @param vbMatchesInliers 匹配点中哪些可以通过F21重投影成功
    * @param F21 基础
    * @param K 内参
    * @param R21 输出
    * @param t21 输出
    * @param vP3D 其大小为vKeys1大小，表示三角化重投影成功的匹配点的3d点在相机1下的坐标
    * @param vbTriangulated 匹配点中哪些可以通过F21重投影成功
    * @param minParallax 设置的最小视差角余弦值参数，输出Rt模型的视差角小于此值则返回失败
    * @param minTriangulated 匹配点中H21重投影成功的个数如果小于此值，返回失败
    * @return 通过输入的H21计算Rt是否成功
    */
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    /**
    * 通过输入的H21计算Rt
    * @param vbMatchesInliers 匹配点中哪些可以通过H21重投影成功
    * @param H21 单应矩阵
    * @param K 内参
    * @param R21 输出
    * @param t21 输出
    * @param vP3D 其大小为vKeys1大小，表示三角化重投影成功的匹配点的3d点在相机1下的坐标
    * @param vbTriangulated 匹配点中哪些可以通过H21重投影成功
    * @param minParallax 设置的最小视差角余弦值参数，输出Rt模型的视差角小于此值则返回失败
    * @param minTriangulated 匹配点中H21重投影成功的个数如果小于此值，返回失败
    * @return 通过输入的H21计算Rt是否成功
    */
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    /**
    * 计算kp1,kp2是匹配的关键点，它对应着世界坐标系中的一个点。
    * P1,P2是F1，F2对应的投影矩阵。
    * 输出综合考虑了P1,P2,kp1,kp2的在世界坐标系中的齐次坐标3D点坐标
    * @param kp1 
    * @param kp2 
    * @param P1 
    * @param P2
    * @param x3D 输出综合考虑了P1,P2,kp1,kp2的在世界坐标系中的齐次坐标3D点坐标
    */
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    /**
    * 将一个特征点集合归一化到另一个坐标系，使得归一化后的坐标点集合均值为0，一阶绝对矩为1
    * @param vKeys 输入待归一化特征点集合
    * @param vNormalizedPoints  输出归一化后特征点集合
    * @param T    vNormalizedPoints=T*vKeys
    */
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    /**
    * @param vMatches12 orbmatcher计算的初匹配
    * @param vbInliers 匹配点中哪些可以通过H或者F重投影成功
    * @param vP3D 其大小为vKeys1大小，表示三角化重投影成功的匹配点的3d点在相机1下的坐标
    * @param th2 根据三角化重投影误差判断匹配点是否重投影成功的阈值
    * @param vbGood 输出：储存匹配点中哪些三角化重投影成功
    * @param parallax 三角化重投影成功匹配点的视差角
    * @return 匹配点三角化重投影成功的数量
    */
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    //储存着匹配点对在参考帧F1和当前帧F2中的序号
    vector<Match> mvMatches12;
    //描述参考帧F1中特征点匹配情况
    vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    //在CheckFundamental和CheckHomography计算F和H得分的时候有用到的参数
    float mSigma, mSigma2;

    // Ransac max iterations
    //Ransac算法的最大迭代次数
    int mMaxIterations;

    // Ransac sets
    //使用8点法计算F或者H时的RANSAC点对
    vector<vector<size_t> > mvSets;   

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
