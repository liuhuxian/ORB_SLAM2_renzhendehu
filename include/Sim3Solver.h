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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

class Sim3Solver
{
public:

    //构造函数
    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    cv::Mat GetEstimatedRotation();
    cv::Mat GetEstimatedTranslation();
    float GetEstimatedScale();


protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    // 根据3对匹配的3D点，计算之间的Sim3变换，也就是计算尺度s旋转R以及平移t
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    void CheckInliers();

    //将vP3Dw中的3d点通过参数Tcw，K投影为2d像素坐标，放入vP2D
    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
    //相机坐标转化为像素坐标
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;

    //mvpMapPoints1在相机mpKF1下的坐标
    std::vector<cv::Mat> mvX3Dc1;
    //mvpMapPoints2在相机mpKF2下的坐标
    std::vector<cv::Mat> mvX3Dc2;
    //与mpKF1匹配的mappoint，大小是匹配点大小
    std::vector<MapPoint*> mvpMapPoints1;
    //与mpKF2匹配的mappoint，大小是匹配点大小
    std::vector<MapPoint*> mvpMapPoints2;
    std::vector<MapPoint*> mvpMatches12;
    //mvnIndices1[i]表示mvpMapPoints1[i]指向的mappoint对应的在mpKF1中的特征点的序号
    std::vector<size_t> mvnIndices1;
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    std::vector<size_t> mvnMaxError1;
    std::vector<size_t> mvnMaxError2;

    int N;
    int mN1;

    // Current Estimation
    //通过sim3计算出的sRt
    cv::Mat mR12i;
    cv::Mat mt12i;
    float ms12i;
    cv::Mat mT12i;
    cv::Mat mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    cv::Mat mBestT12;
    cv::Mat mBestRotation;
    cv::Mat mBestTranslation;
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    //为了随机抽取而准备的序列，如果匹配数为N，则mvAllIndices=[0,1,...,N-1]
    std::vector<size_t> mvAllIndices;

    // Projections
    //mvpMapPoints1在相机mpKF1下的像素坐标
    std::vector<cv::Mat> mvP1im1;
    //mvpMapPoints1在相机mpKF2下的像素坐标
    std::vector<cv::Mat> mvP2im2;

    // RANSAC probability
    double mRansacProb;

    // RANSAC min inliers
    int mRansacMinInliers;

    // RANSAC max iterations
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    cv::Mat mK1;
    cv::Mat mK2;

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
