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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    /**通过优化vpKF的位姿，vpMP等优化变量，使得vpMP通过vpKF里的位姿投影到vpKF的二维坐标的重投影误差最小
     * @param vpKF 位姿优化变量相关的关键帧
     * @param vpMP 空间点优化变量相关的mappoint
     * @param nIterations 使用G2o优化次数
     * @param pbStopFlag  是否强制暂停
     * @param nLoopKF  表明在id为nLoopKF处进行的BA
     * @param bRobust  是否使用核函数
     */
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    /**调用BundleAdjustment()，将map中所有keyframe位姿和mappoint位置作为优化遍历进行BA
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    /**
     * 将Covisibility graph中与pKF连接的关键帧放入lLocalKeyFrames作为g2o图的顶点
     * 将被lLocalKeyFrames看到的mappoint放入lLocalMapPoints中，作为g2o图的顶点
     * lFixedCameras储存着能看到lLocalMapPoints，但是又不在lLocalKeyFrames里的关键帧，作为g2o图的顶点
     * 将lLocalMapPoints里的mappoint的每个观测作为一条误差项边
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    
    /**
    * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
    * 只优化Frame的Tcw，不优化MapPoints的坐标
    * 更新pFrame->mvbOutlier
    * 更新了pFrame的位姿，pFrame->SetPose(pose);
    * @param   pFrame Frame
    * @return  inliers数量
    */
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    //顶点为map中所有keyframe
    //边为LoopConnections中的连接关系,以及essential graph中的边：1.扩展树（spanning tree）连接关系，
    //2.闭环连接关系,3.共视关系非常好的连接关系（共视点为100）
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    /**
     * @param pKF1
     * @param vpMatches1 pKF1的特征点与pKF2的mappoint匹配情况
     * @return 
     */
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
