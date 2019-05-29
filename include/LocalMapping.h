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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    // 当检测到闭环时，请求局部地图停止，防止局部地图线程中InsertKeyFrame函数插入新的关键帧
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    //返回mbAcceptKeyFrames，查询局部地图管理器是否繁忙
    bool AcceptKeyFrames();
    // 设置mbAcceptKeyFrames，false表示告诉Tracking，
    //LocalMapping正处于繁忙状态，不能接受插入新的keyframe
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    //返回mlNewKeyFrames是否为空，也就是查询等待处理的关键帧列表是否空
    bool CheckNewKeyFrames();
    /**
    * @brief 处理列表中的关键帧
    * 
    * - 计算Bow，加速三角化新的MapPoints
    * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
    * - 插入关键帧，更新Covisibility图和Essential图，以及spanningtree
    */
    void ProcessNewKeyFrame();
    /** 
     * 1.找出和当前关键帧共视程度前10/20（单目/双目RGBD）的关键帧
     * 2.计算这些关键帧和当前关键帧的F矩阵；
     * 3.在满足对级约束条件下，匹配关键帧之间的特征点，通过BOW加速匹配；
     */
    void CreateNewMapPoints();

    // 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入在mlpRecentAddedMapPoints的质量不好的MapPoints
    void MapPointCulling();
    // 检查并融合与当前关键帧共视程度高的帧重复的MapPoints
    void SearchInNeighbors();

    // 检测并剔除当前帧相邻的关键帧中冗余的关键帧
    // 剔除的标准是：该关键帧的90%的MapPoints可以被其它至少3个关键帧观测到
    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    //是否能接受插入新的keyframe标志位
    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
