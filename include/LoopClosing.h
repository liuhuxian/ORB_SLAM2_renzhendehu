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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    //判断mlpLoopKeyFrameQueue是否非空
    bool CheckNewKeyFrames();

    //获取候选闭环关键帧放入mvpEnoughConsistentCandidates
    //如果mvpEnoughConsistentCandidates为空则放回false
    bool DetectLoop();

    /**
    * 1. 候选帧和当前关键帧通过Bow加速描述子的匹配，剔除特征点匹配数少的闭环候选帧
    * 2. 利用RANSAC粗略地计算出当前帧与闭环帧的Sim3，选出较好的那个sim3，确定闭环帧
    * 2. 根据确定的闭环帧和对应的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3。
    * 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）
    */
    bool ComputeSim3();
    /**针对CorrectedPosesMap里的关键帧，mvpLoopMapPoints投影到这个关键帧上与其特征点并进行匹配。
     * 如果匹配成功的特征点本身就有mappoint，就用mvpLoopMapPoints里匹配的点替换，替换下来的mappoint则销毁
     * @param CorrectedPosesMap 表示和当前帧在covisibility相连接的keyframe及其修正的位姿
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    /**
    * @brief 闭环
    *
    * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧）
    * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（相连关键帧+当前帧---闭环帧+相连关键帧）
    * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph
    * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整
    * 5. 创建线程进行全局Bundle Adjustment
    */
    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    //找到的和mpCurrentKF形成闭环检测的关键帧
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    //由DetectLoop()得到的候选关键帧
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    
    //将mpMatchedKF闭环关键帧相连的关键帧全部取出来放入vpLoopConnectedKFs
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    //将mvpLoopMapPoints投影到当前关键帧mpCurrentKF进行投影得到的匹配
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    // 将vpLoopConnectedKFs的MapPoints取出来放入mvpLoopMapPoints
    std::vector<MapPoint*> mvpLoopMapPoints;
    //表示通过ComputeSim3()算的当前帧mpCurrentKF到世界坐标系的变换
    cv::Mat mScw;
    //表示通过ComputeSim3()算的当前帧mpCurrentKF到世界坐标系的Sim3变换
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
