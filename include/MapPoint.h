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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    // 平均的观测方向
    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    
    //返回此mappoint可以被keyframe看到的数量
    int Observations();
    
    //让mappoint知道自己可以被哪些keyframe看到
    /**
     *@param pKF mappoint可以被看到的keyframe
     *@param idx mappoint自己是对应pKF中哪个特征点的序号
     */ 
    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    //返回此MapPoint对应的是哪个pKF中哪个特征点
    int GetIndexInKeyFrame(KeyFrame* pKF);
    //此mappoint是否能被pKF看到
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    //将此mappoint的相关信息继承给pMP，修改自己在其他keyframe的信息，并且“自杀”
    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    //在此mappoint能被看到的特征点中找出最能代表此mappoint的描述子
    void ComputeDistinctiveDescriptors();

    //返回此mappoint的描述子
    cv::Mat GetDescriptor();

    //更新此mappoint参考帧光心到mappoint平均观测方向以及观测距离范围
    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    //used by ORBmatcher::Fuse
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    //创建这个mappoint的keyframe id
    long int mnFirstKFid;
    //创建这个mappoint的Frame id
    long int mnFirstFrame;
    //此mappoint可以被keyframe看到的数量
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    //描述现在是否被tracking跟踪
    bool mbTrackInView;
    int mnTrackScaleLevel;
    //Frame观测此点的观测角的cos值
    float mTrackViewCos;
     // mnTrackReferenceForFrame防止重复添加局部MapPoint
    long unsigned int mnTrackReferenceForFrame;
    //最后一次是被哪一帧跟踪
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    //发起闭环检测的关键帧
    long unsigned int mnCorrectedByKF;
    //在loop中是通过mnCorrectedReference关键帧来修正此点的
    long unsigned int mnCorrectedReference; 
    //global BA的优化变量mappoint结果
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     //此mappoint在世界坐标系中的坐标
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     //记录此MapPoint对应的是哪个KeyFrame中哪个特征点
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     // 平均的观测方向，MapPoint::UpdateNormalAndDepth()
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     //生成这个mappoint的关键帧为参考关键帧
     KeyFrame* mpRefKF;

     // Tracking counters
     //预测这个mappoint点能被多少Frame看到
     //它在SearchLocalPoints()中更新
     int mnVisible;
     //它在TrackLocalMap()中更新
     //表示此mappoint点确实被Frame跟踪到了
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     //将要替代此mappoint的mappoint
     MapPoint* mpReplaced;

     // Scale invariance distances
     // 观测到该点的距离下限，具体看MapPoint::UpdateNormalAndDepth()
     float mfMinDistance;
     // 观测到该点的距离上限，具体看MapPoint::UpdateNormalAndDepth()
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
