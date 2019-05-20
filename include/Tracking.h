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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    //这个没有用到
    std::vector<int> mvIniLastMatches;
    //初始化时得到的特征点匹配，大小是mInitialFrame的特征点数量，其值是当前帧特征点序号
    std::vector<int> mvIniMatches;
    //mInitialFrame中待匹配的特征点的像素位置
    std::vector<cv::Point2f> mvbPrevMatched;
    //初始化时三角化投影成功的匹配点对应的3d点
    std::vector<cv::Point3f> mvIniP3D;
    //初始化的第一帧，初始化需要两帧,世界坐标系就是这帧的坐标系
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    //跟踪
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    /**
    * @brief 单目的地图初始化
    *
    * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
    * 得到初始两帧的匹配、相对运动、初始MapPoints
    */
    void MonocularInitialization();
    
    //单目模式下初始化后，开始建图
    //将mInitialFrame和mCurrentFrame都设置为关键帧
    //新建mappoint
    //更新共视关系
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    
    /**
    * 将参考帧关键帧mpReferenceKF的位姿作为当前帧mCurrentFrame的初始位姿；
    * 匹配参考帧关键帧中有对应mappoint的特征点与当前帧特征点，通过dbow加速匹配；
    * 以上一帧的位姿态为初始值，优化3D点重投影误差，得到更精确的位姿以及剔除错误的特征点匹配；
    * @return 如果匹配数大于10，返回true
    */
    bool TrackReferenceKeyFrame();
    /**
     * 更新mLastFrame
     * 更新mlpTemporalPoints
     */
    void UpdateLastFrame();
    
    /**
      * @brief 根据匀速度模型对上一帧mLastFrame的MapPoints与当前帧mCurrentFrame进行特征点跟踪匹配
      * 
      * 1. 非单目情况，需要对上一帧产生一些新的MapPoints（临时）
      * 2. 将上一帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹配
      * 3. 根据匹配优化当前帧的姿态
      * 4. 根据姿态剔除误匹配
      * @return 如果匹配数大于10，返回true
      */
    bool TrackWithMotionModel();

    //BOW搜索候选关键帧，PnP求解位姿
    bool Relocalization();

    //更新局部地图，即更新局部地图关键帧，局部地图mappoint
    void UpdateLocalMap();
    //将mvpLocalKeyFrames中的mappoint，添加到局部地图关键点mvpLocalMapPoints中
    void UpdateLocalPoints();
    
    /**
     * 更新mpReferenceKF，mCurrentFrame.mpReferenceKF
     * 更新局部地图关键帧mvpLocalKeyFrames
     */
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    //在局部地图的mappoint中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
    void SearchLocalPoints();

    //判断是否需要添加新的keyframe
    bool NeedNewKeyFrame();
    /**
    * @brief 创建新的关键帧
    *
    * 对于非单目的情况，同时创建新的MapPoints
    */
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    //在OnlyTracking模式中使用
    //true表明在上一帧中匹配到了足够多的mappoint
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    //参考关键帧，和当前帧共视程度最高的关键帧
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    //
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    //最近新插入的keyframe
    KeyFrame* mpLastKeyFrame;
    // 记录最近的一帧
    Frame mLastFrame;
    //tracking上一次插入mpLastKeyFrame的Frame的ID
    unsigned int mnLastKeyFrameId;
    //上一次Relocalization()使用的Frame ID，最近一次重定位帧的ID
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    //在UpdateLastFrame()更新
    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
