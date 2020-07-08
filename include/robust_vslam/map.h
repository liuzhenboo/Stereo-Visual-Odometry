#pragma once
#ifndef MAP_H
#define MAP_H

#include "robust_vslam/common_include.h"
#include "robust_vslam/config.h"

#include "robust_vslam/frame.h"
#include "robust_vslam/mappoint.h"

namespace robust_vslam
{

/**
 * @brief 地图
 * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
 */
class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() { num_active_keyframes_ = Config::Get<int>("num_active_keyframes"); }
    /// 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);
    /// 增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    /// 获取所有地图点
    LandmarksType GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 获取激活地图点
    LandmarksType GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 清理map中观测数量为零的点
    void CleanMap();

    // 重置地图
    void Reset_Map()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        landmarks_.clear();
        active_landmarks_.clear();
        keyframes_.clear();
        active_keyframes_.clear();
        current_frame_ = nullptr;
    }

private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;        // all landmarks
    LandmarksType active_landmarks_; // active landmarks
    KeyframesType keyframes_;        // all key-frames
    KeyframesType active_keyframes_; // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7; // 激活的关键帧数量
};
} // namespace robust_vslam

#endif // MAP_H