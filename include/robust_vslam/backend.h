#ifndef robust_vslam_BACKEND_H
#define robust_vslam_BACKEND_H

#include "robust_vslam/common_include.h"
#include "robust_vslam/frame.h"
#include "robust_vslam/map.h"

namespace robust_vslam
{
class Map;

/**
 * 后端
 * 有单独优化线程，在Map更新时启动优化
 * Map更新由前端触发
 */
class Backend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    /// 设置地图
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 后端线程启动。触发地图更新，启动优化
    void UpdateMap();

    /// 关闭后端线程
    void Stop();

    // 重置后端
    void Reset();

private:
    /// 后端线程
    void BackendLoop();

    /// 对给定关键帧和路标点进行优化
    void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

} // namespace robust_vslam

#endif // robust_vslam_BACKEND_H