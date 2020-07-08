#ifndef robust_vslam_VIEWER_H
#define robust_vslam_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "robust_vslam/common_include.h"
#include "robust_vslam/frame.h"
#include "robust_vslam/map.h"

namespace robust_vslam
{

/**
 * 可视化
 */
class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    // 后端重置
    void Reset()
    {
        active_keyframes_.clear();
        active_landmarks_.clear();
        current_frame_ = nullptr;
        map_updated_ = false;
    }

    // 更新地图
    void UpdateMap();

private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float *color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    // In stereoicp_f2f tracking mode, plot the features in current frame into an image
    cv::Mat PlotFrameImage_f2f();
    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
} // namespace robust_vslam

#endif // robust_vslam_VIEWER_H