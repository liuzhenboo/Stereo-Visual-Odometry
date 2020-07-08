#ifndef lzbslam_DATASET_H
#define lzbslam_DATASET_H
#include "lzbslam/camera.h"
#include "lzbslam/common_include.h"
#include "lzbslam/frame.h"

namespace lzbslam
{

/**
 * 数据集读取
 * 构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
 * Init之后可获得相机和下一帧图像
 */
class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string &dataset_path);

    /// 初始化，返回是否成功
    bool Init();
    // 初始化双目，设置左右目类
    bool Stereo_Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const
    {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
} // namespace lzbslam

#endif