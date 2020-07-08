#pragma once
#ifndef robust_vslam_CONFIG_H
#define robust_vslam_CONFIG_H

#include "robust_vslam/common_include.h"

namespace robust_vslam
{

/**
 * 配置类，使用SetParameterFile确定配置文件位置，这个类就代表这个配置文件，通过类读取配置文件信息。本代码默认是
 * 然后用Get得到对应值
 * 单例模式
 */
class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} // private constructor makes a singleton
public:
    ~Config(); // close the file when deconstructing

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key)
    {
        return T(Config::config_->file_[key]);
    }
};
} // namespace robust_vslam

#endif // robust_vslam_CONFIG_H
