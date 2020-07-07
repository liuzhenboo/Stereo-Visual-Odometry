//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "/home/lzb/Projects/robust-vslam/config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry *vo(
        new myslam::VisualOdometry(FLAGS_config_file));

    vo->Init_StereoRos();
    vo->Run();
    //assert(vo->Init_StereoRos() == true);

    //assert(vo->Init() == true);
    //vo->Run();

    return 0;
}
