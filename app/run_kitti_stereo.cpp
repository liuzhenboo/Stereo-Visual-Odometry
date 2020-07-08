//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "lzbslam/visual_odometry.h"

DEFINE_string(config_file, "/home/lzb/Projects/robust-vslam/config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    lzbslam::VisualOdometry *vo(
        new lzbslam::VisualOdometry(FLAGS_config_file));

    vo->Init_StereoRos();
    vo->Run();
    //assert(vo->Init_StereoRos() == true);

    //assert(vo->Init() == true);
    //vo->Run();

    return 0;
}
