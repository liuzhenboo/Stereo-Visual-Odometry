//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "robust_vslam/System.h"

DEFINE_string(config_file, "/home/lzb/Projects/robust_vslam/config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    robust_vslam::System *vo(
        new robust_vslam::System(FLAGS_config_file));

    vo->Init_System();
    vo->Run();
    //assert(vo->Init_System() == true);

    //assert(vo->Init() == true);
    //vo->Run();

    return 0;
}
