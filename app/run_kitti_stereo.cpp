#include "robust_vslam/System.h"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Wrong input parameter!" << std::endl
                  << "./xx yy/cc.yaml" << std::endl;
    }
    std::string config_file_path = argv[1];
    robust_vslam::System *vo(
        new robust_vslam::System(config_file_path));

    vo->Init_System();
    vo->Run();

    return 0;
}
