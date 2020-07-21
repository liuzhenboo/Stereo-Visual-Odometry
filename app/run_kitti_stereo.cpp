// created by liuzhenbo in 2020/7/10

#include "lzb_vio/System.h"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Wrong input parameter!" << std::endl
                  << "./xx yy/cc.yaml" << std::endl;
    }
    std::string config_file_path = argv[1];
    lzb_vio::System *vo(
        new lzb_vio::System(config_file_path));

    vo->Run();

    return 0;
}
