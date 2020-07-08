#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
using namespace std;

//void ImageGrabber::Get_stereo_data();
//void System::init();
//void System::Step_ros(Frame::Ptr new_frame);
class ImageGrabber;
class System;
class Frontend;
struct Frame;

struct Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
};

class Frontend
{
public:
    typedef std::shared_ptr<Frontend> Ptr;
    Frame::Ptr current_frame_ = nullptr; // 当前帧
};

class System
{
public:
    typedef std::shared_ptr<System> Ptr;
    Frontend::Ptr frontend_ = nullptr;

    void init();
    void Step_ros(Frame::Ptr new_frame);
};

class ImageGrabber
{
public:
    ImageGrabber(System::Ptr pSLAM) : mp_vo(pSLAM) {}
    System::Ptr mp_vo;
    void Get_stereo_data();
};

void ImageGrabber::Get_stereo_data()
{
    Frame::Ptr new_frame(new Frame);
    mp_vo->Step_ros(new_frame);
}
void System::init()
{
    frontend_ = Frontend::Ptr(new Frontend);
}

void System::Step_ros(Frame::Ptr new_frame)
{
    frontend_->current_frame_ = new_frame;
}

int main()
{
    System::Ptr vo(new System());
    vo->init();
    ImageGrabber igb(vo);
    igb.Get_stereo_data();
    return 0;
}