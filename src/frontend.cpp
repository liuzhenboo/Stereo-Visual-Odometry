#include <opencv2/opencv.hpp>
#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/visual_odometry.h"

#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"
#include "myslam/frontend.h"

namespace myslam
{

Frontend::Frontend()
{
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad");
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
    init_landmarks_ = Config::Get<int>("init_landmarks");
    feature_match_error_ = Config::Get<int>("feature_match_error");
    track_mode_ = Config::Get<std::string>("track_mode");
    num_features_tracking_ = Config::Get<int>("num_features_tracking");
    inlier_rate_ = Config::Get<double>("inlier_rate");
    iterationsCount_ = Config::Get<int>("iterationsCount");
    reprojectionError_ = Config::Get<float>("reprojectionError");
    confidence_ = Config::Get<float>("confidence");
    orb_dir_ = Config::Get<std::string>("orb_dir");
    display_scale_ = Config::Get<int>("display_scale");
    display_x_ = Config::Get<int>("display_x");
    display_y_ = Config::Get<int>("display_y");
    t_[0] = 0;
    t_[1] = 0;
    t_[2] = 0;
    //rotation_ << ( 1, 0, 0, 0, 1, 0, 0, 0, 1 );
    //cv::Mat translation_ = cv::Mat::zeros(3, 1, CV_64F);

    cv::FileStorage fSettings(orb_dir_, cv::FileStorage::READ);

    // 1:RGB 0:BGR
    int nRGB = fSettings["Camera.RGB"];

    if (nRGB)
        std::cout << "- color order: RGB (ignored if grayscale)" << std::endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << std::endl;

    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺度 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈值 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    std::cout << std::endl
              << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << nFeatures << std::endl;
    std::cout << "- Scale Levels: " << nLevels << std::endl;
    std::cout << "- Scale Factor: " << fScaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    std::cout << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
}
// 向系统输入新的图片数据
bool Frontend::AddFrame(myslam::Frame::Ptr frame)
{
    current_frame_ = frame;
    // std::cout << std::endl << "Add a New Stereo Frame!!" << std::endl;

    switch (status_)
    {
    case FrontendStatus::INITING:
        StereoInit_f2f();
        break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
        if (!Track())
        {
            status_ = FrontendStatus::LOST;
            return false;
        } //{Reset();}
        break;
    case FrontendStatus::LOST:
        //Reset();
        break;
    }

    //last_frame_ = current_frame_;
    return true;
}
bool Frontend::StereoInit_f2f()
{
    int num_features_left = DetectFeatures();
    // 设置初始位姿
    Vec3 t;
    t << 0.0, 0.0, 0.0;
    current_frame_->SetPose(SE3(SO3(), t));
    std::cout << "初始位置：" << t << std::endl;

    last_frame_ = current_frame_;
    status_ = FrontendStatus::TRACKING_GOOD;
    return true;
}

bool Frontend::StereoInit()
{
    //std::cout << "num_features_left" << std::endl;

    int num_features_left = DetectFeatures();
    //std::cout << "num_features_left" << std::endl;
    int num_coor_features = FindFeaturesInRight();
    //int inliers = CV_RANSAC();

    if (num_coor_features < num_features_init_)
    {
        LOG(INFO) << "初始化地图时，左右图像匹配点较少...Try Again!";
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success)
    {
        status_ = FrontendStatus::TRACKING_GOOD;
        std::cout << "双目初始化成功，开始跟踪！！" << std::endl;
        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

bool Frontend::BuildInitMap()
{
    // 外参
    //int j = 0;
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    std::cout << "正在初始化地图！" << std::endl;
    double mean_depth = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        if (current_frame_->features_right_[i] == nullptr)
            continue;

        // 像素坐标转换为归一化坐标
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y),
                1.0),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y),
                1.0)};

        Vec3 pworld = Vec3::Zero();
        bool tria = triangulation(poses, points, pworld);
        //std::cout << "tria:" << tria << " " << "pworld[2]:" << pworld[2] << std::endl;

        if (tria && pworld[2] > 0)
        {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
            mean_depth += pworld[2];
        }
    }

    // 初始化地图点数目要保证一定的数目
    if (cnt_init_landmarks >= init_landmarks_)
    {
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points,"
                  << " the mean depth is " << mean_depth / cnt_init_landmarks;
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap();
        return true;
    }
    else
    {
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points，Try again!!";
        return false;
    }
}

int Frontend::DetectFeatures()
{
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_)
    {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);

    int cnt_detected = 0;
    for (auto &kp : keypoints)
    {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    //LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

// 没有利用外参
int Frontend::FindFeaturesInRight()
{
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> points_left, points_right;

    // 先把左图的特征点位置存放到points_left，然后在points_right对应的位置上存放可能的匹配坐标.
    for (auto &feature : current_frame_->features_left_)
    {
        // points_left
        points_left.push_back(feature->position_.pt);
        // points_right
        auto mp = feature->map_point_.lock(); // 查看feature对应的mappoint是否存在；
        mp = nullptr;                         //debug
        if (mp)
        {
            // use projected points as initial guess
            auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            points_right.push_back(cv::Point2f(px[0], px[1]));
        }
        else
        {
            // use same pixel in left iamge
            points_right.push_back(feature->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, points_left,
        points_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        // debug
        //if (status[i] && ((points_left[i].x - points_right[i].x)*(points_left[i].x - points_right[i].x) <= feature_match_error_)) {
        if (status[i] && ((points_left[i].y - points_right[i].y) * (points_left[i].y - points_right[i].y) <= feature_match_error_))
        {
            //if (status[i]){
            // feature与frame关系更新
            cv::KeyPoint kp(points_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        }
        else
        {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "通过左图的特征点位置，在右图估计到 " << num_good_pts << " 个特征点";
    return num_good_pts;
}

bool Frontend::Track()
{
    if (track_mode_ == "stereof2f_pnp")
    {
        std::cout << "跟踪模式为：stereof2f_pnp" << std::endl;
        return StereoF2F_PnP_Track();
    }
    else
    {
        return F2LocalMap_Track();
    }
}
bool Frontend::triangulation_opencv(
    const std::vector<cv::Point2f> &p_1,
    const std::vector<cv::Point2f> &p_2,
    const cv::Mat &t,
    const cv::Mat &K1,
    const cv::Mat &K2,
    std::vector<cv::Point3f> &points)
{
    cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    cv::Mat T2 = (cv::Mat_<float>(3, 4) << 1, 0, 0, -0.12,
                  0, 1, 0, 0,
                  0, 0, 1, 0);
    // cv::Mat K1 = (cv::Mat_<double>(3, 3) << 699.39, 0, 648.854, 0, 699.715, 350.34, 0, 0, 1);
    //Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    std::vector<cv::Point2f> pts_1, pts_2;
    for (size_t i = 0; i < p_1.size(); i++)
    {
        // 将像素坐标转换至相机坐标
        //Point2f((p_1[i].x - K.at<double>(0, 2)) / K.at<double>(0, 0), (p_1[i].y - K.at<double>(1, 2)) / K.at<double>(1, 1));
        pts_1.push_back(cv::Point2f((p_1[i].x - K1.at<double>(0, 2)) / K1.at<double>(0, 0), (p_1[i].y - K1.at<double>(1, 2)) / K1.at<double>(1, 1)));
        pts_2.push_back(cv::Point2f((p_2[i].x - K2.at<double>(0, 2)) / K2.at<double>(0, 0), (p_2[i].y - K2.at<double>(1, 2)) / K2.at<double>(1, 1)));
        //std::cout << "相机系：" << pts_1[i].x << ", " << pts_1[i].y  << "---" << pts_2[i].x << ", " << pts_2[i].y << std::endl;
    }
    cv::Mat pts_4d;

    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    //std::cout << pts_4d << std::endl;

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++)
    {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        points.push_back(p);
    }
    return true;
}
bool Frontend::StereoF2F_PnP_Track()
{
    //int num_features_left = DetectFeatures();

    // 光流跟踪前后帧
    std::vector<cv::Point2f> matched_t1_left; // 光流跟踪到的像素点
    std::vector<cv::Point2f> matched_t2_left;
    std::vector<cv::Point2f> matched_t2_right;
    std::vector<cv::Point2f> matched_t1_right;

    /*for (auto &feature : current_frame_->features_left_)
    {
        matched_t2_left.push_back(feature->position_.pt);
        matched_t2_right.push_back(feature->position_.pt);
        matched_t1_left.push_back(feature->position_.pt);
        matched_t1_right.push_back(feature->position_.pt);
    }*/
    //std::cout << "LK_Robust_Find_MuliImage_MatchedFeatures" << std::endl;
    //int num_f2f_trackedfeatures = LK_Robust_Find_MuliImage_MatchedFeatures(matched_t2_left, matched_t1_left, matched_t1_right);
    int num_f2f_trackedfeatures = ORB_Robust_Find_MuliImage_MatchedFeatures(matched_t2_left, matched_t1_left, matched_t1_right);
    if (num_f2f_trackedfeatures < num_features_tracking_)
    {
        last_frame_ = current_frame_;
        return true;
    }
    //std::cout << "LK_Robust_Find_MuliImage_MatchedFeatures" << std::endl;
    /*for (int i = 0; i < matched_t1_left.size(); i++)
    {
        std::cout << matched_t1_left[i].x << ", " << matched_t1_left[i].y << ",--- " << matched_t1_right[i].x << ", " << matched_t1_right[i].y << std::endl;
    }*/
    //std::cout << "当前帧提取到" << num_features_left << "个特征点，"
    //          << "其中用光流跟踪到上一帧" << num_f2f_trackedfeatures << "个特征点" << std::endl;
    //if (num_f2f_trackedfeatures < num_features_tracking_)
    //{
    //    std::cout << "前后帧跟踪的点过少,跟踪失败！" << std::endl;
    //    return false;
    //}
    // ---------------------
    //三角化上一帧跟踪到的特征点
    // ---------------------
    cv::Mat points3D_t0, points4D_t0;
    cv::Mat projMatrl = camera_left_->projMatr_;
    cv::Mat projMatrr = camera_right_->projMatr_;
    //std::cout << projMatrl << "==" << projMatrr << std::endl;

    //std::cout << "triangulatePoints" << std::endl;
    cv::triangulatePoints(projMatrl, projMatrr, matched_t1_left, matched_t1_right, points4D_t0);
    //std::cout << "triangulatePoints" << std::endl;

    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
    //std::cout << "3D点坐标：" << std::endl << points3D_t0 << std::endl;

    //---------------------------
    //PnP估计姿态；3D->2D
    //-----------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat delta_translation = cv::Mat::zeros(3, 1, CV_64F);
    //for(int i = 0; i < matched_t1_left.size(); i++){
    //    std::cout << matched_t1_left[i].x << ", " << matched_t1_left[i].y << "---" << matched_t2_left[i].x << ", " << matched_t2_left[i].y << std::endl;
    //}
    //bool sum_estimate_pose = slambook_EstimatePose_PnP(projMatrl, matched_t2_left, points3D_t0, rotation, //translation);
    //std::cout << "slambook_EstimatePose_PnP " << std::endl;
    bool sum_estimate_pose = slambook_EstimatePose_PnP(projMatrl, matched_t2_left, points3D_t0, rotation, translation);
    //std::cout << "slambook_EstimatePose_PnP " << std::endl;

    //bool sum_estimate_pose = opencv_EstimatePose_PnP(projMatrl, matched_t2_left, points3D_t0, rotation, translation);
    //if (!sum_estimate_pose)
    //    return false;
    std::cout << "f2f rotation:" << std::endl
              << rotation << std::endl;
    std::cout << "f2f translation:" << std::endl
              << translation << std::endl;

    // 5cm,100cm，保证帧之间的运动合适
    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
    /*double pose_norm2 = (std::pow(translation.at<double>(0), 2) + std::pow(translation.at<double>(1), 2) + std::pow(translation.at<double>(2), 2));*/

    /*if (pose_norm2 < 1 && pose_norm2 > 0.0001)
    {
        std::cout << "有效的位姿估计！" << std::endl;
        std::cout << translation << std::endl;
        Px_ = Px_ + translation.at<double>(0);
        Py_ = Py_ + translation.at<double>(1);
        Pz_ = Pz_ + translation.at<double>(2);
        last_frame_ = current_frame_;
    }*/

    if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1)
    //if (1)
    {
        double pose_norm2 = (std::pow(translation.at<double>(0), 2) + std::pow(translation.at<double>(1), 2) + std::pow(translation.at<double>(2), 2));
        if (pose_norm2 < 1 && pose_norm2 > 0.00001)
        {
            std::cout << "有效的位姿估计！" << std::endl;
            delta_translation = rotation_ * translation;
            std::cout << delta_translation << std::endl;
            Px_ = Px_ + delta_translation.at<double>(0);
            Py_ = Py_ + delta_translation.at<double>(1);
            Pz_ = Pz_ + delta_translation.at<double>(2);
            rotation_ = rotation_ * rotation;
            last_frame_ = current_frame_;
        }
    }
    else
    {

        //std::cout   << std::endl;
    }

    std::cout << "global translation:" << std::endl
              << Px_ << ", " << Py_ << ", " << Pz_ << std::endl;
    display(trajectory_, Px_, Pz_);
    return true;
}
bool Frontend::slambook_EstimatePose_PnP(cv::Mat &projMatrl,
                                         std::vector<cv::Point2f> &pointsLeft_t2,
                                         cv::Mat &points3D_t0,
                                         cv::Mat &rotation,
                                         cv::Mat &translation)
{

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
                                projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

    int iterationsCount = iterationsCount_;       // number of Ransac iterations.
    float reprojectionError = reprojectionError_; // maximum allowed distance to consider it an inlier.
    float confidence = confidence_;               // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::Mat inliers;
    //cv::solvePnP(points3D_t0, pointsLeft_t2, intrinsic_matrix, Mat(), rvec, translation, false);
    cv::solvePnPRansac(points3D_t0, pointsLeft_t2, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);
    cv::Rodrigues(rvec, rotation);
    //std::cout << "三角化结果：" << points3D_t0 << std::endl;
    std::cout << "其中估计位姿的内点数量为: " << inliers.size[0] << std::endl;
    //std::cout << "inliers.size()" << inliers.size[0] << ", " << inliers.size[1] << ", " << inliers.size[2] << std::endl;
    if ((double)inliers.size[0] / (double)pointsLeft_t2.size() < inlier_rate_)
    {
        std::cout << "inliers / tracking point = " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << ",  姿态估计时候内点过少!" << std::endl;
        last_frame_ = current_frame_;
        return false;
    }
    else
    {
        std::cout << "inliers / tracking point: " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << std::endl;
        return true;
    }
}

bool Frontend::isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return norm(I, shouldBeIdentity) < 1e-6;
}
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f Frontend::rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}
void Frontend::display(cv::Mat &trajectory, double p1, double p2)
{
    // draw estimated trajectory
    int x = (int)(p1 * display_scale_) + display_x_;
    int y = (int)(p2 * display_scale_) + display_y_;
    circle(trajectory, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);
    cv::imshow("Trajectory", trajectory);
    cv::waitKey(1);
}
bool Frontend::opencv_EstimatePose_PnP(cv::Mat &projMatrl,
                                       std::vector<cv::Point2f> &pointsLeft_t2,
                                       cv::Mat &points3D_t0,
                                       cv::Mat &rotation,
                                       cv::Mat &translation)
{

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
                                projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                                projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

    int iterationsCount = iterationsCount_;       // number of Ransac iterations.
    float reprojectionError = reprojectionError_; // maximum allowed distance to consider it an inlier.
    float confidence = confidence_;               // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::Mat inliers;
    cv::solvePnPRansac(points3D_t0, pointsLeft_t2, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);
    cv::Rodrigues(rvec, rotation);
    std::cout << "其中估计位姿的内点数量为: " << inliers.size[0] << std::endl;
    //std::cout << "inliers.size()" << inliers.size[0] << ", " << inliers.size[1] << ", " << inliers.size[2] << std::endl;
    if ((double)inliers.size[0] / (double)pointsLeft_t2.size() < inlier_rate_)
    {
        std::cout << "inliers / tracking point = " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << ",  姿态估计时候内点过少！，跟踪失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "inliers / tracking point: " << (double)inliers.size[0] / (double)pointsLeft_t2.size() << std::endl;
        return true;
    }
}

int Frontend::ORB_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t2_left,
                                                        std::vector<cv::Point2f> &points_t1_left,
                                                        std::vector<cv::Point2f> &points_t1_right)
{
    //-- 初始化
    /*cv::Mat descriptors_t1_l, descriptors_t1_r, descriptors_t2_l;
    // used in OpenCV3
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000);

    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    std::vector<cv::KeyPoint> keypoints_t1_l, keypoints_t1_r, keypoints_t2_l;
    detector->detect(last_frame_->left_img_, keypoints_t1_l);
    detector->detect(last_frame_->right_img_, keypoints_t1_r);
    detector->detect(current_frame_->left_img_, keypoints_t2_l);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(last_frame_->left_img_, keypoints_t1_l, descriptors_t1_l);
    descriptor->compute(last_frame_->right_img_, keypoints_t1_r, descriptors_t1_r);
    descriptor->compute(current_frame_->left_img_, keypoints_t2_l, descriptors_t2_l);

    //std::cout << descriptors_1.cols << ", " << descriptors_2.cols;
    //std::cout << descriptors_1.rows << ", " << descriptors_2.rows;

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match1, match2;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_t1_l, descriptors_t1_r, match1);
    matcher->match(descriptors_t1_l, descriptors_t2_l, match2);*/
    // 当前帧图像中提取的特征点集合
    std::vector<cv::KeyPoint> mvKeys1, mvKeys2, mvKeys3;
    // 特征点对应的描述子
    cv::Mat mDescriptors1, mDescriptors2, mDescriptors3;
    (*mpORBextractorLeft)(last_frame_->left_img_, cv::Mat(), mvKeys1, mDescriptors1);
    (*mpORBextractorLeft)(last_frame_->right_img_, cv::Mat(), mvKeys2, mDescriptors2);
    (*mpORBextractorLeft)(current_frame_->left_img_, cv::Mat(), mvKeys3, mDescriptors3);

    //vector<DMatch> matches;
    //find_feature_matches(imLeft, imRight, mvKeys1, mvKeys2, matches);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> match1, match2;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(mDescriptors1, mDescriptors2, match1);
    matcher->match(mDescriptors1, mDescriptors3, match2);
    int des_index = std::min(std::min(mDescriptors2.rows, mDescriptors3.rows), mDescriptors1.rows);
    /*for (int i = 0; i < match1.size(); i++)
    {
        std::cout << "双目左边：" << mvKeys1[match1[i].queryIdx].pt.x << ", " << mvKeys1[match1[i].queryIdx].pt.y << std::endl;
        std::cout << "双目右边：" << mvKeys2[match1[i].trainIdx].pt.x << ", " << mvKeys2[match1[i].trainIdx].pt.y << std::endl;
        std::cout << "--------" << std::endl;
    }*/

    //-- 第四步:匹配点对筛选
    double min_dist = 10000,
           max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离

    //std::cout << "mDescriptors1.rows:" << mDescriptors1.rows << "match1.size():" << match1.size() << "match2: " << match2.size() << "mDescriptors2" << mDescriptors2.rows << "mDescriptors3:" << mDescriptors3.rows << std::endl;
    for (int i = 0; i < des_index; i++)
    {
        double dist = std::max(match1[i].distance, match2[i].distance);
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < des_index; i++)
    {
        if ((match1[i].distance <= std::max(2 * min_dist, 30.0)) && (match2[i].distance <= std::max(2 * min_dist, 30.0)) && (std::abs((mvKeys1[match1[i].queryIdx].pt.y) - (mvKeys2[match1[i].trainIdx].pt.y)) < feature_match_error_))
        {
            cv::Point2f t1_l = mvKeys1[match1[i].queryIdx].pt;
            cv::Point2f t1_r = mvKeys2[match1[i].trainIdx].pt;
            cv::Point2f t2_l = mvKeys3[match2[i].trainIdx].pt;
            //std::cout << "-----------------------" << std::endl;
            //std::cout << match1[i].queryIdx << ", " << match2[i].queryIdx << std::endl;
            points_t2_left.push_back(t2_l);
            points_t1_left.push_back(t1_l);
            points_t1_right.push_back(t1_r);
            //std::cout << "good双目匹配:" << t1_l.x << ", " << t1_l.y << "-----" << t1_r.x << ", " << t1_r.y << std::endl;
        }
    }

    std::cout << "跟踪到上一帧" << points_t2_left.size() << "个特征点" << std::endl;
    return points_t2_left.size();
}

int Frontend::LK_Robust_Find_MuliImage_MatchedFeatures(std::vector<cv::Point2f> &points_t2_left,
                                                       std::vector<cv::Point2f> &points_t1_left,
                                                       std::vector<cv::Point2f> &points_t1_right)
{
    std::vector<cv::Point2f> points_final = points_t1_left;

    // 4次LK光流跟踪
    std::vector<uchar> status1, status2, status3;
    cv::Mat error1, error2, error3;
    // 上一时刻左图与当前左图(t2_l --> t2_r)
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, points_t1_left,
        points_t2_left, status1, error1, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;

    // 当前时刻左图与上一时刻右图
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, last_frame_->right_img_, points_t2_left,
        points_t1_right, status2, error2, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);
    //std::cout << "calcOpticalFlowPyrLK" <<  std::endl;

    // 上一时刻右图与上一时刻左图
    cv::calcOpticalFlowPyrLK(
        last_frame_->right_img_, last_frame_->left_img_, points_t1_right,
        points_final, status3, error3, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // select good point
    int good = 0;
    bool y_error = false;
    bool xy_error = false;
    for (size_t i = 0; i < status1.size(); ++i)
    {
        if (status1[i] && status2[i] && status3[i])
        {
            // 左右目跟踪特征点y值必须差不多
            if ((points_t1_left[i].y - points_t1_right[i].y) * (points_t1_left[i].y - points_t1_right[i].y) <= feature_match_error_)
                y_error = true;
            else
                y_error = false;
            if ((std::pow((points_final[i].x - points_t1_left[i].x), 2) + std::pow((points_final[i].y - points_t1_left[i].y), 2)) <= feature_match_error_)
                xy_error = true;
            else
                xy_error = false;

            if (y_error && xy_error)
            {
                points_t1_left[good] = points_t1_left[i];
                points_t2_left[good] = points_t2_left[i];
                points_t1_right[good] = points_t1_right[i];
                good++;
            }
        }
    }
    points_t1_left.resize(good);
    points_t2_left.resize(good);
    points_t1_right.resize(good);

    return good;
}

bool Frontend::F2LocalMap_Track()
{
    if (last_frame_)
    {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    // 不提特征点，通过光流估计特征点位置
    int num_track_last = TrackLastFrame();
    // 根据三维点来计算，类似于PnP(Perspective-n-Point)
    tracking_inliers_ = G2O_F2LocalMap_EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_)
    {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
        std::cout << "跟踪良好，共跟踪地图点" << tracking_inliers_ << "个" << std::endl;
    }
    else if (tracking_inliers_ > num_features_tracking_bad_)
    {
        // tracking bad
        std::cout << "跟踪较差，共跟踪地图点" << tracking_inliers_ << "个" << std::endl;
        status_ = FrontendStatus::TRACKING_BAD;
    }
    else
    {
        // lost
        std::cout << "跟踪丢失，共跟踪地图点" << tracking_inliers_ << "个" << std::endl;

        status_ = FrontendStatus::LOST;
        return false;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_)
        viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::ICP_SVD_EstimateCurrentPose(const std::vector<cv::Point3f> &pts1, const std::vector<cv::Point3f> &pts2, Eigen::Matrix3d &R_, Eigen::Vector3d &t_)
{
    cv::Point3f p1, p2; // center of mass
    int N = pts1.size();
    //std::cout << "N" << N <<std::endl;
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);
    std::vector<cv::Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    //std::cout << "W=" << W << std::endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    //std::cout << "U=" << U << std::endl;
    //std::cout << "V=" << V << std::endl;

    R_ = U * (V.transpose());
    if (R_.determinant() < 0)
    {
        R_ = -R_;
    }
    t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
    return true;
}

// slide window
int Frontend::G2O_F2LocalMap_EstimateCurrentPose()
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    vertex_pose->setId(0);
    // 这里不是应该设置T_wc，但是改为T_wc之后，跟踪极易丢失！
    vertex_pose->setEstimate((current_frame_->Pose()));
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    // 一元边，利用跟踪得到的(二维特征点，三维地图点)的对应关系来计算当前pose
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp)
        {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i)
        {
            auto e = edges[i];
            if (features[i]->is_outlier_)
            {
                e->computeError();
            }
            if (e->chi2() > chi2_th)
            {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            }
            else
            {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2)
            {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n"
              << current_frame_->Pose().matrix();

    for (auto &feat : features)
    {
        if (feat->is_outlier_)
        {
            // 释放内存资源
            feat->map_point_.reset();
            feat->is_outlier_ = false; // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

bool Frontend::InsertKeyframe()
{
    if (tracking_inliers_ >= num_features_needed_for_keyframe_)
    {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "第" << current_frame_->id_ << "帧即将丢失，设置其为第"
              << current_frame_->keyframe_id_ << "个关键帧";

    SetObservationsForKeyFrame();
    DetectFeatures(); // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_)
        viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame()
{
    for (auto &feat : current_frame_->features_left_)
    {
        auto mp = feat->map_point_.lock();
        if (mp)
            mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints()
{
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr)
        {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::TrackLastFrame()
{
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> points_last, points_current;
    for (auto &feature : last_frame_->features_left_)
    {
        //lock()函数用来判断weak_ptr指向的对象是否存在，是否被释放调；
        //如果对象存在，lock()函数返回一个指向共享对象的shared_ptr，否则返回一个空的shared_ptr.
        points_last.push_back(feature->position_.pt);
        auto mp = feature->map_point_.lock();
        if (mp)
        {
            // use project point
            auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            points_current.push_back(cv::Point2f(px[0], px[1]));
        }
        else
        {
            points_current.push_back(feature->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, points_last,
        points_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            cv::KeyPoint kp(points_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }
    LOG(INFO) << "通过前图的特征点位置，在当前图估计到 " << num_good_pts << " 个特征点";

    return num_good_pts;
}

bool Frontend::Reset()
{
    LOG(INFO) << "跟踪丢失，重新初始化！. ";
    //status_ = FrontendStatus::INITING;
    vo_->Reset();
    current_frame_ = nullptr;
    last_frame_ = nullptr;
    t_[0] = 0;
    t_[1] = 0;
    t_[2] = 0;
    return true;
}
int Frontend::RANSAC(std::vector<std::shared_ptr<Feature>> &Features_1, std::vector<std::shared_ptr<Feature>> &Features_2)
{
    return 0;
}
void Frontend::Set_vo(VisualOdometry *vo)
{
    vo_ = vo;
}

} // namespace myslam