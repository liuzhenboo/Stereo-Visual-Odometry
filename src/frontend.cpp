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


namespace myslam {

Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad");
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
    init_landmarks_ = Config::Get<int>("init_landmarks");
    feature_match_error_ = Config::Get<int>("feature_match_error");

}
// 向系统输入新的图片数据
bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;
    std::cout << "Add a New Stereo Frame!!" << std::endl;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    // 不提特征点，通过光流估计特征点位置
    int num_track_last = TrackLastFrame();
    // 根据三维点来计算，类似于PnP(Perspective-n-Point)
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
        std::cout << "跟踪良好，共跟踪" <<  tracking_inliers_ << "个" << "内点" << std::endl;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        std::cout << "跟踪较差，共跟踪" <<  tracking_inliers_ << "个" << "内点" << std::endl;
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        std::cout << "跟踪丢失，共跟踪" <<  tracking_inliers_ << "个" << "内点" << std::endl;

        status_ = FrontendStatus::LOST;
        return false;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "第" << current_frame_->id_  << "帧即将丢失，设置其为第"
              << current_frame_->keyframe_id_ << "个关键帧";

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
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

int Frontend::EstimateCurrentPose() {
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
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    // 这里不是应该设置T_wc，但是改为T_wc之后，跟踪极易丢失！
    vertex_pose->setEstimate( (current_frame_->Pose()) );
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    // 一元边，利用跟踪得到的(二维特征点，三维地图点)的对应关系来计算当前pose
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
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
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            // 释放内存资源
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> points_last, points_current;
    for (auto &feature : last_frame_->features_left_) {
        //lock()函数用来判断weak_ptr指向的对象是否存在，是否被释放调；
        //如果对象存在，lock()函数返回一个指向共享对象的shared_ptr，否则返回一个空的shared_ptr.
        points_last.push_back(feature->position_.pt);
        auto mp = feature->map_point_.lock();
        if (mp) {
            // use project point
            auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            points_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
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

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
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

bool Frontend::StereoInit() {
    //std::cout << "num_features_left" << std::endl;

    int num_features_left = DetectFeatures();
    //std::cout << "num_features_left" << std::endl;
    int num_coor_features = FindFeaturesInRight();
    //int inliers = CV_RANSAC();

    if (num_coor_features < num_features_init_) {
        LOG(INFO) << "初始化地图时，左右图像匹配点较少...Try Again!";
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        std::cout << "双目初始化成功，开始跟踪！！"<< std::endl;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);

    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

// 没有利用外参
int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> points_left, points_right; 
    
    // 先把左图的特征点位置存放到points_left，然后在points_right对应的位置上存放可能的匹配坐标.
        for (auto &feature : current_frame_->features_left_) {
        // points_left
        points_left.push_back(feature->position_.pt);
        // points_right
        auto mp = feature->map_point_.lock();// 查看feature对应的mappoint是否存在；
        mp = nullptr; //debug
        if (mp) {
            // use projected points as initial guess
            auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            points_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
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
    for (size_t i = 0; i < status.size(); ++i) {
        // debug
        //if (status[i] && ((points_left[i].x - points_right[i].x)*(points_left[i].x - points_right[i].x) <= feature_match_error_)) {
        if (status[i] && ((points_left[i].y - points_right[i].y)*(points_left[i].y - points_right[i].y) <= feature_match_error_)) {
        //if (status[i]){ 
            // feature与frame关系更新
            cv::KeyPoint kp(points_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "通过左图的特征点位置，在右图估计到 " << num_good_pts << " 个特征点";
    return num_good_pts;
}

bool Frontend::BuildInitMap() {
    // 外参
    //int j = 0;
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    std::cout << "正在初始化地图！" << std::endl;
    double mean_depth = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        
        // 像素坐标转换为归一化坐标
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y), 1.0),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y), 1.0)};
        
        Vec3 pworld = Vec3::Zero();
        bool tria = triangulation(poses, points, pworld);
        //std::cout << "tria:" << tria << " " << "pworld[2]:" << pworld[2] << std::endl;
        
        if ( tria && pworld[2] > 0) 
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
    if(cnt_init_landmarks >= init_landmarks_)
    {
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points," << " the mean depth is " << mean_depth/cnt_init_landmarks;
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

bool Frontend::Reset() {
    LOG(INFO) << "跟踪丢失，重新初始化！. ";
    //status_ = FrontendStatus::INITING;
    vo_ -> Reset();
    return true;
}
int Frontend::RANSAC(std::vector<std::shared_ptr<Feature>> &Features_1, std::vector<std::shared_ptr<Feature>> &Features_2){
    return 0;
}
void Frontend::Set_vo(VisualOdometry* vo)
{
    vo_ = vo;
}

}  // namespace myslam