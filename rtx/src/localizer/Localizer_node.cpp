// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "Localizer_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

//#define DEBUG
//#define LOG

Localizer::Localizer() : Node("Localizer")
{
    auto publish_AlignMap =
      [this]() -> void
    {
        static bool flag = true;

        static double l_xOdom = xOdom_;
        static double l_yOdom = yOdom_;
        
        if(flag){

            //pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;

            voxelgrid.setInputCloud(PointcloudMapRaw_pcl_.makeShared());
            voxelgrid.setLeafSize(leafSize_, leafSize_, leafSize_);

            if(flag_show_all_map_== false){
                voxelgrid.setFilterFieldName("x");  // Z軸（高さ）の値でフィルタをかける
                voxelgrid.setFilterLimits(-trim_area_range_ + xOdom_, trim_area_range_ + xOdom_);  // 0.1 ～ 1.0 m の間にある点群を抽出
                //voxelgrid.filter(*downsampled);
                voxelgrid.filter(PointcloudMap_pcl_);

                voxelgrid.setInputCloud(PointcloudMap_pcl_.makeShared());
                voxelgrid.setLeafSize(leafSize_, leafSize_, leafSize_);
                voxelgrid.setFilterFieldName("y");  // Z軸（高さ）の値でフィルタをかける
                voxelgrid.setFilterLimits(-trim_area_range_ + yOdom_, trim_area_range_ + yOdom_);  // 0.1 ～ 1.0 m の間にある点群を抽出
            }
            
            voxelgrid.filter(PointcloudMap_pcl_);

            pcl::toROSMsg(PointcloudMap_pcl_, PointcloudMap_ros_);

            PointcloudMap_ros_.header.stamp = this->get_clock()->now();
            PointcloudMap_ros_.header.frame_id = "map";
            publisher_PointcloudMap_->publish(std::move(PointcloudMap_ros_));

            //Area Map
            // std::cout << "xOdom,yOdom      = "<< xOdom_ << ", "<< yOdom_ << std::endl;
            // area_map_.crop(xOdom_,yOdom_,trim_area_range_,trim_area_range_);
            // publisher_AreaMap_->publish(area_map_.occupancyGrid());

            if (flag_aera_map_ready_){
                //area_map_.paintTrajectory(l_xOdom,l_yOdom,xOdom_,yOdom_,1.5);

                // area_map_.savePGM(
                //     "/home/june/data/test_pcd_all2/global_drivable_area_map_updated.pgm",
                //     "/home/june/data/test_pcd_all2/global_drivable_area_map_updated.yaml");

                if(flag_show_all_map_ == true){
                    auto global_map =area_map_.occupancyGrid();
                    publisher_AreaMap_->publish(global_map);
                }
                else{
                    auto local_map = area_map_.crop(xOdom_, yOdom_,trim_area_range_,trim_area_range_);
                    publisher_AreaMap_->publish(local_map);
                }
            
            }

            /*
            nav_msgs::msg::OccupancyGrid local;
            area_map_->crop(xOdom_,yOdom_,trim_area_range_,trim_area_range_,local);
            local.header.stamp = now();
            area_map_->publish(local);
            */

            // std::string area_map_file = declare_parameter<std::string>("area_map_file",
            //                     "/home/june/data/test_pcd_all2/global_drivable_area_map.pgm");

            //----------------------------------
            // Load AreaMap
            //----------------------------------
            // if(area_map_->loadPGM(area_map_file)){
            //     //RCLCPP_FATAL(get_logger(),"Failed to load AreaMap.");

            //     // rclcpp::shutdown();
            //     // return;
            
            //     area_map_->crop(xOdom_,yOdom_,trim_area_range_,trim_area_range_);
            //     publisher_AreaMap_->publish(area_map_->occupancyGrid());
            // }
            // else{
            //     RCLCPP_FATAL(get_logger(),"Failed to load AreaMap.");
            // }

            l_xOdom = xOdom_;
            l_yOdom = yOdom_;

        }
        //flag = false;
    };

    rclcpp::on_shutdown(
    [this]()
    {
        if(flag_aera_map_ready_ && flag_show_all_map_ == false){
            std::cout << "Save learned AreaMap..." << std::endl;

            std::string pgm = this->get_parameter("AreaMap_pgm").as_string();
            std::string yaml = this->get_parameter("AreaMap_yaml").as_string();

            std::string target = ".pgm";
            std::string replacement = "_updated.pgm";

            std::string::size_type pos = pgm.rfind(target);
            if (pos != std::string::npos) {
                // 見つかった場合、その位置からtargetの長さ分を置換
                pgm.replace(pos, target.length(), replacement);
            }
            printf("save pgm as :%s\n",pgm.c_str());

            target = ".yaml";
            replacement = "_updated.yaml";

            pos = yaml.rfind(target);
            if (pos != std::string::npos) {
                // 見つかった場合、その位置からtargetの長さ分を置換
                yaml.replace(pos, target.length(), replacement);
            }
            printf("save yaml as :%s\n",yaml.c_str());
            
            area_map_.savePGM(pgm,yaml);
            std::cout << "Completed."<< std::endl;
        }
    });


    auto publish_BroadTF =
      [this]() -> void
    {
        broadcastTF();
    };

    declare_parameter("mode",0);
    mode_ = this->get_parameter("mode").as_int();
    if(mode_ < 0){
        flag_show_all_map_= true;
    }
    declare_parameter("pointcloud_map","");
    declare_parameter("input_pointcloud","");
    declare_parameter("input_odom","");
    declare_parameter("input_imu","");
    declare_parameter("threshold_align_fitness_score",0.5);
    declare_parameter("save_pcd_dir","");
    declare_parameter("threshold_dist_save_pcd",5.0);
    declare_parameter("Submap_Range",60.0);
    declare_parameter("SumAlign_fitnessScore",500.0);
    declare_parameter("leaf_size",0.5);
    declare_parameter("matching_leaf_size",0.5);

    declare_parameter("threshold_score_align_with_map",1.0);
    declare_parameter("threshold_adjust_pose",-2.2);
    declare_parameter("threshold_EKF_align_fitness_score",0.5);
    declare_parameter("threshold_mh_dist",0.5);

    declare_parameter("GICP_TransformationEpsilon",0.1);
    declare_parameter("GICP_EuclideanFitnessEpsilon",0.1);
    declare_parameter("GICP_MaximumIterations",30);

    declare_parameter("NDT_OMP_TransformationEpsilon", 0.1);
    declare_parameter("NDT_OMP_StepSize",0.75);
    declare_parameter("NDT_OMP_Resolution", 5.0);
    declare_parameter("NDT_OMP_MaximumIterations", 30);
    declare_parameter("NDT_OMP_Max_Correspondence_Dist", 5.0);
    declare_parameter("NDT_OMP_EuclideanFitnessEpsilon", 0.1);

    declare_parameter("EKF_COV_SYSTEM_NOISE_SXX",0.01);
    declare_parameter("EKF_COV_SYSTEM_NOISE_SYY",0.01);
    declare_parameter("EKF_COV_SYSTEM_NOISE_SYAWYAW",0.01);

    declare_parameter("EKF_COV_SENSOR_NOISE_X",0.01);
    declare_parameter("EKF_COV_SENSOR_NOISE_Y",0.01);

    declare_parameter("EKF_WNOISE_X",0.05);
    declare_parameter("EKF_WNOISE_Y", 0.05);
    declare_parameter("EKF_WNOISE_YAW", 0.01);

    declare_parameter("EKF_VNOISE_X",0.25);
    declare_parameter("EKF_VNOISE_Y", 0.25);
    declare_parameter("EKF_VNOISE_YAW", 0.16);

    declare_parameter("EKF_EVAL_NUM", 10);
    declare_parameter("RADIUS_SEARCH_GROUND", 1.5);

    declare_parameter("InitLatitude", 36.0828676000000002);
    declare_parameter("InitLongitude", 140.0769872999999848);

    declare_parameter("Xinit", 0.0);
    declare_parameter("Yinit", 0.0);
    declare_parameter("YawInit", 0.0);
    
    declare_parameter("x_map_offset", 0.0);
    declare_parameter("y_map_offset", 0.0);
    declare_parameter("z_map_offset", 0.0);
    declare_parameter("map_scale", 1.0);
    declare_parameter("trim_area_range", 80.0);
    declare_parameter("Lidar_height", 0.65);
    
    str_pointcloud_map_     = this->get_parameter("pointcloud_map").as_string();
    str_source_pointcloud_  = this->get_parameter("input_pointcloud").as_string();
    str_input_odom_         = this->get_parameter("input_odom").as_string();
    str_input_imu_          = this->get_parameter("input_imu").as_string();
    threshold_align_fitness_score_= this->get_parameter("threshold_align_fitness_score").as_double();
    str_save_pcd_dir_       = this->get_parameter("save_pcd_dir").as_string();
    threshold_dist_save_pcd_= this->get_parameter("threshold_dist_save_pcd").as_double();
    SubmapRange_            = this->get_parameter("Submap_Range").as_double();
    SumAlignFitnessScore_   = this->get_parameter("SumAlign_fitnessScore").as_double();
    leafSize_               = this->get_parameter("leaf_size").as_double();
    matching_leafSize_      = this->get_parameter("matching_leaf_size").as_double();
    threshold_score_align_with_map_ = this->get_parameter("threshold_score_align_with_map").as_double();
    threshold_adjust_pose_  = this->get_parameter("threshold_adjust_pose").as_double();
    threshold_EKF_align_fitness_score_  = this->get_parameter("threshold_EKF_align_fitness_score").as_double();
    threshold_mh_dist_      = this->get_parameter("threshold_mh_dist").as_double();

    GICP_TransformationEpsilon_ = this->get_parameter("GICP_TransformationEpsilon").as_double();
    GICP_EuclideanFitnessEpsilon_ = this->get_parameter("GICP_EuclideanFitnessEpsilon").as_double();
    GICP_MaximumIterations_ = this->get_parameter("GICP_MaximumIterations").as_int();

    NDT_OMP_TransformationEpsilon_  = this->get_parameter("NDT_OMP_TransformationEpsilon").as_double();//(0.1);
    NDT_OMP_StepSize_               = this->get_parameter("NDT_OMP_StepSize").as_double();//(0.75); //<- これかも
    NDT_OMP_Resolution_             = this->get_parameter("NDT_OMP_Resolution").as_double();//(5.0);
    NDT_OMP_MaximumIterations_      = this->get_parameter("NDT_OMP_MaximumIterations").as_int();//(30);
    NDT_OMP_Max_Correspondence_Dist_ = this->get_parameter("NDT_OMP_Max_Correspondence_Dist").as_double();//(30);
    NDT_OMP_EuclideanFitnessEpsilon_ = this->get_parameter("NDT_OMP_EuclideanFitnessEpsilon").as_double();//(30);

    EKF_Cov_SystemNoise_SXX_       = this->get_parameter("EKF_COV_SYSTEM_NOISE_SXX").as_double();
    EKF_Cov_SystemNoise_SYY_       = this->get_parameter("EKF_COV_SYSTEM_NOISE_SYY").as_double();
    EKF_Cov_SystemNoise_SYawYaw_   = this->get_parameter("EKF_COV_SYSTEM_NOISE_SYAWYAW").as_double();

    EKF_Cov_SensorNoise_X_              = this->get_parameter("EKF_COV_SENSOR_NOISE_X").as_double();
    EKF_Cov_SensorNoise_Y_              = this->get_parameter("EKF_COV_SENSOR_NOISE_Y").as_double();

    EKF_Wnoise_X_                   = this->get_parameter("EKF_WNOISE_X").as_double();
    EKF_Wnoise_Y_                   = this->get_parameter("EKF_WNOISE_Y").as_double();
    EKF_Wnoise_Yaw_                 = this->get_parameter("EKF_WNOISE_YAW").as_double();

    EKF_Vnoise_Y_                   = this->get_parameter("EKF_VNOISE_Y").as_double();
    EKF_Vnoise_Y_                   = this->get_parameter("EKF_VNOISE_Y").as_double();
    EKF_Vnoise_Yaw_                 = this->get_parameter("EKF_VNOISE_YAW").as_double();

    //yawAdjOdom_ = 0.0;

    n_EKF_eval_  = this->get_parameter("EKF_EVAL_NUM").as_int();

    radius_search_ground_      = this->get_parameter("RADIUS_SEARCH_GROUND").as_double();//(30);

    InitLatitude_ = this->get_parameter("InitLatitude").as_double();
    InitLongitude_ = this->get_parameter("InitLongitude").as_double();

    Xinit_ = this->get_parameter("Xinit").as_double();
    Yinit_ = this->get_parameter("Yinit").as_double();
    YawInit_ = this->get_parameter("YawInit").as_double();

    x_map_offset_ = this->get_parameter("x_map_offset").as_double();
    y_map_offset_ = this->get_parameter("y_map_offset").as_double();
    z_map_offset_ = this->get_parameter("z_map_offset").as_double();
    map_scale_ = this->get_parameter("map_scale").as_double();
    trim_area_range_ = this->get_parameter("trim_area_range").as_double();

    Lidar_height_ = this->get_parameter("Lidar_height").as_double();

    printf("pointcloud_map=%s \n",str_pointcloud_map_.c_str());
    printf("source_pointcloud=%s \n",str_source_pointcloud_.c_str());
    printf("input_odom=%s \n",str_input_odom_.c_str());
    printf("threshold_align_fitness_score=%lf \n",threshold_align_fitness_score_);
    printf("str_save_pcd_dir_=%s \n",str_save_pcd_dir_.c_str());
    printf("threshold_dist_save_pcd =%lf \n",threshold_dist_save_pcd_);
    printf("leaf_size =%lf \n",leafSize_);
    printf("matching_leaf_size =%lf \n",matching_leafSize_);
    printf("SumAlign_FitnessScore =%lf \n",SumAlignFitnessScore_);
    printf("threshold_score_align_with_map =%lf \n",threshold_score_align_with_map_);
    printf("threshold_adjust_pose =%lf \n",threshold_adjust_pose_);
    printf("threshold_EKF_align_fitness_score =%lf \n",threshold_EKF_align_fitness_score_);
    printf("threshold_mh_dist =%lf \n",threshold_mh_dist_);

    printf("GICP_TransformationEpsilon =%lf \n",GICP_TransformationEpsilon_);
    printf("GICP_EuclideanFitnessEpsilon =%lf \n",GICP_EuclideanFitnessEpsilon_);
    printf("GICP_MaximumIterations =%d \n",GICP_MaximumIterations_);

    printf("NDT_OMP_TransformationEpsilon =%lf \n",NDT_OMP_TransformationEpsilon_);
    printf("NDT_OMP_StepSize=%lf \n",NDT_OMP_StepSize_);
    printf("NDT_OMP_Resolution =%lf \n",NDT_OMP_Resolution_);
    printf("NDT_OMP_MaximumIterations =%d \n",NDT_OMP_MaximumIterations_);

    printf("RADIUS_SEARCH_GROUND =%lf \n",radius_search_ground_);

    printf("Xinit =%lf \n",Xinit_);
    printf("Yinit =%lf \n",Yinit_);
    printf("YawInit =%lf \n",YawInit_);
    
    printf("pointcloud_map=%s \n",str_pointcloud_map_.c_str());
    printf("x_map_offset_=%lf \n",x_map_offset_);
    printf("y_map_offset_=%lf \n",y_map_offset_);
    printf("z_map_offset_=%lf \n",z_map_offset_);
    printf("map_scale_=%lf \n",map_scale_);
    printf("trim_area_range_=%lf \n",trim_area_range_);

    IMU_FIX_Odom_Status_.data = {0,0,0};
    fixStatus_ = -1;

    subscriberLidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        str_source_pointcloud_.c_str(), rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Localizer::lidar_callback, this, _1));

    subscriberOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        str_input_odom_.c_str(),rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Localizer::odom_callback, this, std::placeholders::_1));

    subscriberImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        str_input_imu_.c_str(), rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Localizer::imu_callback, this, std::placeholders::_1));

    subscriberInitPose_ = this->create_subscription<geometry_msgs::msg::Pose>("initPose", 1, std::bind(&Localizer::initPose_callback, this, _1));
    subscriberFix_= this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", rclcpp::QoS(5).reliable().durability_volatile(), std::bind(&Localizer::fix_callback, this, std::placeholders::_1));

    subscriberInitialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&Localizer::initialpose_callback, this, _1));

    subscriberJoypad_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Localizer::joypad_callback, this, _1));
 
    subscriberLowObjectCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lowobstacles/points", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Localizer::LowObjectPointcloud_callback, this, _1));

    // トピックの送信設定
    publisher_SourcePointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/source_pointcloud", rclcpp::QoS(1));
    publisher_TargetPointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_pointcloud", 1);
    publisher_AlignPointcloud_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_pointcloud", 1);
    publisher_PointcloudMap_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pointcloud", rclcpp::QoS(1));
    publisher_PointcloudMapAlign_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_aligned_pointcloud", rclcpp::QoS(1).best_effort().durability_volatile());

    publisher_PointcloudObject1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object1_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());
    publisher_PointcloudObject2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object2_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());
    publisher_JustAlignPointcloud_= this->create_publisher<sensor_msgs::msg::PointCloud2>("/just_aligned_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());

    publisher_PredictPoseVis_   = this->create_publisher<visualization_msgs::msg::Marker>("/Predist_track", rclcpp::QoS(1).best_effort().durability_volatile());
    publisher_EstimatePoseVis_  = this->create_publisher<visualization_msgs::msg::Marker>("/Estimate_track", rclcpp::QoS(1).best_effort().durability_volatile());
    publisher_MapPoseVis_  = this->create_publisher<visualization_msgs::msg::Marker>("/ScanMatching_track", rclcpp::QoS(1).best_effort().durability_volatile());

    publisher_PoseErrorCov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_error_cov", rclcpp::QoS(1).reliable().durability_volatile());
    publisher_PointcloudMapMatching_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapmatching_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());

    publisher_IMU_FIX_Odom_status_  = this->create_publisher<std_msgs::msg::Int16MultiArray>("/IMU_FIX_Odom_Status", 1);
    
    publisher_AreaMap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/area_map",rclcpp::QoS(1).transient_local());

    load_pointcloud(str_pointcloud_map_.c_str());

    timer_[0] = create_wall_timer(2000ms, publish_AlignMap);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    count_RosbagPlay_ = -100;
    current_timeOdom_ = 0;

    dx_ = 0;
    dy_ = 0;
    dyaw_ = 0;

    ddOdom_ = 0;
    dxOdom_ = 0;
    dyOdom_ = 0;

    xOdom_ = Xinit_;
    yOdom_ = Yinit_;
    zOdom_ = 0;
    yawOdom_ = YawInit_;

    Xpose_ = Xinit_;
    Ypose_ = Yinit_;
    tmp_Yawpose_ =Yawpose_ = YawInit_;

    dxy_mh_ = 0;
    dgap_ = 0;

    minZ_ = 0;

    std::string str_pose_data = str_save_pcd_dir_;
    str_pose_data += "pose.dat";

    std::string str_odom_data = str_save_pcd_dir_;
    str_odom_data += "odom.dat";

    std::string str_ekf_data = str_save_pcd_dir_;
    str_ekf_data += "ekf.dat";

    std::string str_fix_data = str_save_pcd_dir_;
    str_fix_data += "fixM.dat";

    std::string str_fix2_data = str_save_pcd_dir_;
    str_fix2_data += "fix2M.dat";

    f_fix_ = fopen( str_fix_data.c_str(),"w");
    f_fix2_ = fopen( str_fix2_data.c_str(),"w");

    f_pose_ = fopen(str_pose_data.c_str(),"w");
    f_odom_ = fopen( str_odom_data.c_str(),"w");
    f_ekf_  = fopen( str_ekf_data.c_str(),"w");

    //std::cout << "--- pcl::NDT ---" << std::endl;
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt_(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ptr_ndt_ = ndt_;

    ptr_ndt_->setTransformationEpsilon(NDT_OMP_TransformationEpsilon_);
    ptr_ndt_->setStepSize(NDT_OMP_StepSize_);
    ptr_ndt_->setResolution( NDT_OMP_Resolution_);
    ptr_ndt_->setMaximumIterations(NDT_OMP_MaximumIterations_);
    ptr_ndt_->setMaxCorrespondenceDistance(NDT_OMP_Max_Correspondence_Dist_);
    ptr_ndt_->setEuclideanFitnessEpsilon(NDT_OMP_EuclideanFitnessEpsilon_);

    EKF_init(Xinit_,Yinit_,YawInit_);
    
    pcgpc_ = new geo_pos_conv();

    flag_AlignedWithMap_ = false;

    //-- visualization on rviz
    PrePoseVis_.id = 0;
    EstPoseVis_.id = 1;
    MapPoseVis_.id = 2;

    PrePoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    EstPoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    MapPoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;

    PrePoseVis_.scale.x = PrePoseVis_.scale.y = 1.0;
    EstPoseVis_.scale.x = EstPoseVis_.scale.y = 0.8;
    MapPoseVis_.scale.x = MapPoseVis_.scale.y = 1.0;

    //Orange
    PrePoseVis_.color.r = 1.0f; //FF
    PrePoseVis_.color.g = 0.65f; //A5
    PrePoseVis_.color.b = 0.0f; //00
    PrePoseVis_.color.a = 1.0;

    //Green
    EstPoseVis_.color.r = 0.0f; //FF
    EstPoseVis_.color.g = 1.0f; //A5
    EstPoseVis_.color.b = 0.0f; //00
    EstPoseVis_.color.a = 1.0;

    //Blue
    MapPoseVis_.color.r = 0.3f; //FF
    MapPoseVis_.color.g = 0.25f; //A5
    MapPoseVis_.color.b = 1.0f; //00
    MapPoseVis_.color.a = 1.0;

    PrePoseVis_.points.clear();
    EstPoseVis_.points.clear();
    MapPoseVis_.points.clear();

    PrePoseVis_.header.stamp = EstPoseVis_.header.stamp = MapPoseVis_.header.stamp = this->now();

    PrePoseVis_.pose.position.x = PrePoseVis_.pose.position.y = PrePoseVis_.pose.position.z = 0;
    EstPoseVis_.pose.position.x = EstPoseVis_.pose.position.y = EstPoseVis_.pose.position.z = 0;
    MapPoseVis_.pose.position.x = MapPoseVis_.pose.position.y = MapPoseVis_.pose.position.z = 0;
    UrgVis_.pose.position.x = UrgVis_.pose.position.y = UrgVis_.pose.position.z = 0;

    PrePoseVis_.pose.orientation.x = 0;
    PrePoseVis_.pose.orientation.y = 0;
    PrePoseVis_.pose.orientation.z = 0;
    PrePoseVis_.pose.orientation.w = 1;

    EstPoseVis_.pose.orientation.x = 0;
    EstPoseVis_.pose.orientation.y = 0;
    EstPoseVis_.pose.orientation.z = 0;
    EstPoseVis_.pose.orientation.w = 1;

    MapPoseVis_.pose.orientation.x = 0;
    MapPoseVis_.pose.orientation.y = 0;
    MapPoseVis_.pose.orientation.z = 0;
    MapPoseVis_.pose.orientation.w = 1;

    //PrePoseVis_.header.frame_id ="base_link";
    PrePoseVis_.header.frame_id ="map";
    PrePoseVis_.action = visualization_msgs::msg::Marker::ADD;

    //EstPoseVis_.header.frame_id ="base_link";
    EstPoseVis_.header.frame_id ="map";
    EstPoseVis_.action = visualization_msgs::msg::Marker::ADD;

    //MapPoseVis_.header.frame_id ="base_link";
    MapPoseVis_.header.frame_id ="map";
    MapPoseVis_.action = visualization_msgs::msg::Marker::ADD;

    // Area Map 
    declare_parameter("AreaMap_pgm", "");
    declare_parameter("AreaMap_yaml", "");

    std::string pgm = this->get_parameter("AreaMap_pgm").as_string();
    std::string yaml = this->get_parameter("AreaMap_yaml").as_string();
    
    if(!area_map_.load(pgm,yaml)){
        RCLCPP_FATAL(get_logger(),"Failed to load AreaMap.");
    }
    else{
        flag_aera_map_ready_ = true;
    }

}

void Localizer::joypad_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    static sensor_msgs::msg::Joy l_msg = *msg;
    if(msg->buttons[2] > 0 && l_msg.buttons[2]==0){ //Xボタン
        if(flag_show_all_map_ == false){
            printf("Show All Map ON\n");
            flag_show_all_map_ = true;
        }
        else{
            printf("Show All Map OFF\n");
            flag_show_all_map_ = false;
        }
    }
    l_msg = *msg;
}

void Localizer::load_pointcloud(const std::string &file_name){

    // PointCloudMapを読み込む

    pcl::PointCloud<pcl::PointXYZI> tmp_PointcloudMapRaw_pcl_;

    if(pcl::io::loadPCDFile(file_name, tmp_PointcloudMapRaw_pcl_) == 0){

        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;

        voxelgrid.setLeafSize(leafSize_, leafSize_, leafSize_);

        voxelgrid.setInputCloud(tmp_PointcloudMapRaw_pcl_.makeShared());
        voxelgrid.filter(*downsampled);
        
        PointcloudMapRaw_pcl_ = *downsampled;

        //For Rinkai-Area Map ---
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        //double scale_factor = map_scale_;

        transform(0, 0) = map_scale_;
        transform(1, 1) = map_scale_;
        transform(2, 2) = map_scale_;

        transform(0, 3) = -x_map_offset_*map_scale_;
        transform(1, 3) = -y_map_offset_*map_scale_;
        transform(2, 3) = Lidar_height_ -z_map_offset_*map_scale_;

        pcl::transformPointCloud(PointcloudMapRaw_pcl_, PointcloudMapRaw_pcl_, transform);
        //---

        pcl::toROSMsg(PointcloudMapRaw_pcl_, PointcloudMap_ros_);
        PointcloudMap_ros_.header.stamp = this->get_clock()->now();
        PointcloudMap_ros_.header.frame_id = "map";

        publisher_PointcloudMap_->publish(std::move(PointcloudMap_ros_));

        is_loadPCDFile_ = true;
    }
    else{
        is_loadPCDFile_ = false;
    }
}

bool Localizer::Is_LoadPCDFile(void){
    return is_loadPCDFile_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Localizer::align(int n, boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud ) {

    transform_ros_.position.x = xOdom_;
    transform_ros_.position.y = yOdom_;
    transform_ros_.position.z = zOdom_;

    /*initial guess*/
	Eigen::Translation3f init_translation(
        (float)transform_ros_.position.x,
        (float)transform_ros_.position.y,
        (float)transform_ros_.position.z
    );

    tf2::Quaternion Quat;
    Quat.setRPY( Rollpose_, Pitchpose_, yawOdom_);

    transform_ros_.orientation.x = Quat.getX();
    transform_ros_.orientation.y = Quat.getY();
    transform_ros_.orientation.z = Quat.getZ();
    transform_ros_.orientation.w = Quat.getW();

	Eigen::AngleAxisf init_rotation(
		QuatMsgToEigen(transform_ros_.orientation)
    );

    //std::cout << "init_translation = (" << init_translation.x() << ", " << init_translation.y() << ", " << init_translation.z() << ")" << std::endl;
	//std::cout << "init_rotation : (" << init_rotation.axis()(0) << ", " << init_rotation.axis()(1) << ", " << init_rotation.axis()(2) << "), " << init_rotation.angle() << " [rad]" << std::endl;
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();

    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());

    rclcpp::Clock system_clock;

    auto t1 = system_clock.now();
    registration->align(*aligned,init_guess);

    align_fitnessScore_ = registration->getFitnessScore();

    auto t2 = system_clock.now();
    //std::cout << "single : " << n << ":" <<(t2 - t1).seconds()* 1000 << "[msec]" << std::endl;
    //std::cout << "fitness: " << n << ":" << registration->getFitnessScore() << std::endl << std::endl;
    transform_eigen_ = registration->getFinalTransformation().cast<double>();

    return aligned;
}

void Localizer::convertTransformEigenToROS(void)
{
    // // convert Matrix4d to Affine3d for ROS conversion
    // Eigen::Affine3d transform_affine_eigen;
    // transform_affine_eigen = transform_eigen_;
    // transform_ros_ = tf2::toMsg(transform_affine_eigen);
    // //transform_ros_ = tf2::toMsg(transform_eigen_);

    // //const geometry_msgs::msg::Pose pose = matrix4d_to_pose(transform_eigen_);

    pose_  = matrix4d_to_pose(transform_eigen_);

    tf2::Quaternion q(
        pose_.orientation.x,
        pose_.orientation.y,
        pose_.orientation.z,
        pose_.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tmp_Yawpose_ = yaw;
    tmp_Pitchpose_ = pitch;
    tmp_Rollpose_ = roll;

    //printf("pose_.x=%lf,pose_.y=%lf,tmp_Yawpose=%lf \n",pose_.position.x,pose_.position.y,tmp_Yawpose_*180/M_PI);
    //printf("tmp_Yawpose=%lf \n",tmp_Yawpose_*180/M_PI);
}

geometry_msgs::msg::Pose Localizer::matrix4d_to_pose(const Eigen::Matrix4d & eigen_pose_matrix)
{
  Eigen::Affine3d eigen_pose_affine;
  eigen_pose_affine.matrix() = eigen_pose_matrix.cast<double>();
  geometry_msgs::msg::Pose ros_pose = tf2::toMsg(eigen_pose_affine);
  return ros_pose;
}

void Localizer::accept_MapMatching(void){

    //printf("accept_MapMatching\n");

    yawOdom_ = tmp_Yawpose_;
    //yawOdom_ = 0.9*tmp_Yawpose_ + 0.1*yawOdom_; //LPF
    //yawAdjOdom_ = tmp_Yawpose_ - yawOdom_;

    Yawpose_ = yawOdom_;
    Yawpose_ = modAngle(Yawpose_);

    Pitchpose_  =  tmp_Pitchpose_;
    //Rollpose_   =  -tmp_Rollpose_;
    Rollpose_   =  tmp_Rollpose_;

    xOdom_ = pose_.position.x;
    yOdom_ = pose_.position.y;
    zOdom_ = pose_.position.z;

    Xpose_ = xOdom_;
    Ypose_ = yOdom_;
    Zpose_ = zOdom_;
}


void Localizer::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    double lidar_align_fitnessScore = 0;

    pcl::fromROSMsg(*cloud_msg, PointcloudRaw_pcl_);
    PointcloudRaw_pcl_.header.frame_id = "map";

    current_timeLidar_ = this->get_clock()->now().seconds();

    static double prev_time = current_timeLidar_;
    dtLidar_ = current_timeLidar_ - prev_time;

    //map offset
    // for(auto &p : PointcloudRaw_pcl_.points)
    // {
    //     //p.z += -z_map_offset_*map_scale_;
    //     p.z += Lidar_height_ -z_map_offset_*map_scale_;
    // }

    static pcl::VoxelGrid<pcl::PointXYZI> voxelSampler;
    voxelSampler.setInputCloud(PointcloudRaw_pcl_.makeShared());
    voxelSampler.setLeafSize(leafSize_, leafSize_ , leafSize_);
    voxelSampler.filter(PointcloudTarget_pcl_);

    pcl::toROSMsg(PointcloudTarget_pcl_, PointcloudTarget_ros_);

    //printf("CM:lidar_callback:: dtLidar_ = %lf \n",dtLidar_);

    if(PointcloudTarget_pcl_.size () > 0){

        //Obtain_Z_from_PointCloudMap();

        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_mapmatching = align(1,ptr_ndt_, PointcloudMap_pcl_.makeShared(), PointcloudTarget_pcl_.makeShared());
        pcl::toROSMsg(*aligned_mapmatching, PointcloudMapMatching_ros_);
        publisher_PointcloudMapMatching_->publish(std::move(PointcloudMapMatching_ros_));

        if(align_fitnessScore_ < threshold_score_align_with_map_ ){

            convertTransformEigenToROS();
            //EKF_evalSensing();

            if(dxy_mh_ < threshold_mh_dist_ ){
                accept_MapMatching();
                EKF_update();
            }
        }

        Eigen::Affine3f transformation = pcl::getTransformation(x_pre_(0), x_pre_(1), zOdom_, Rollpose_, Pitchpose_, x_pre_(2));

        pcl::transformPointCloud(PointcloudTarget_pcl_, PointcloudAlign_pcl_, transformation);
        pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_);

        publisher_AlignPointcloud_ ->publish(std::move(PointcloudAlign_ros_));

        setPose();
        broadcastTF();

        //ExtractObject1(PointcloudTarget_pcl_.makeShared(),0.1);
        ExtractObject1(PointcloudRaw_pcl_.makeShared(),0.05);

        if(!PointcloudLowObject_pcl_.empty()){
            PointcloudObject1_pcl_ += PointcloudLowObject_pcl_; //test
        }

        //PointcloudObject1_pcl_ = PointcloudLowObject_pcl_;
    
        pcl::toROSMsg(PointcloudObject1_pcl_, PointcloudObject1_ros_);
        
        PointcloudObject1_ros_.header.stamp = this->get_clock()->now();
        PointcloudObject1_ros_.header.frame_id = "cloud_matching";
        
        publisher_PointcloudObject1_->publish(std::move(PointcloudObject1_ros_));

    }


#ifdef LOG
    fprintf(f_pose_,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            current_timeOdom_ , //1
            Xpose_,//2
            Ypose_, //3
            Zpose_,//4
            align_fitnessScore_, //5
            Yawpose_,//6
            Pitchpose_*180/M_PI,//7
            vLidar_,//8
            vOdom_,//9
            dtLidar_,//10
            dtOdom_,//11
            lidar_align_fitnessScore,//12
            xOdom_,//13
            yOdom_,//14
            zOdom_,//15
            yawOdom_,//16
            ddMap_,//17
            yawMap_,//18
            pose_.position.x,//19
            pose_.position.y,//20
            tmp_Yawpose_,//21
            minZ_);//22

    fprintf(f_ekf_,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            current_timeLidar_,//1
            dtOdom_,//2
            vOdom_,//3
            wOdom_,//4
            x_pre_(0),//5
            x_pre_(1), //6
            x_pre_(2),//7
            x_est_(0),//8
            x_est_(1),//9
            x_est_(2),//10
            dxy_mh_,//11
            prePt_(0,0),
            prePt_(1,1),
            prePt_(2,2),
            ddMap_,
            dgap_,
            Sx2_,
            Sy2_,
            Sz2_,
            dyaw_);
    #endif

    prev_time = current_timeLidar_;

    if(IMU_error_count_ ++ > 100){
        IMU_FIX_Odom_Status_.data[0] = -1; //IMU
    }
    if(FIX_error_count_ ++ > 100){
        IMU_FIX_Odom_Status_.data[1] = -10; //GNSS
    }
    if(Odom_error_count_ ++ > 100){
        IMU_FIX_Odom_Status_.data[2] = -1; //Odom
    }

    if(IMU_error_count_ > 200) IMU_error_count_ = 200;
    if(FIX_error_count_ > 200) FIX_error_count_ = 200;
    if(Odom_error_count_ > 200) Odom_error_count_ = 200;
    
    publisher_IMU_FIX_Odom_status_->publish(std::move(IMU_FIX_Odom_Status_));
}

void Localizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    //printf("odom_callback \n");
    prev_poseOnOdom_ = poseOnOdom_;

    poseOnOdom_ = *msg;
    poseOnOdom_.header.frame_id = "base_link";

    //convert ROS1 bag to ROS2
    static double prev_time = -1.0;
    static double l_vOdom = 0;
    static double x = msg->pose.pose.position.x;
    static double y = msg->pose.pose.position.y;
    static double l_x = x;
    static double l_y = y;
    static double dd = 0;

    current_timeOdom_ = this->get_clock()->now().seconds();

    //printf("Odom::%lf::x=%lf,y=%lf\n",current_timeOdom_,msg->pose.pose.position.x,msg->pose.pose.position.y);
    //printf("Twist::%lf::v=%lf,w=%lf\n",current_timeOdom_,msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    dtOdom_ = current_timeOdom_ - prev_time;
    vOdom_  = msg->twist.twist.linear.x;

    if(dtOdom_ > 0)aOdom_  = (vOdom_- l_vOdom)/dtOdom_;
    else aOdom_ = 0;

    wOdom_  = msg->twist.twist.angular.z;

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    dd = sqrt((x-l_x)*(x-l_x) + (y-l_y)*(y-l_y));

    dxOdom_ = x-l_x;
    dyOdom_ = y-l_y;
    ddOdom_ = dd;

#ifdef LOG
    fprintf(f_odom_,"%lf %lf %lf %lf %lf %lf\n",
        current_timeOdom_,
        poseOnOdom_.pose.pose.position.x,
        poseOnOdom_.pose.pose.position.y,
        msg->twist.twist.linear.x,
        wOdom_,
        wLidar_);
#endif
    tf2::Quaternion q(
        poseOnOdom_.pose.pose.orientation.x,
        poseOnOdom_.pose.pose.orientation.y,
        poseOnOdom_.pose.pose.orientation.z,
        poseOnOdom_.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //yaw += M_PI;    
    static double l_yaw = yaw;

    //qOdom_ = yaw;
    yawOdom_ += -(yaw - l_yaw);
    
    xOdom_ += dd*cos(Pitchpose_)*cos(yawOdom_);
    yOdom_ += dd*cos(Pitchpose_)*sin(yawOdom_);
    //zOdom_ = Zpose_;
    l_vOdom = vOdom_;
    prev_time = current_timeOdom_;

    l_x=x;l_y=y; l_yaw = yaw;

    EKF_predict();

    Odom_error_count_ = 0;
    IMU_FIX_Odom_Status_.data[2] = 1;
    //printf("Odom:x=%lf,y=%lf,yaw=%lf\n",xOdom_,yOdom_,yawOdom_*180/M_PI);
}

void Localizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);


    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //double current_timeImu = (double)(msg->header.stamp.sec + RCL_NS_TO_MS(msg->header.stamp.nanosec)*0.001);
    double current_timeImu  = this->get_clock()->now().seconds();

    static double prev_time = current_timeImu;
    double dt = current_timeImu - prev_time;

    YawImu_     = -yaw;
    //PitchImu_   =  -pitch;
    PitchImu_   +=  msg->angular_velocity.x*dt;
    RollImu_    +=  msg->angular_velocity.y*dt;

    IMU_FIX_Odom_Status_.data[0] = 1;

    IMU_error_count_ = 0;
}

void Localizer::fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){

    double ts = this->get_clock()->now().seconds();

    static bool flag_init = true;
    static bool flag_first = true;
    double xfix = 0;
    double yfix = 0;
    double zfix = 0;

    if(flag_init){
        pcgpc_->set_plane(InitLatitude_,InitLongitude_); //Tokyo

        //printf("msg: init status:%d::lat:%lf,lon:%lf \n",msg->status.status,msg->latitude,msg->longitude);
        //pcgpc_->set_plane(9); //Tokyo
            
        //flag_init = false;
    }

    pcgpc_->llh_to_xyz(msg->latitude,msg->longitude,msg->altitude);

    xfix = -pcgpc_->y();
    yfix = -pcgpc_->x();
    zfix = pcgpc_->z();

    Xfix_ = xfix;
    Yfix_ = yfix;
    Zfix_ = zfix;

    // if(flag_init){
    //     pose_map_.translation().x() = msg->pose.pose.position.x();
    //     pose_map_.translation().y() = msg->pose.pose.position.y();
    //     pose_map_.translation().z() = msg->pose.pose.position.z();

    //     Eigen::Quaterniond q(
    //         msg->pose.pose.orientation.w,
    //         msg->pose.pose.orientation.x,
    //         msg->pose.pose.orientation.y,
    //         msg->pose.pose.orientation.z);

    //     pose_map_.linear() = q.toRotationMatrix();
    // }

    //fixStatus_ = msg->status.status;

    double cov1 = msg->position_covariance[0];
    double cov2 = msg->position_covariance[4];
    double cov3 = msg->position_covariance[8];

    if(msg->status.status>=2){
        fixStatus_ = 2;
    }
    else if(cov1 < 0.1 || cov2 < 0.1){
        fixStatus_ = 1;
    }
    else{
        fixStatus_ = 0;
    }

    //printf("fixstatus:%d::status:%d::cov1::%lf::cov2::%lf::lat:%lf,lon:%lf \n",fixStatus_,msg->status.status,cov1,cov2,msg->latitude,msg->longitude);

    fprintf(f_fix_,"%lf %d %lf %lf %lf %lf %lf %lf\n",
                    ts,//1
                    msg->status.status,//2
                    msg->latitude,//3
                    msg->longitude,//4
                    msg->altitude,//5
                    xfix,//6
                    yfix,//7
                    zfix);//8

    if(fixStatus_ >=1){
      fprintf(f_fix2_,"%lf %d %lf %lf %lf %lf %lf %lf\n",
                    ts,//1
                    msg->status.status,//2
                    msg->latitude,//3
                    msg->longitude,//4
                    msg->altitude,//5
                    xfix,//6
                    yfix,//7
                    zfix);//8

        if(fixStatus_ >=2){
            if(flag_first){
                Zfix0_ = Zfix_;
                flag_first = false;
            }
            //zOdom_ = Zfix_- Zfix0_;
        }
        //EKF_update();
    }

    IMU_FIX_Odom_Status_.data[1] = msg->status.status;
    FIX_error_count_ = 0;

    flag_init = false;

}

void Localizer::initPose_callback(const geometry_msgs::msg::Pose::SharedPtr msg){

    printf("initPose_callback \n");

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(fixStatus_ >=2){
        xOdom_ = Xfix_;
        yOdom_ = Yfix_;
    }
    else{
        xOdom_ = Xpose_ = msg->position.x;
        yOdom_ = Ypose_ = msg->position.y;
    }
    
    yawOdom_ = Yawpose_ = yaw;

    EKF_init();
    
    printf("Localizer:: %d::InitPose=%lf,%lf,%lf\n",Xpose_,Ypose_,Yawpose_*180/M_PI);
    printf("Localizer:: fixStatus_ = %d InitOdom=%lf,%lf,%lf\n",fixStatus_,xOdom_,yOdom_,yawOdom_*180/M_PI);
    
}

void Localizer::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    static int cnt=0;
    printf("Localizer:: initialpose_callback:%d\n",cnt++);

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
   
    xOdom_ = Xpose_ = msg->pose.pose.position.x;
    yOdom_ = Ypose_ = msg->pose.pose.position.y;
    yawOdom_ = Yawpose_ = yaw;

    EKF_init();
    printf("Localizer:: InitialPose=%lf,%lf,%lf\n",Xpose_,Ypose_,Yawpose_*180/M_PI);
}


void Localizer::broadcastTF(void) {

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    //t.header.stamp = cloud_msg->header.stamp;
    //t.header.frame_id   = "/map";
    //t.child_frame_id    = "/base_link";

    t.header.frame_id   = "/map";
    t.child_frame_id    = "/cloud_matching";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0

    // t.transform.translation.x = poseOnAlign_.pose.pose.position.x;
    // t.transform.translation.y = poseOnAlign_.pose.pose.position.y;
    // t.transform.translation.z = poseOnAlign_.pose.pose.position.z;

    t.transform.translation.x = poseErrorCov_.pose.pose.position.x;
    t.transform.translation.y = poseErrorCov_.pose.pose.position.y;
    t.transform.translation.z = poseErrorCov_.pose.pose.position.z;


    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message

    // t.transform.rotation.x = poseOnAlign_.pose.pose.orientation.x;
    // t.transform.rotation.y = poseOnAlign_.pose.pose.orientation.y;
    // t.transform.rotation.z = poseOnAlign_.pose.pose.orientation.z;
    // t.transform.rotation.w = poseOnAlign_.pose.pose.orientation.w;

    t.transform.rotation.x = poseErrorCov_.pose.pose.orientation.x;
    t.transform.rotation.y = poseErrorCov_.pose.pose.orientation.y;
    t.transform.rotation.z = poseErrorCov_.pose.pose.orientation.z;
    t.transform.rotation.w = poseErrorCov_.pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
}

void Localizer::broadcastTF_Odom(void) {

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    //t.header.stamp = cloud_msg->header.stamp;
    
    /*
    t.header.frame_id   = "/map";
    t.child_frame_id    = "/base_link";
    */
    
    t.header.frame_id   = "/map";
    t.child_frame_id    = "/cloud_matching";
    
    
    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0

    // t.transform.translation.x = poseOnAlign_.pose.pose.position.x;
    // t.transform.translation.y = poseOnAlign_.pose.pose.position.y;
    // t.transform.translation.z = poseOnAlign_.pose.pose.position.z;

    t.transform.translation.x = xOdom_;
    t.transform.translation.y = yOdom_;
    t.transform.translation.z = poseErrorCov_.pose.pose.position.z;


    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message

    // t.transform.rotation.x = poseOnAlign_.pose.pose.orientation.x;
    // t.transform.rotation.y = poseOnAlign_.pose.pose.orientation.y;
    // t.transform.rotation.z = poseOnAlign_.pose.pose.orientation.z;
    // t.transform.rotation.w = poseOnAlign_.pose.pose.orientation.w;

    tf2::Quaternion Quat;
    Quat.setRPY( Rollpose_, Pitchpose_, yawOdom_);

    t.transform.rotation.x = Quat.getX();
    t.transform.rotation.y = Quat.getY();
    t.transform.rotation.z = Quat.getZ();
    t.transform.rotation.w = Quat.getW();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
}


Eigen::Quaternionf Localizer::QuatMsgToEigen(geometry_msgs::msg::Quaternion q_msg){
    Eigen::Quaternionf q_eigen;

	q_eigen.x() = (float)q_msg.x;
    q_eigen.y() = (float)q_msg.y;
    q_eigen.z() = (float)q_msg.z;
    q_eigen.w() = (float)q_msg.w;

    q_eigen.normalize();
	return q_eigen;
}

void Localizer::EKF_init(void){

    prePt_  = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列
    postPt_ = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列
    It_ = Eigen::Matrix3d::Identity(3, 3);      //誤差共分散行列

    Qt_ <<  EKF_Cov_SystemNoise_SXX_ , 0.0 , 0.0,
            0.0 , EKF_Cov_SystemNoise_SYY_ , 0.0,
            0.0 , 0.0 , EKF_Cov_SystemNoise_SYawYaw_ ; //Covariance Array of System noize

    Rt_ <<  EKF_Cov_SensorNoise_X_,0.0 ,
            0.0 ,EKF_Cov_SensorNoise_Y_; //Covariance Array of Sensor noize

    Ht_ <<  1.0, 0.0, 0.0,
            0.0, 1.0, 0.0;//Jacobian

    wNoize_ << EKF_Wnoise_X_,EKF_Wnoise_Y_,EKF_Wnoise_Yaw_; //System Noize
    vNoize_ << EKF_Vnoise_X_,EKF_Vnoise_Y_,EKF_Vnoise_Yaw_; //Sensor Noize


    x_pre_(0) = xOdom_;
    x_pre_(1) = yOdom_;
    x_pre_(2) = yawOdom_;

    x_est_ << 0,0,0;
    //n_EKF_eval_ = 10;
}

void Localizer::EKF_init(double x_init, double y_init, double yaw_init){

    prePt_  = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列
    postPt_ = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列
    It_ = Eigen::Matrix3d::Identity(3, 3);      //誤差共分散行列

    Qt_ <<  EKF_Cov_SystemNoise_SXX_ , 0.0 , 0.0,
            0.0 , EKF_Cov_SystemNoise_SYY_ , 0.0,
            0.0 , 0.0 , EKF_Cov_SystemNoise_SYawYaw_ ; //Covariance Array of System noize

    Rt_ <<  EKF_Cov_SensorNoise_X_,0.0 ,
            0.0 ,EKF_Cov_SensorNoise_Y_; //Covariance Array of Sensor noize

    Ht_ <<  1.0, 0.0, 0.0,
            0.0, 1.0, 0.0;//Jacobian

    wNoize_ << EKF_Wnoise_X_,EKF_Wnoise_Y_,EKF_Wnoise_Yaw_; //System Noize
    vNoize_ << EKF_Vnoise_X_,EKF_Vnoise_Y_,EKF_Vnoise_Yaw_; //Sensor Noize

    //x_pre_(0) = xOdom_;
    //x_pre_(1) = yOdom_;
    //x_pre_(2) = yawOdom_;
    //x_est_ << 0,0,0;

    x_pre_(0) = x_init;
    x_pre_(1) = y_init;
    x_pre_(2) = yaw_init;

    x_est_ << 0,0,0;

    //n_EKF_eval_ = 10;
}



void Localizer::EKF_predict(void){

    //double v = vLidar_;
    double v = vOdom_;
    //double dt = dtLidar_;
    double dt = dtOdom_;

    //printf("vLidar_=%lf \n",vLidar_);
    if(dt <=0 || dt > 1.0) return;
    //x_pre_(2) += wOdom_* dt;
    x_pre_(2) = yawOdom_;
    //x_pre_(2) = 0.01*x_pre_(2) + 0.99*yawOdom_;

    //aOdom_
    double q_tmp = x_pre_(2);//+ wOdom_* dtOdom_;
    
    
    double dd = v*dt ;//+ 0.5*aOdom_*dt*dt;

    x_pre_(0) += dd*cos(q_tmp);
    x_pre_(1) += dd*sin(q_tmp);

    // Jacobian
    // Ft_ <<  1.0,0.0,-v*dt*sin(q_tmp),
    //         0.0,1.0,v*dt*cos(q_tmp),
    //         0.0,0.0, 1.0;

    Ft_ <<  1.0,0.0,-dd*sin(q_tmp),
            0.0,1.0,dd*cos(q_tmp),
            0.0,0.0, 1.0;


    //予測誤差共分散行列
    prePt_ = Ft_*prePt_*Ft_.transpose() + Qt_;

    // std::cout << "pre Pt_:"<< std::endl;
    // std::cout << prePt_ << std::endl;

    double dx = Xpose_- x_pre_(0);
    double dy = Ypose_- x_pre_(1);

    dgap_ = sqrt(dx*dx + dy*dy);
    double vOdom_abs = fabs(vOdom_);
}

void Localizer::EKF_update(void){

    x_z_ << Xpose_, Ypose_;

    //Kalman Gain
    St_ = Ht_*prePt_*Ht_.transpose() + Rt_;

    Kt_ = (prePt_*Ht_.transpose())*St_.inverse();

    //推定値
    x_est_ = x_pre_ + Kt_*(x_z_ - Ht_*x_pre_);

    //推定誤差共分散行列
    postPt_ = ( It_ - Kt_*Ht_)*prePt_;

    // std::cout << "Kt_:"<< std::endl;
    // std::cout << Kt_ << std::endl;

    // std::cout << "post Pt_:"<< std::endl;
    // std::cout << postPt_ << std::endl;

    prePt_ = postPt_;

    x_pre_(0) = x_est_(0);
    x_pre_(1) = x_est_(1);
    x_pre_(2) = x_est_(2);

 }
#if 1
 int Localizer::EKF_evalSensing(void){

    //評価値: Mahalanobis Distance
    static point3D prev_pose = {0,0,0};

    point3D ave ={0,0,0};
    Vector3d x_ave;
    double sx2=0,sy2=0,sz2=0;
    Eigen::Vector3d x_Z_tmp;
    Eigen::Vector3d x_Zxyz_pre;

    static std::deque <point3D> dq_pose;

    if(dq_pose.size() < n_EKF_eval_) {
        dxy_mh_ = 0;
    }
    else{
        int nn = dq_pose.size();
        //AVERAGE
        for(int k=0;k < nn;k++){
            ave.x += dq_pose[k].x;
            ave.y += dq_pose[k].y;
            ave.z += dq_pose[k].z;
        }
        ave.x /= nn;
        ave.y /= nn;
        ave.z /= nn;

        //Cvariance
        for(int k=0;k < nn;k++){
            sx2 += (dq_pose[k].x - ave.x)*(dq_pose[k].x - ave.x);
            sy2 += (dq_pose[k].y - ave.y)*(dq_pose[k].y - ave.y);
            sz2 += (dq_pose[k].z - ave.z)*(dq_pose[k].z - ave.z);
        }
        sx2 /= nn; sy2 /= nn; sz2 /= nn;

        Sxyzt_ <<   sx2,0,0,
                    0,sy2,0,
                    0,0,sz2;

        x_ave << ave.x, ave.y,ave.z;

        x_Z_tmp << pose_.position.x, pose_.position.y, pose_.position.z;

        x_Zxyz_pre = x_Z_tmp - x_ave;

        dxy_mh_ = sqrt(x_Zxyz_pre.transpose()*Sxyzt_.inverse()*x_Zxyz_pre);

        Sx2_ = sx2;
        Sy2_ = sy2;
        Sz2_ = sz2;
    }

    point3D cur_pose = {pose_.position.x, pose_.position.y, pose_.position.z};

    if(prev_pose.x != cur_pose.x || prev_pose.y != cur_pose.y|| prev_pose.z != cur_pose.z){
        dq_pose.push_back(cur_pose);
        if(dq_pose.size() > n_EKF_eval_)dq_pose.pop_front();
        flag_AlignedWithMap_ = true;
    }

    prev_pose.x = cur_pose.x;
    prev_pose.y = cur_pose.y;
    prev_pose.z = cur_pose.z;

    return 1;
 }
#endif

void Localizer::setPose(void){

    poseErrorCov_.header.stamp = this->get_clock()->now();
    poseErrorCov_.header.frame_id = "map";

    //poseErrorCov_.pose.pose.position.x = x_pre_(0);
    //poseErrorCov_.pose.pose.position.y = x_pre_(1);

    //poseErrorCov_.pose.pose.position.x = Xpose_;
    //poseErrorCov_.pose.pose.position.y = Ypose_;
    //poseErrorCov_.pose.pose.position.z = Zpose_;

    // poseErrorCov_.pose.pose.position.x = xOdom_;
    // poseErrorCov_.pose.pose.position.y = yOdom_;
    // poseErrorCov_.pose.pose.position.z = zOdom_;
    
    poseErrorCov_.pose.pose.position.x = x_pre_(0);
    poseErrorCov_.pose.pose.position.y = x_pre_(1);
    poseErrorCov_.pose.pose.position.z = zOdom_;

    tf2::Quaternion Quat;
    //Quat.setRPY( Rollpose_, Pitchpose_, yawOdom_);
    Quat.setRPY( Rollpose_, Pitchpose_, x_pre_(2));
    
    //Quat.setRPY( Rollpose_, Pitchpose_, Yawpose_);
    //yawOdom_= Yawpose_;
    //Quat.setRPY( Rollpose_, Pitchpose_, x_pre_(2));

    poseErrorCov_.pose.pose.orientation.x = Quat.getX();
    poseErrorCov_.pose.pose.orientation.y = Quat.getY();
    poseErrorCov_.pose.pose.orientation.z = Quat.getZ();
    poseErrorCov_.pose.pose.orientation.w = Quat.getW();

    poseErrorCov_.pose.covariance.at(0) = prePt_(0,0);
    poseErrorCov_.pose.covariance.at(7) = prePt_(1,1);
    poseErrorCov_.pose.covariance.at(35) = prePt_(2,2);

    publisher_PoseErrorCov_->publish(std::move(poseErrorCov_));

}

void Localizer::ExtractObject1(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size){

    //Extract Object
    static pcl::VoxelGrid<pcl::PointXYZI> voxelSamplerObject;
    voxelSamplerObject.setInputCloud(Pointcloud_input->makeShared());
    voxelSamplerObject.setLeafSize(leaf_size, leaf_size, leaf_size);
    //voxelSamplerObject.setLeafSize(0.05, 0.05, 0.05);

    voxelSamplerObject.setFilterFieldName("z");
  
    voxelSamplerObject.setFilterLimits(-0.75, 0.5);
    //voxelSamplerObject.setFilterLimits(-0.56, 0.5);
    voxelSamplerObject.filter(PointcloudObject1_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudObject1_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("x");
    //voxelSamplerObject.setFilterLimits(-2.0, 8.0);
    voxelSamplerObject.setFilterLimits(-2.0, 30.0); //251111
    voxelSamplerObject.filter(PointcloudObject1_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudObject1_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("y");
    //voxelSamplerObject.setFilterLimits(-2.5, 2.5);
    voxelSamplerObject.setFilterLimits(-5.0, 5.0);
    //voxelSamplerObject.setFilterLimits(-10.0, 10.0);
    voxelSamplerObject.filter(PointcloudObject1_pcl_);

}
#if 0
void Localizer::ExtractObject2(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size){

    //Extract Object
    static pcl::VoxelGrid<pcl::PointXYZI> voxelSamplerObject;
    voxelSamplerObject.setInputCloud(Pointcloud_input->makeShared());
    voxelSamplerObject.setLeafSize(leaf_size, leaf_size, leaf_size);

    voxelSamplerObject.setFilterFieldName("z");
    voxelSamplerObject.setFilterLimits(-0.7, 0.5);
    voxelSamplerObject.filter(PointcloudObject2_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudObject2_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("x");
    voxelSamplerObject.setFilterLimits(-2.0, 8.0);
    voxelSamplerObject.filter(PointcloudObject2_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudObject2_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("y");
    voxelSamplerObject.setFilterLimits(-2.5, 2.5);
    voxelSamplerObject.filter(PointcloudObject2_pcl_);

    pcl::toROSMsg(PointcloudObject2_pcl_, PointcloudObject2_ros_);
    //pcl::toROSMsg(PointcloudObject_pcl_, PointcloudTarget_ros_);
    //publisher_PointcloudObject_->publish(std::move(PointcloudObject_ros_));
}

//Eigen::Affine3f transform
Eigen::Affine3f Localizer::Make_transform_pointcloud(Pose3D pose){

    Eigen::Vector4d q = RotMtoQuat(pose.Rot);
        
    Eigen::Quaternionf quot;
    quot.x() = q[0];quot.y() = q[1];quot.z() = q[2];quot.w() = q[3];        
    Eigen::Vector3f translation(pose.X[0], pose.X[1], pose.X[2]);

    // Affine3f変換を使って回転＋並進を合成
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = quot.toRotationMatrix();   // 回転を設定
    transform.translation() = translation;       // 並進を設定
        
    return transform;  
}

int Localizer::LowObjectPointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // 時刻
    time_LowObject_ = this->get_clock()->now().seconds();

    static double prev_time = time_LowObject_;

    //==================================================
    // 1. PointCloud2の空チェック
    //==================================================
    if (cloud_msg == nullptr)
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "LowObject: cloud_msg is nullptr. Skip."
        );

        return 0;
    }

    if (cloud_msg->width == 0 ||
        cloud_msg->height == 0 ||
        cloud_msg->data.empty())
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "LowObject: Empty PointCloud2. Skip."
        );

        prev_time = time_LowObject_;

        return 0;
    }

    //==================================================
    // 2. PointCloud2 → PointXYZ
    //==================================================
    pcl::PointCloud<pcl::PointXYZ> Pointcloud_tmp_pcl;

    pcl::fromROSMsg(
        *cloud_msg,
        Pointcloud_tmp_pcl
    );

    // 変換後の空チェック
    if (Pointcloud_tmp_pcl.empty())
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "LowObject: Empty PCL PointCloud after conversion. Skip."
        );

        prev_time = time_LowObject_;

        return 0;
    }

    //==================================================
    // 3. PointXYZ → PointXYZI
    //==================================================
    PointcloudLowObject_pcl_.clear();

    PointcloudLowObject_pcl_.reserve(
        Pointcloud_tmp_pcl.size()
    );

    for (const auto &p : Pointcloud_tmp_pcl.points)
    {
        // NaN除去
        if (!pcl::isFinite(p))
            continue;

        pcl::PointXYZI point;

        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        // RealSenseにはintensityがない
        point.intensity = 0.0f;

        PointcloudLowObject_pcl_.push_back(point);
    }

    //==================================================
    // 4. Frame ID
    //==================================================
    PointcloudLowObject_pcl_.header.frame_id =
        cloud_msg->header.frame_id;

    //==================================================
    // 5. 有効点がない場合
    //==================================================
    if (PointcloudLowObject_pcl_.empty())
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "LowObject: No valid points after NaN filtering. Skip."
        );

        prev_time = time_LowObject_;

        return 0;
    }

    // 時刻更新
    prev_time = time_LowObject_;

    return 1;
}
#endif


int Localizer::LowObjectPointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

    //printf("LC: cb_LowObjectPointcloud\n");
    PointcloudLowObject_pcl_.header.frame_id = "/cloud_matching";
    //PointcloudLowObject_pcl_.header.frame_id = "/map";
    //PointcloudLowObject_pcl_.header.frame_id = "/base_link";

    // 空フレームは処理せず抜ける
    if (cloud_msg->width * cloud_msg->height == 0) {
        PointcloudLowObject_pcl_.clear();
        return 1;
    }

    time_LowObject_  = this->get_clock()->now().seconds();
    
    static double prev_time = time_LowObject_;

    pcl::PointCloud<pcl::PointXYZ> Pointcloud_tmp_pcl;

    pcl::fromROSMsg(*cloud_msg, Pointcloud_tmp_pcl); 
    //PointcloudLowObject_pcl_.header.frame_id = "/cloud_matching";
    
    PointcloudLowObject_pcl_.clear();
    PointcloudLowObject_pcl_.reserve(Pointcloud_tmp_pcl.size());

    for (const auto &p : Pointcloud_tmp_pcl.points){
    
        // NaN除去
        if (!pcl::isFinite(p)) continue;

        pcl::PointXYZI point;

        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        // RealSenseにはintensityがない
        point.intensity = 0.0f;

        PointcloudLowObject_pcl_.push_back(point);
    }

    //cout << "PointcloudLowObject_pcl_.size()=" << PointcloudLowObject_pcl_.size() << std::endl;
    
    prev_time = time_LowObject_;

    return 1;
}

# if 0
int Localizer::LowObjectPointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

    //printf("LC: cb_LowObjectPointcloud\n");

    time_LowObject_  = this->get_clock()->now().seconds();
    
    static double prev_time = time_LowObject_;
    
    pcl::PointCloud<pcl::PointXYZ> Pointcloud_tmp_pcl;

    
    pcl::fromROSMsg(*cloud_msg, Pointcloud_tmp_pcl); 

    //cout << "LC: Pointcloud_tmp_pcl.size()=" << Pointcloud_tmp_pcl.size() << std::endl;

    PointcloudLowObject_pcl_.header.frame_id = "/cloud_matching";
    
    //==================================================
    // 3. PointXYZ → PointXYZI
    //==================================================
    PointcloudLowObject_pcl_.clear();
    PointcloudLowObject_pcl_.reserve(Pointcloud_tmp_pcl.size());

    for (const auto &p : Pointcloud_tmp_pcl.points){
    
        // NaN除去
        if (!pcl::isFinite(p)) continue;

        pcl::PointXYZI point;

        point.x = p.x;
        point.y = p.y;
        point.z = p.z;

        // RealSenseにはintensityがない
        point.intensity = 0.0f;

        PointcloudLowObject_pcl_.push_back(point);
    }

    //cout << "PointcloudLowObject_pcl_.size()=" << PointcloudLowObject_pcl_.size() << std::endl;
    
    prev_time = time_LowObject_;

    return 1;
}
#endif

int main(int argc, char * argv[])
{
    // クライアントライブラリの初期化
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    /*
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localizer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    */

    //Autoware
    /*
    rclcpp::init(argc, argv);
    auto cloud_matcher = std::make_shared<Localizer>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(cloud_matcher);
    exec.spin();
    rclcpp::shutdown();
    */

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto cloud_matcher = std::make_shared<Localizer>();
    exec.add_node(cloud_matcher);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}
