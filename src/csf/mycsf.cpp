#include"include/mycsf.h"


ClothSimulationFilter::ClothSimulationFilter(ros::NodeHandle& nh) {
    sub_ = nh.subscribe("/livox/lidar", 1, &ClothSimulationFilter::scanCallback, this);
    ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 10);
    obstacles_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_points", 10);
    initializeConfig();
}

ClothSimulationFilter::~ClothSimulationFilter() {}

 void ClothSimulationFilter::initializeConfig() 
{
    std::string slop_smooth, class_threshold, cloth_resolution, iterations, rigidness, time_step, terr_pointClouds_filepath;

    cfg.readConfigFile(CONFIG_FILE_PATH, "slop_smooth", slop_smooth);
    bool ss = slop_smooth == "true" || slop_smooth == "True";

    cfg.readConfigFile(CONFIG_FILE_PATH, "class_threshold", class_threshold);
    cfg.readConfigFile(CONFIG_FILE_PATH, "cloth_resolution", cloth_resolution);
    cfg.readConfigFile(CONFIG_FILE_PATH, "iterations", iterations);
    cfg.readConfigFile(CONFIG_FILE_PATH, "rigidness", rigidness);
    cfg.readConfigFile(CONFIG_FILE_PATH, "time_step", time_step);
    cfg.readConfigFile(CONFIG_FILE_PATH, "terr_pointClouds_filepath", terr_pointClouds_filepath);

    // 使用读取的参数更新csf实例
    csf.params.bSloopSmooth = ss;
    csf.params.class_threshold = atof(class_threshold.c_str());
    csf.params.cloth_resolution = atof(cloth_resolution.c_str());
    csf.params.interations = atoi(iterations.c_str());
    csf.params.rigidness = atoi(rigidness.c_str());
    csf.params.time_step = atof(time_step.c_str());
}

void ClothSimulationFilter::scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->data.empty()) {
        ROS_ERROR("Received empty point cloud message.");
        return;
    }

    auto start = std::chrono::steady_clock::now();

    // 转换ROS点云到PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    csf.setLivoxPointcloud(cloud);

    std::vector<int> groundIndexes, offGroundIndexes;
    csf.do_filtering(groundIndexes, offGroundIndexes, false);

    // 创建地面点云和障碍物点云的子集
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
    pcl::PointIndices::Ptr off_ground_indices(new pcl::PointIndices());

    // 填充地面和非地面点索引
    for (int index : groundIndexes) {
        ground_indices->indices.push_back(index);
    }
    for (int index : offGroundIndexes) {
        off_ground_indices->indices.push_back(index);
    }

    // 提取地面点云
    extract.setInputCloud(cloud);   
    extract.setIndices(ground_indices);
    extract.filter(*ground_cloud);

    // 提取非地面点云
    extract.setIndices(off_ground_indices);
    extract.filter(*obstacles_cloud);

    // 转换PCL点云回ROS消息
    sensor_msgs::PointCloud2 ground_msg;
    sensor_msgs::PointCloud2 obstacles_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    pcl::toROSMsg(*obstacles_cloud, obstacles_msg);

    ground_msg.header = msg->header;
    obstacles_msg.header = msg->header;

    // 发布地面点云和障碍物点云
    ground_pub_.publish(ground_msg);
    obstacles_pub_.publish(obstacles_msg);


    // 记录结束时间
    auto end = std::chrono::steady_clock::now();
    // 计算持续时间（以毫秒为单位）
    std::chrono::duration<double, std::milli> duration_ms = end - start;

    std::cout<<"seg ground: "<<duration_ms.count()<<"ms"<<std::endl;
}