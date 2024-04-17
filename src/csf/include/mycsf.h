#ifndef MYCSF_H
#define MYCSF_H

#include<ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "../src/CSF.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <locale.h>
#include <time.h>
#include <cstdlib>
#include <cstring>
#include "../src/Cfg.h"
#include <chrono>

//需要改成用户的具体路径
#define CONFIG_FILE_PATH "/home/hlc/code/ground_segment_ws/src/csf/include/params.cfg"

class ClothSimulationFilter {
public:

    ClothSimulationFilter(ros::NodeHandle& nh);
    ~ClothSimulationFilter();

    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg);  // Declaration

private:
    Cfg cfg;
    CSF csf;
    
    ros::Subscriber sub_;
    ros::Publisher ground_pub_;
    ros::Publisher obstacles_pub_;
    void initializeConfig();

};


#endif