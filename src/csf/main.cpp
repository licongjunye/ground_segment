#include"include/mycsf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csf_node");
    ros::NodeHandle nh;

    ClothSimulationFilter csf(nh);
    
    ros::spin();

    return 0;
}
