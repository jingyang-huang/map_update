#include <iostream>
#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头

#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>

#include "voxel.h"
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_update_node");
    cout<<"hello ws"<<endl;
    
    MapUpdate mu;

    mu.generateVmap();
    mu.updateChanges();
    mu.restoreMmap();

    ros::spin();
    return 0;
}
