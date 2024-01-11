#include <iostream>
#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头

#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <thread>
#include "voxel.h"
#include "record_data.h"


using namespace std;


int main(int argc, char** argv) {
    ros::init(argc,argv,"map_update_node");
    cout<<"hello ws"<<endl;
    Record recorder;
    MapUpdate mu;


    cout<<RED<<"record started"<<RESET<<endl;
    recorder.start(); 

    while(1) //持续检测record_ok情况,while会占用过多资源导致订阅失败
    {
        if(recorder.record_ok)
        {
            cout<<RED<<"update started"<<RESET<<endl;
            mu.generateVmap();
            mu.updateChanges();
            mu.restoreMmap();
            break; //执行一次后退出
        }
        else
        {
            ros::spinOnce(); //允许进入回调函数，由于一直有publisher，因此相当于进入回调函数，即保存pcd和tf
            //cout<<RED<<"not started yet"<<RESET<<endl;
        }
    }


    return 0;
}
