#pragma once

#include <iostream>
#include <memory>
#include <math.h>
#include <filesystem>
#include <fstream>　// c++文件操作
#include <iomanip>　// 设置输出格式
#include <dirent.h> 
#include <Eigen/Core>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

class Record
{
    private:

    public:
    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Subscriber subLaserCloud;
    ofstream outfile;
    std::string Scantopic;
    std::string WorkspaceDir;
    std::string RecordPrefix;
    const std::string SaveFilename = "tf.txt";
    bool record_ok = 0;
    
    Record()
    {

        nh.param<std::string>("RecordPrefix", RecordPrefix, "");
        nh.param<std::string>("record/Scantopic", Scantopic, "/velodyne_points");
        cout<< GREEN<<"record params "<<RESET<<endl;
        cout<< "RecordPrefix "<<RecordPrefix<<endl;

    }

    ~Record()
    {
        outfile.close();
    }
    void mkdirIfnotexist(std::string DirPrefix)
    {
        DIR *dir; 
        if ((dir=opendir(DirPrefix.c_str())) == NULL)   //判断路径或文件夹是否存在,不在需要创建
        {   
            std::string mkdir_order = "mkdir -p " + DirPrefix;
            system( mkdir_order.c_str() );
        }
        else
        {
            //TODO：现在的删除会导致系统出问题
            std::string rm_order = "rm -rf " + DirPrefix;
            std::string mkdir_order = "mkdir -p " + DirPrefix;
            system( rm_order.c_str() ); //先移除再创建，达到清空的目的
            system( mkdir_order.c_str() );
            
        }  
    }
    void start()
    {
        mkdirIfnotexist(RecordPrefix);

        // DIR *dir;   
        // if ((dir=opendir(RecordPrefix.c_str())) == NULL)   //判断路径或文件夹是否存在,不在需要创建
        // {   
        //     std::string mkdir_order = "mkdir -p " + RecordPrefix;
        //     system( mkdir_order.c_str() );
        // }
        // else
        // {
        //     //TODO：现在的删除会导致系统出问题
        //     std::string rm_order = "rm -rf " + RecordPrefix;
        //     std::string mkdir_order = "mkdir -p " + RecordPrefix;
        //     system( rm_order.c_str() ); //先移除再创建，达到清空的目的
        //     system( mkdir_order.c_str() );
            
        // }  

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(Scantopic, 5, &Record::topicCallback,this);
        outfile.open(RecordPrefix + SaveFilename , ios::trunc| ios::in | ios::out); //TODO:后期可以u换成binary
        //outfile << std::fixed;
    }
    void topicCallback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        static int i = 0;
        if(subLaserCloud.getNumPublishers() != 0) //有publishers才存储
        {
            //TODO:加入cacheu缓存操作
            //std::deque<sensor_msgs::PointCloud2> cloudQueue;
            pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloudIn (new pcl::PointCloud<pcl::PointXYZI>);//仅对于velodyne
            pcl::fromROSMsg(*laserCloudMsg, *rawCloudIn);
            //TODO: 改成输入参数字符串，放在.launch里  //TODO2:根据header命名.pcd

            //打印输出存储的点云数据
            //std::cerr << "Saved " << rawCloudIn->points.size() << " data points to "<< pcdname << std::endl;

            //record();
            //TODO 订阅velodyne 到 map 的 tf，确定是这个吗，有可能是反过来的  --确定是这个
            float saved_x = 0.0;
            float saved_y = 0.0;
            float saved_z = 0.0;
            float theta = 0.0;
            tf::StampedTransform transform;
            Eigen::Matrix4f eigen_transform;
            // Eigen::VectorXcf temp_vec;
            
            //TODO:获取一致的时间戳
            try
            {
                // base_link to map
                listener.waitForTransform("/map",
                                        "/velodyne",
                                        ros::Time::now(), ros::Duration(0.1)); //这句必须要, 等到可用来了才进行变换
                //改成laserCloudMsg->header.stamp也可以，不过有些错误
                //改成ros::Time::now()了之后几乎没有错误了，似乎建图效果也好点
                //ros::Time(0)似乎更科学，但是会出错
                listener.lookupTransform("/map",
                                        "/velodyne",
                                        laserCloudMsg->header.stamp, transform);

                //pcl_ros::transformPointCloud(eigen_transform, *in, *out);
                
            }
            catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                return;
            }
            catch (tf::LookupException &e)
            {
                ROS_ERROR ("%s", e.what ());
                return ;
            }
            catch (tf::ExtrapolationException &e)
            {
                ROS_ERROR ("%s", e.what ());
                return ;
            }
            
            std::string pcdname = std::to_string(i+1)+ ".pcd";
            i++;
            pcl_ros::transformAsMatrix(transform, eigen_transform);
            //转成vec方便写入
            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> temp_mat(eigen_transform);
            Eigen::Map<Eigen::RowVectorXf> temp_vec(temp_mat.data(), temp_mat.size());
                
            outfile<<temp_vec<< endl; //保存到.txt
            pcl::io::savePCDFileASCII(RecordPrefix+ pcdname , *rawCloudIn); 
            std::cerr << "Saved pcd to "<< pcdname << std::endl;
        }
        else //没有就不存储
        {
            record_ok = 1;
            subLaserCloud.shutdown();
            cout<< GREEN<<"record finished!" <<RESET <<endl;
        }
            //cout<<"NumPublishers "<<subLaserCloud.getNumPublishers()<<endl;
    }
};