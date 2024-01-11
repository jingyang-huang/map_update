#pragma once

#include <iostream>
#include <memory>
#include <math.h>
#include <fstream>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/ndt.h>       // ndt配准文件头
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/ModelCoefficients.h>
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


typedef Eigen::Matrix<short, 3, 1> voxelCoord;
typedef Eigen::Matrix<float, 3, 1> Point; //Vector3f
typedef Eigen::Matrix<float, 3, 1> Position;
typedef Eigen::Matrix<float, 3, 1> Ray;
typedef std::size_t GlobalIndex;
extern int quene_size;
extern int LOG_THRESH;

class voxel
{
    private:
    int LOG_THRESH = 4000;

    public:
    int16_t log_prob;
    std::vector<Point> points;
    voxel(void)
    {
        log_prob = 0;
    }
    void insertPoint(pcl::PointXYZ point);
    void update_prob(int prob);

};

class MapUpdate
{
    private:
    
    float voxel_size;
    float voxel_size_inv ;
    float Vmap_expand_ratio;
    size_t voxel_num_x,voxel_num_y,voxel_num_z; 
    GlobalIndex map_size;
    public:
    ros::NodeHandle nh;
    ros::Publisher Mmap_pub;
    ros::Publisher Scan_pub;
    ros::Publisher Ray_pub;
    ros::Publisher temp_pub;
    ros::Publisher start_pub;
    ros::Publisher Vmap_pub;
    //std::shared_ptr<voxel> Vmap_ptr;
    //pcl::visualization::PCLVisualizer viewer;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    std::shared_ptr<std::vector<voxel>> Vmap_ptr;
    std::shared_ptr<std::vector<voxelCoord>> Vmapidxs_ptr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Mmap_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Vmap_pts2show;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxCloud;
    
    int pos_logprob;
    int neg_logprob;
    int init_logprob;
    bool if_view_ray;
    bool if_view_Vmap;
    bool if_view_process;
    bool allow_plane_align;

    float down_sample_size;
    float step = 0.1f;
    float steps ;
    std::string Mmapname;
    std::string SavePrefix;
    std::string Savevoxpcd; 
    std::string SaveMmapds;   
    std::string RecordPrefix;
    std::string RotatedRecordPrefix;
    const std::string ReadTffilename = "tf.txt";//默认不能动

    std::string ReadMmapfile;
    std::string ReadTffile;
    std::string SavePcdfile;
    std::string ReadScanDir;

    Eigen::Affine3f inner_transform;

    //索引转换函数
    voxelCoord idx2xyz( GlobalIndex idx);
    GlobalIndex xyz2idx( voxelCoord xyz);
    Position xyz2realpos(voxelCoord xyz);
    voxelCoord realpos2xyz( pcl::PointXYZ pcl_point);
    voxelCoord realpos2xyz( Position point);

    //主功能函数
    void generateVmap();
    void updateChanges();  
    void updateChangesFromLIOSAM();
    void restoreMmap();
    void clearTempPcds();
    Eigen::Matrix4f linetoMatrix(std::string line_info);
    Eigen::Matrix4f linetoMatrixLIOSAM(std::string line_info);
    int CountLines(std::string filename);
    bool getNextRayIndex(Point start, Point end,GlobalIndex * crossvox_idx) ;
    void test();
    
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

    MapUpdate()
    {
        //算法参数
        // nh.param<std::string>("update/SavePrefix", SavePrefix, "../data/");
        // nh.param<std::string>("update/Savevoxpcd", Savevoxpcd, "voxCloudmap.pcd" );
        // nh.param<std::string>("update/ReadPrefix", ReadPrefix, "../data/record/");
        // nh.param<std::string>("update/ReadTfname", ReadTffilename, "tf.txt" );
        // nh.param<std::string>("update/Mmapname", Mmapname, "");
        nh.param<int>("update/init_logprob", init_logprob, 10);
        nh.param<int>("update/pos_logprob", pos_logprob, 4);
        nh.param<int>("update/neg_logprob", neg_logprob, -1);
        nh.param<int>("update/quene_size", quene_size, 5);
        nh.param<int>("update/LOG_THRESH", LOG_THRESH, 4000);
        nh.param<float>("update/voxel_size", voxel_size, 0.1);
        nh.param<float>("update/Vmap_expand_ratio", Vmap_expand_ratio, 1.0);
        nh.param<float>("update/down_sample_size", down_sample_size, 0.15f);
        nh.param<bool>("update/allow_plane_align", allow_plane_align, false);
        nh.param<bool>("update/if_view_process", if_view_process, false);
        nh.param<bool>("update/if_view_Vmap", if_view_Vmap, false);
        nh.param<bool>("update/if_view_ray", if_view_ray, false);
        cout<< BLUE<<"update params "<<RESET<<endl;
        //文件IO名
        nh.param<std::string>("RecordPrefix",RecordPrefix,"");
        std::cout<<"RecordPrefix="<<RecordPrefix<<std::endl;
        nh.param<std::string>("RotatedRecordPrefix",RotatedRecordPrefix,"");
        std::cout<<"RotatedRecordPrefix="<<RotatedRecordPrefix<<std::endl;
        nh.param<std::string>("ReadMmapfile",ReadMmapfile,"");
        std::cout<<"ReadMmapfile="<<ReadMmapfile<<std::endl;
        nh.param<std::string>("SavePcdfile",SavePcdfile,"");
        std::cout<<"SavePcdfile="<<SavePcdfile<<std::endl;
        nh.param<std::string>("ReadTffile",ReadTffile,"");
        std::cout<<"ReadTffile="<<ReadTffile<<std::endl;
        nh.param<std::string>("ReadScanDir",ReadScanDir,"");
        std::cout<<"ReadScanDir="<<ReadScanDir<<std::endl;

        mkdirIfnotexist(RotatedRecordPrefix);
        Mmapname = ReadMmapfile.substr(ReadMmapfile.find_last_of('/')+1);
        SaveMmapds = Mmapname.substr(0,Mmapname.rfind("."))  + "_downsampled.pcd" ; //
        SavePrefix = SavePcdfile.substr(0,SavePcdfile.find_last_of('/')+1);
        voxel_size_inv = 1/voxel_size;
        Mmap_pub = nh.advertise<sensor_msgs::PointCloud2> ("Mmap", 1);
        Scan_pub = nh.advertise<sensor_msgs::PointCloud2> ("Scan", 1);
        Ray_pub = nh.advertise<sensor_msgs::PointCloud2> ("Ray", 1);
        start_pub = nh.advertise<sensor_msgs::PointCloud2> ("start", 1);
        Vmap_pub = nh.advertise<sensor_msgs::PointCloud2> ("Vmap", 1);
        Mmap_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>());//用这种方法可以对Ptr分配内存
        Vmap_pts2show.reset(new pcl::PointCloud<pcl::PointXYZ>());
        inner_transform = Eigen::Affine3f::Identity();
        
    }
};