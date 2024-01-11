#include "voxel.h"

using namespace std;
int LOG_THRESH;
int quene_size;



void voxel::insertPoint(pcl::PointXYZ pcl_point)//TODO:insert是粗暴队列加入，没找关键点
{
    //cout<<this->cloud.size()<<endl;

    Point pt(pcl_point.x,pcl_point.y,pcl_point.z);
    //Point pt = pcl_point.getVector3fMap();
    if(this->points.size() < quene_size)
    {
        this->points.emplace_back(pt);
    }
    else //=5
    {
        std::vector<Point>::iterator index = this->points.begin();
        this->points.erase(index);//删除第一个
        this->points.emplace_back(pt);
    }
    
}
//从pcd文件生成地图
void voxel::update_prob(int prob)
{
    if(this->log_prob > - LOG_THRESH&& this->log_prob < LOG_THRESH )
    {
        this->log_prob = this->log_prob + prob;
    }
}

voxelCoord MapUpdate::idx2xyz( GlobalIndex idx)
{
    
    uint16_t x,y,z,res;
    x = idx/(voxel_num_y*voxel_num_z);
    res = idx%(voxel_num_y*voxel_num_z);
    y = res/(voxel_num_z);
    z = res%voxel_num_z;
    voxelCoord voxidx(x,y,z);
    return voxidx;
}


GlobalIndex MapUpdate::xyz2idx( voxelCoord xyz)
{
    //test:三维到一维的转化是否成功
    // uint32_t  pos = (10)*voxel_num_y*voxel_num_z+(9)*voxel_num_z + (8);
    // cout<< Vmapidxs_ptr->at(pos)<<endl; 
    GlobalIndex pos = xyz[0]*voxel_num_y*voxel_num_z+ xyz[1]*voxel_num_z + xyz[2];
    return pos;
}


//使用vox的中心代替vox中的点；另一种方法是使用vox内所有点的平均数
Position MapUpdate::xyz2realpos(voxelCoord xyz)
{
    Position pos;
    pos[0] = xyz[0]*voxel_size + voxel_size*0.5;
    pos[1] = xyz[1]*voxel_size + voxel_size*0.5;
    pos[2] = xyz[2]*voxel_size + voxel_size*0.5;
    return pos;

}
voxelCoord MapUpdate::realpos2xyz( pcl::PointXYZ pcl_point)
{
    voxelCoord vox_xyz;
    vox_xyz[0] = ceil(pcl_point.x*voxel_size_inv);
    vox_xyz[1] = ceil(pcl_point.y*voxel_size_inv);
    vox_xyz[2] = ceil(pcl_point.z*voxel_size_inv);
    return vox_xyz;
}
voxelCoord MapUpdate::realpos2xyz( Position point)
{
    voxelCoord vox_xyz;
    vox_xyz[0] = ceil(point[0]*voxel_size_inv);
    vox_xyz[1] = ceil(point[1]*voxel_size_inv);
    vox_xyz[2] = ceil(point[2]*voxel_size_inv);
    return vox_xyz;
}

void MapUpdate::test()
{
    //test: xyz2realpos

    
}

void MapUpdate::generateVmap()
{
    //读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr Mmap(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_read(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ds_write(new pcl::PointCloud<pcl::PointXYZ>);
    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(ReadMmapfile, *map_read) == -1)//必须用绝对路径吗
    {
        string err_info = "Couldn't read file " + Mmapname + "\n" ; 
        PCL_ERROR(err_info.data());
        return ;
    }

    //Mmap下采样并保存
    pcl::VoxelGrid<pcl::PointXYZ> sor;    // 创建滤波（下采样）对象
    sor.setInputCloud(map_read);             //设置需要过滤的点云
    sor.setLeafSize(down_sample_size, down_sample_size, down_sample_size);    //设置滤波时创建的体素体积为1cm的立方体,意思是1cm 
    sor.filter(*map_ds_write);          //执行滤波处理，储存输出点云 

    pcl::io::savePCDFileASCII(SavePrefix + SaveMmapds, *map_ds_write); //不经过内部变换的纯采样点
    cout << "downsampled Mmap "<< SaveMmapds << endl;

    Eigen::Matrix4f align_rotation_inverse = Eigen::Matrix4f::Identity();
    //找到地面法向量，实现点云的水平校准
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType ( pcl::SACMODEL_PLANE );
    plane_seg.setMethodType ( pcl::SAC_RANSAC );
    plane_seg.setDistanceThreshold ( 0.01 );
    plane_seg.setInputCloud ( map_read );
    plane_seg.segment (*plane_inliers, *plane_coefficients);//得到平面系数，进而得到平面法向量

    Eigen::Vector3f plane_normal;
    plane_normal << plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2];
    cout<<CYAN<<"plane_normal "<<endl<<plane_normal<<RESET<<endl;

    if(allow_plane_align)
    {
        //是否允许进行变换
        Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
        /* x转移矩阵
        |1   0   0    |
        |0  cos -sin  |
        |0  sin  cos  |
        */
        double cosx, sinx;
        double dis = sqrt(pow(plane_normal[0], 2) + pow(plane_normal[1], 2) + pow(plane_normal[2], 2));
        cosx = sqrt(pow(plane_normal[0], 2) + pow(plane_normal[2], 2))/dis;
        sinx = plane_normal[1] / dis;
        transform_x(1, 1) = cosx;
        transform_x(1, 2) = -sinx;
        transform_x(2,1) = sinx;
        transform_x(2,2) = cosx;

        Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
        /* y转移矩阵
        |cos  0  sin  |
        |0    1   0   |
        |-sin 0  cos  |
        */
        double cosy, siny;
        cosy = plane_normal[2]/dis;
        siny = sqrt(pow(plane_normal[0],2)+pow(plane_normal[1],2))/dis;
        // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        transform_y(0, 0) = cosy;
        transform_y(0, 2) = siny;
        transform_y(2, 0) = -siny;
        transform_y(2, 2) = cosy;

        align_rotation_inverse = transform_x*transform_y;
        pcl::transformPointCloud(*map_read, *map_read, align_rotation_inverse); //对map_read先进行变换
    }

    // pcl::io::savePCDFileASCII(SavePrefix + "rotated_map.pcd", *map_read);
    //cout << "Mmap process ok "<< "processed_map.pcd" << endl;

    //确定分辨率
    int w = map_read->width ;
    int h = map_read->height;
    std::cout << "width "
              << map_read->width <<" height "<< map_read->height // 宽*高
              << " data points from " + Mmapname
              << std::endl;
    //划分体素格子
    pcl::PointXYZ min, max;
    
    pcl::getMinMax3D(*map_read, min, max); //查找点云的x，y，z方向的极值
    
    /// 方式2：Affine3f
	// 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
	// Eigen::Affine3f inner_transform = Eigen::Affine3f::Identity();
	// 定义平移
    // float x_width = max.x- min.x;
    // float y_width = max.y- min.y;
    // float z_width = max.z- min.z;
    cout << "original map " <<endl;
    cout << "->min_x = " << min.x << endl;
	cout << "->min_y = " << min.y << endl;
	cout << "->min_z = " << min.z << endl;
	cout << "->max_x = " << max.x << endl;
	cout << "->max_y = " << max.y << endl;
	cout << "->max_z = " << max.z << endl;

    Eigen::Affine3f minmax_transform = Eigen::Affine3f::Identity();
	minmax_transform.translation() << -min.x +  (max.x- min.x)* (Vmap_expand_ratio-1.0)/2 
                                     , -min.y +  (max.y- min.y)* (Vmap_expand_ratio-1.0)/2 
                                     , -min.z +  (max.z- min.z)* (Vmap_expand_ratio-1.0)/2 ;	// 三个数分别对应X轴、Y轴、Z轴方向上的平移
	// 定义旋转矩阵，绕z轴
	minmax_transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));	//同理，UnitX(),绕X轴；UnitY(),绕Y轴.

	pcl::transformPointCloud(*map_read, *Mmap, minmax_transform);	//再平移到全部为正

    inner_transform =  minmax_transform * align_rotation_inverse; //这是处理阶段用的两个两个变换

    // cout<<"1-2"<<endl;
    cout<<"transformed map"<<endl;

    pcl::getMinMax3D(*Mmap, min, max); //查找点云的x，y，z方向的极值

    cout << "->min_x = " << min.x << endl;
	cout << "->min_y = " << min.y << endl;
	cout << "->min_z = " << min.z << endl;
	cout << "->max_x = " << max.x << endl;
	cout << "->max_y = " << max.y << endl;
	cout << "->max_z = " << max.z << endl;

    voxel_num_x = floor(Vmap_expand_ratio*(max.x - min.x)*voxel_size_inv);
    voxel_num_y = floor(Vmap_expand_ratio*(max.y - min.y)*voxel_size_inv);
    voxel_num_z = floor(Vmap_expand_ratio*(max.z - min.z)*voxel_size_inv);

    cout << "->voxel_num_x = " << voxel_num_x<< endl;
    cout << "->voxel_num_y = " << voxel_num_y<< endl;
    cout << "->voxel_num_z = " << voxel_num_z<< endl;

    //Mmap下采样用于显示
    sor.setInputCloud(Mmap);             //设置需要过滤的点云
    sor.setLeafSize(down_sample_size,down_sample_size, down_sample_size);    //设置滤波时创建的体素体积为1cm的立方体,意思是1cm 
    sor.filter(*Mmap_downsampled);          //执行滤波处理，储存输出点云 

    // pcl::io::savePCDFileASCII(SavePrefix + "processed_map.pcd", *Mmap_downsampled);
    // cout << "Mmap process ok "<< "processed_map.pcd" << endl;
    //
    //for 

    // //更改了voxel的实现，现在没那么多没用的指针了
    size_t size = -1;
    map_size = (voxel_num_x)*(voxel_num_y)*(voxel_num_z);
    cout << map_size<<endl;
    cout << size << " is max size_t size"<<endl;
    cout << 2305843009213693951<< " is max vector size"<<endl;
    // 
    Vmap_ptr  = std::make_shared<std::vector<voxel>>(map_size); 
    cout<< GREEN<< "allocate ok" << RESET<<endl;
    cout<< BLUE <<"map size is "<< Vmap_ptr->size()<<RESET <<endl;
    //cout<< (int)Vmap_ptr->at(10).log_prob<<endl;

    voxelCoord vox_xyz;
    std::vector<voxelCoord> Vmap_idxs_check; ;//暂时check有没有重复
    cout << "Generating Vmap " <<endl;
    for (size_t i = 0; i < Mmap->points.size(); i++)
    {      
        vox_xyz = realpos2xyz(Mmap->points[i]);
        //cout<<"for point "<<i<<endl;
        GlobalIndex idx = xyz2idx(vox_xyz);
        Vmap_ptr->at(idx).insertPoint(Mmap->points[i]);
        Vmap_ptr->at(idx).log_prob = init_logprob; //比较坚固，给10

        if(if_view_Vmap){
            std::vector<voxelCoord>::iterator it=std::find(Vmap_idxs_check.begin(), Vmap_idxs_check.end(), vox_xyz);
            if ( it == Vmap_idxs_check.end()){ //防止重复添加
                Vmap_idxs_check.emplace_back(vox_xyz);
                Vmap_pts2show->emplace_back(pcl::PointXYZ(xyz2realpos(vox_xyz)[0], xyz2realpos(vox_xyz)[1], xyz2realpos(vox_xyz)[2]));//加入点云用来show
            } 
        }
    }
    cout<< GREEN << "Vmap generate ok! "<< RESET <<endl;
    cout << "unique vox num " << Vmap_pts2show->size() <<endl;

}


//从Vmap中的voxel恢复点云并显示
void MapUpdate::restoreMmap()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //把所有Vmap里的points放进来
    //现在对每个voxel都循环一下，明显太慢了
    for (size_t i = 0; i < Vmap_ptr->size(); i++)
    {
        if(Vmap_ptr->at(i).log_prob > 0)
        {
            //cout << "for vox "<< i <<endl;
            for(uint16_t n = 0; n < Vmap_ptr->at(i).points.size(); n++)
            {
                Point this_pt = Vmap_ptr->at(i).points[n];
                //voxCloud->emplace_back(pcl::PointXYZ(this_pt[0], this_pt[1], this_pt[2]));
                voxCloud->emplace_back(pcl::PointXYZ(this_pt[0], this_pt[1], this_pt[2]));
            }
        }
    }
    pcl::transformPointCloud(*voxCloud, *voxCloud, inner_transform.inverse());
    pcl::io::savePCDFileASCII(SavePcdfile, *voxCloud); //SavePrefix + Savevoxpcd
    cout<<GREEN<<"voxCloud restore ok!"<< RESET<<endl;

}

//读取txt文件，保存到vec，转成matrix，用到点云变换
Eigen::Matrix4f MapUpdate::linetoMatrix(string line_info)
{
    string input_result;
    int16_t read_lines = 0;
    Eigen::VectorXf read_vec(16);
    
    stringstream input(line_info);
    //cout<<line_info<<endl;
    for (int j = 0; input >> input_result; j++) {
            string::size_type size;
            //cout<<input_result<<endl;
            read_vec(j) = stof(input_result, &size);//string 转float
        }
    read_lines++;

    Eigen::Matrix4f A = Eigen::Map<Eigen::Matrix4f>(read_vec.data()).transpose();
    return A;
}

Eigen::Matrix4f MapUpdate::linetoMatrixLIOSAM(string line_info)
{
    string input_result;
    int16_t read_lines = 0;
    Eigen::VectorXf read_vec(16);

    stringstream input(line_info);

    //cout<<line_info<<endl;
    for (int j = 0; input >> input_result; j++) {
            string::size_type size;
            //cout<<input_result<<endl;
            read_vec(j) = stof(input_result, &size);//string 转float
        }
    read_vec.tail(4) << 0 , 0 , 0 , 1; 
    read_lines++;

    Eigen::Matrix4f A = Eigen::Map<Eigen::Matrix4f>(read_vec.data()).transpose();
    return A;
}

int MapUpdate::CountLines(std::string Tffile)
{
    ifstream ReadFile;
    int n=0;
    string tmp;
    ReadFile.open(Tffile,ios::in);//ios::in 表示以只读的方式读取文件
    if(ReadFile.fail())//文件打开失败:返回0
    {
        return 0;
    }
    else//文件存在
    {
        while(getline(ReadFile,tmp,'\n'))
        {
            n++;
        }
        ReadFile.close();
        return n;
    }
}


bool MapUpdate::getNextRayIndex(Point start, Point end,GlobalIndex * crossvox_idx) 
{   
    
    Ray free_ray = start - end; // 反着来，不容易越界
    Ray unit_free_ray = free_ray.normalized();

    float free_ray_length = std::sqrt(std::pow(free_ray[0],2)+std::pow(free_ray[1],2)+std::pow(free_ray[2],2));

    if(free_ray_length < step)//光线长度比step短，没法用光线模型
    {
        return false;
    }

    steps = steps + step;
    if(steps > free_ray_length){
        return false;
    }
    Point step_end = steps*unit_free_ray + end;
    voxelCoord  step_end_xyz = realpos2xyz(step_end);
    if(xyz2idx(step_end_xyz) > map_size)
    {
        return false;
    }
    else
    {
        *crossvox_idx = xyz2idx(step_end_xyz); //对负数的处理，边界问题
        return true;
    }
}



void MapUpdate::updateChanges()
{
    //读取数据，获取points和tf信息
    //Step1 读取文件，并且发布出来方便观测
    //读取点云readFiletoMatrix
    //Step2 完整读取一次的所有数据，并更新在viewer里，同时留出voxelmap的更新接口
    int times;
    bool this_time_ok = 0;
    bool file_ok = 0;
    Eigen::Matrix4f eigen_transform;
    ifstream fin( RecordPrefix +  ReadTffilename  , ios::in); 
    if (fin)
	{
        file_ok = 1;
    }
    else
    {
        cout<< "open file wrong!"<<endl;
    }

    int totaltimes = CountLines(RecordPrefix +  ReadTffilename );
    cout<< "totaltimes "<<totaltimes<<endl;
    for(times = 1;times <= totaltimes;times++)
    {
        if(times%50 ==0 || (times < 50)||(totaltimes - times < 50)){
            cout<< "times "<<times<<"/"<<totaltimes<<endl;
        }
        
        std::string line_info;
        if(file_ok)
        {
            if(getline(fin, line_info))
            {
                this_time_ok = 1;
                eigen_transform = linetoMatrix(line_info);
                //cout << eigen_transform <<endl;
            }
        }

        std::string this_pcd_name = std::to_string(times) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr Scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_read(new pcl::PointCloud<pcl::PointXYZ>);
        //打开点云文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(RecordPrefix + this_pcd_name, *scan_read) == -1)//必须用绝对路径吗
        {
            PCL_ERROR("Couldn't read file %d.pcd \n",times);
            return ;
        }

        pcl::transformPointCloud(*scan_read, *Scan, eigen_transform);

        pcl::io::savePCDFileASCII(RotatedRecordPrefix+ this_pcd_name , *Scan); 
        pcl::transformPointCloud(*Scan, *Scan, inner_transform);

        
        //DONE:对齐后的pcd建立栅格地图
        //定义发布的消息
        sensor_msgs::PointCloud2 output;
        //把点云转化为ros消息

        voxelCoord thisidx;
        Eigen::Affine3f A;
        A = inner_transform * eigen_transform;
        Point start,end;
        start = A.translation();
        //voxelCoord start_vox, end_vox;
        voxelCoord end_vox;
        pcl::PointCloud<pcl::PointXYZ>::Ptr StartPoint(new pcl::PointCloud<pcl::PointXYZ>);

        if(if_view_process){

            pcl::PointXYZ point_start(start(0),start(1),start(2)); 


            StartPoint->points.emplace_back(point_start);//point 也ok了，能看到point是在点云的起始点
            //同时由于每次循环会重新创建点云，所以也不存在需要删除的问题

            pcl::toROSMsg(*Scan, output);
            output.header.frame_id = "map";
            Scan_pub.publish(output);

            pcl::toROSMsg(*StartPoint, output);
            output.header.frame_id = "map";
            start_pub.publish(output);

            //突然明白，这是Vtemp的啊，肯定不行。必须要都是V的才可以
            pcl::toROSMsg(*Mmap_downsampled, output);
            output.header.frame_id = "map";
            Mmap_pub.publish(output);

        }
        if(if_view_Vmap){
                pcl::toROSMsg(*Vmap_pts2show, output);
                output.header.frame_id = "map";
                Vmap_pub.publish(output);
        }

        Ray free_ray,unit_free_ray;
        

        pcl::PointCloud<pcl::PointXYZ>::Ptr RayPoints(new pcl::PointCloud<pcl::PointXYZ>);
        

        sensor_msgs::PointCloud2 ray_output;
        for (size_t i = 0; i < Scan->points.size(); i++)
        {   
            //TODO:建立光线投影模型，更新Vmap
            
            end <<  Scan->points[i].x , Scan->points[i].y , Scan->points[i].z;
            unit_free_ray = (end - start).normalized(); //每次朝那个方向迭代一点检测是否和新的voxel有交集            

            GlobalIndex cross_voxel_idx,end_vox_idx;
            voxelCoord vox_xyz;
            steps = 0;
            while (getNextRayIndex(start,end,&cross_voxel_idx)) 
            {
                vox_xyz = idx2xyz(cross_voxel_idx);
                Vmap_ptr->at(cross_voxel_idx).update_prob(neg_logprob);
                if(if_view_ray){
                    RayPoints->points.emplace_back(pcl::PointXYZ(xyz2realpos(vox_xyz)[0], xyz2realpos(vox_xyz)[1], xyz2realpos(vox_xyz)[2]));//加入点云用来show
                }
            }
            end_vox = realpos2xyz(Scan->points[i]);
            end_vox_idx = xyz2idx(end_vox);
            //cout <<"5-3"<<endl;
            if(end_vox_idx < map_size) // 范围内才更新
            {           
                Vmap_ptr->at(end_vox_idx).update_prob(pos_logprob);
                Vmap_ptr->at(end_vox_idx).insertPoint(Scan->points[i]);
            }
            if(if_view_ray){
                pcl::toROSMsg(*RayPoints, ray_output);
                ray_output.header.frame_id = "map";
                Ray_pub.publish(ray_output);
                RayPoints->clear();
            }

        }

    }
    cout<< GREEN << "Process done!"<<RESET<<endl;
    
}

void MapUpdate::updateChangesFromLIOSAM()
{
    //读取数据，获取points和tf信息
    //Step1 读取文件，并且发布出来方便观测
    //读取点云readFiletoMatrix
    //Step2 完整读取一次的所有数据，并更新在viewer里，同时留出voxelmap的更新接口
    int times;
    bool this_time_ok = 0;
    bool file_ok = 0;
    Eigen::Matrix4f eigen_transform;
    ifstream fin( ReadTffile , ios::in);
    if (fin)
	{
        file_ok = 1;
    }
    else
    {
        cout<< "open file wrong!"<<endl;
        //cout<< ReadPrefix + ReadTffilename <<endl;
    }

    int totaltimes = CountLines( ReadTffile );
    cout<< "totaltimes "<<totaltimes<<endl;
    for(times = 0; times < totaltimes ; times++)
    {
        if(times%50 ==0 || (times < 50)||(totaltimes - times < 50)){
            cout<< "times "<<times+1 <<"/"<<totaltimes<<endl;
            //cout<<"voxel size is "<<Vmapidxs_ptr->size()<<endl;
        }
        
        std::string line_info;
        if(file_ok)
        {
            if(getline(fin, line_info))
            {
                this_time_ok = 1;
                eigen_transform = linetoMatrixLIOSAM(line_info);
                //cout << eigen_transform <<endl;
            }
        }
        char ss[10];
        sprintf(ss,"%06d",times);
        string times_str(ss);
        std::string this_pcd_name = times_str + ".pcd";
        //cout<<"this_pcd_name " << this_pcd_name<<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr Scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_read(new pcl::PointCloud<pcl::PointXYZ>);
        //打开点云文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(ReadScanDir + this_pcd_name, *scan_read) == -1)//必须用绝对路径吗
        {
            PCL_ERROR("Couldn't read file %d.pcd \n",times);
            return ;
        }

        pcl::transformPointCloud(*scan_read, *Scan, eigen_transform);
        pcl::transformPointCloud(*Scan, *Scan, inner_transform);

        //DONE:对齐后的pcd建立栅格地图
        //定义发布的消息
        sensor_msgs::PointCloud2 output;
        //把点云转化为ros消息

        //试一下遍历点来更新voxel
        int16_t pos_x,pos_y,pos_z;
        //cout<<Scan->points.size()<<endl;

        voxelCoord thisidx;
        Eigen::Affine3f A;
        A = inner_transform * eigen_transform;
        Point start,end;
        start = A.translation();
        //voxelCoord start_vox, end_vox;
        voxelCoord end_vox;
        pcl::PointCloud<pcl::PointXYZ>::Ptr StartPoint(new pcl::PointCloud<pcl::PointXYZ>);

        if(if_view_process){

            pcl::PointXYZ point_start(start(0),start(1),start(2)); //极其顺滑

            StartPoint->points.emplace_back(point_start);//point 也ok了，能看到point是在点云的起始点

            pcl::toROSMsg(*Scan, output);
            output.header.frame_id = "map";
            Scan_pub.publish(output);

            pcl::toROSMsg(*StartPoint, output);
            output.header.frame_id = "map";
            start_pub.publish(output);

            //突然明白，这是Vtemp的啊，肯定不行。必须要都是V的才可以
            pcl::toROSMsg(*Mmap_downsampled, output);
            output.header.frame_id = "map";
            Mmap_pub.publish(output);
        }

        if(if_view_Vmap){
            pcl::toROSMsg(*Vmap_pts2show, output);
            output.header.frame_id = "map";
            Vmap_pub.publish(output);
        }

        Ray free_ray,unit_free_ray;

        pcl::PointCloud<pcl::PointXYZ>::Ptr RayPoints(new pcl::PointCloud<pcl::PointXYZ>);

        sensor_msgs::PointCloud2 ray_output;
        for (size_t i = 0; i < Scan->points.size(); i++)
        {               
            end <<  Scan->points[i].x , Scan->points[i].y , Scan->points[i].z;
            unit_free_ray = (end - start).normalized(); //每次朝那个方向迭代一点检测是否和新的voxel有交集            

            GlobalIndex cross_voxel_idx,end_vox_idx;
            voxelCoord vox_xyz;
            steps = 0;
            while (getNextRayIndex(start,end,&cross_voxel_idx)) 
            {
                //cout <<"5-1"<<endl;
                vox_xyz = idx2xyz(cross_voxel_idx);
                Vmap_ptr->at(cross_voxel_idx).update_prob(neg_logprob);
                if(if_view_ray){
                    RayPoints->points.emplace_back(pcl::PointXYZ(xyz2realpos(vox_xyz)[0], xyz2realpos(vox_xyz)[1], xyz2realpos(vox_xyz)[2]));//加入点云用来show
                }
                //cout <<"5-2"<<endl;
            }
            end_vox = realpos2xyz(Scan->points[i]);
            end_vox_idx = xyz2idx(end_vox);
            //cout <<"5-3"<<endl;
            if(end_vox_idx < map_size) // 范围内才更新
            {           
                Vmap_ptr->at(end_vox_idx).update_prob(pos_logprob);
                Vmap_ptr->at(end_vox_idx).insertPoint(Scan->points[i]);
            }
            if(if_view_ray){
            pcl::toROSMsg(*RayPoints, ray_output);
            ray_output.header.frame_id = "map";
            Ray_pub.publish(ray_output);
            RayPoints->clear();
            }

        }

    }
    cout<< GREEN << "Process done!"<<RESET<<endl;
}

