#include <surfel_map.h>
#include <timer.h>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include "CameraPoseVisualization.h"

SurfelMap::SurfelMap(ros::NodeHandle &_nh):
nh(_nh),
// fuse_param_gpuptr(NULL),
inactive_pointcloud(new PointCloud)
{
    // get the parameters
    bool get_all = true;
    get_all &= nh.getParam("cam_width", cam_width);
    get_all &= nh.getParam("cam_height", cam_height);
    get_all &= nh.getParam("cam_fx", cam_fx);
    get_all &= nh.getParam("cam_cx", cam_cx);
    get_all &= nh.getParam("cam_fy", cam_fy);
    get_all &= nh.getParam("cam_cy", cam_cy);
    camera_matrix = Eigen::Matrix3d::Zero();
    camera_matrix(0, 0) = cam_fx;
    camera_matrix(0, 2) = cam_cx;
    camera_matrix(1, 1) = cam_fy;
    camera_matrix(1, 2) = cam_cy;
    camera_matrix(2, 2) = 1.0;

    get_all &= nh.getParam("fuse_far_distence", far_dist);
    get_all &= nh.getParam("fuse_near_distence", near_dist);
    get_all &= nh.getParam("drift_free_poses", drift_free_poses);

    if(!get_all)
        printf("ERROR! Do not have enough parameters!");
    else
    {
        printf("Have the following settings: \n");
        printf("camera matrix: \n");
        cout << camera_matrix << endl;
        printf("fuse the distence between %4f m and %4f m.\n", near_dist, far_dist);
    }


    fusion_functions.initialize(cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, far_dist, near_dist);

    // ros publisher
    pointcloud_publish = nh.advertise<PointCloud>("pointcloud", 10);
    raw_pointcloud_publish = nh.advertise<PointCloud>("raw_pointcloud", 10);
    active_pointcloud_publish = nh.advertise<PointCloud>("active_pointcloud", 10);
    inactive_pointcloud_publish = nh.advertise<PointCloud>("inactive_pointcloud", 10);
    loop_path_publish = nh.advertise<nav_msgs::Path>("fusion_loop_path", 10);
    driftfree_path_publish = nh.advertise<visualization_msgs::Marker>("driftfree_loop_path", 10);
    loop_marker_publish = nh.advertise<visualization_msgs::Marker>("loop_marker", 10);
    camera_marker_publish = nh.advertise<visualization_msgs::MarkerArray>("camera_marker", 10);

}

SurfelMap::~SurfelMap()
{
    // if (fuse_param_gpuptr)
    //     cudaFree(fuse_param_gpuptr);
}

void SurfelMap::save_map(const std_msgs::StringConstPtr &save_map_input)
{
    string save_name = save_map_input->data;
    printf("save mesh modelt to %s.\n", save_name.c_str());
    save_mesh(save_name);
    printf("save done!\n");
}

void SurfelMap::image_input(const sensor_msgs::ImageConstPtr &image_input)
{
    // printf("receive image!\n");
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_input, sensor_msgs::image_encodings::MONO8);
    cv::Mat image = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;
    image_buffer.push_back(std::make_pair(stamp, image));
    synchronize_msgs();
}

void SurfelMap::depth_input(const sensor_msgs::ImageConstPtr &depth_input)
{
    // printf("receive depth!\n");
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(depth_input, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat image = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;
    depth_buffer.push_back(std::make_pair(stamp, image));
    synchronize_msgs();
}

void SurfelMap::synchronize_msgs()
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> total_time;
    start_time = std::chrono::system_clock::now();

    if(pose_reference_buffer.size() == 0)
        return;
    ros::Time fuse_stamp = std::get<0>(pose_reference_buffer.front());
    double pose_reference_time = fuse_stamp.toSec();
    bool find_image = false, find_depth = false;
    while(image_buffer.size() > 0)
    {
        double image_time = image_buffer.front().first.toSec();
        if(image_time < pose_reference_time)
        {
            image_buffer.pop_front();
        }
        else if(image_time == pose_reference_time)
        {
            find_image = true;
            break;
        }
    }
    while(depth_buffer.size() > 0)
    {
        double depth_time = depth_buffer.front().first.toSec();
        if(depth_time < pose_reference_time)
        {
            depth_buffer.pop_front();
        }
        else if(depth_time == pose_reference_time)
        {
            find_depth = true;
            break;
        }
    }

    if((!find_image) || (!find_depth))
        return;

    geometry_msgs::Pose relative_pose_ros = std::get<1>(pose_reference_buffer.front());
    int relative_index = std::get<2>(pose_reference_buffer.front());

    Eigen::Matrix4d reference_pose, fuse_pose, relative_pose;
    pose_ros2eigen(poses_database[relative_index].cam_pose, reference_pose);
    pose_ros2eigen(relative_pose_ros, relative_pose);
    fuse_pose = reference_pose * relative_pose;
    geometry_msgs::Pose fuse_pose_ros;
    pose_eigen2ros(fuse_pose, fuse_pose_ros);

    move_add_surfels(relative_index);

    // fuse the current image/depth
    printf("fuse map begins!\n");
    cv::Mat image, depth;
    image = image_buffer.front().second;
    depth = depth_buffer.front().second;
    fuse_map(image, depth, fuse_pose.cast<float>(), relative_index);
    printf("fuse map done!\n");
    pose_reference_buffer.pop_front();
    
    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("fuse surfels cost %f ms.\n", total_time.count()*1000.0);
    start_time = std::chrono::system_clock::now();    

    publish_pose_graph(fuse_stamp, relative_index);
    publish_camera_position(fuse_stamp, fuse_pose_ros);
    {
        publish_inactive_pointcloud(fuse_stamp);
        publish_active_pointcloud(fuse_stamp);
    }
    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("publish results cost %f ms.\n", total_time.count()*1000.0);
    calculate_memory_usage();
}

//!@brief 从SLAM那获得三个消息
//!@param1 loop_stamp_input 获得的是闭环信息
//!@param2 loop_path_input  表示得到的是关键帧的位姿序列->这里位姿序列是指和该Map有关的所有关键帧位姿和其对应的时间戳
//!@param3 this_pose_input  表示的是当前关键帧信息
void SurfelMap::orb_results_input(
    const sensor_msgs::PointCloudConstPtr &loop_stamp_input,
    const nav_msgs::PathConstPtr &loop_path_input,
    const nav_msgs::OdometryConstPtr &this_pose_input)
{
    //!@brief 开始处理新的数据帧
    printf("\nbegin new frame process!!!\n");
    geometry_msgs::Pose input_pose = this_pose_input->pose.pose;

    // transform the kitti pose
    //!@brief 这里是由于Kitti的坐标系,需要将其进行更改
    static Eigen::Matrix4d transform_kitti;
    {
        Eigen::Matrix4d received_psoe;
        pose_ros2eigen(input_pose, received_psoe);        
        if(poses_database.size() == 0)
        {
            Eigen::Matrix4d idea_pose;
            idea_pose = Eigen::Matrix4d::Zero();
            idea_pose(0,0) = 1.0;
            idea_pose(1,2) = 1.0;
            idea_pose(2,1) = -1.0;
            idea_pose(3,3) = 1.0;
            transform_kitti = idea_pose * received_psoe.inverse();
        }
        Eigen::Matrix4d transformed_pose;
        //!@brief transfom_kitti = idea_pose * received_psoe.inverse() * received_psoe;
        transformed_pose = transform_kitti * received_psoe;
        pose_eigen2ros(transformed_pose, input_pose);
    }
    // transform end
    
    // first update the poses
    /**********************************************************************************
     * !@brief 将poses_database中的位姿进行更新，更新的是每一帧对应的loop_pose
     * 如果loop_pose与之前的cam_pose信息不一样，则代表orb_slam中检测到了闭环，进行了位姿更新
     * ********************************************************************************/
    bool loop_changed = false;
    for(int i = 0; i < poses_database.size() && i < loop_path_input->poses.size(); i++)
    {
        poses_database[i].loop_pose = loop_path_input->poses[i].pose;
        {
            // transform the kitti pose
            Eigen::Matrix4d received_pose, transformed_pose;
            pose_ros2eigen(poses_database[i].loop_pose, received_pose);
            transformed_pose = transform_kitti *  received_pose;
            pose_eigen2ros(transformed_pose, poses_database[i].loop_pose);
        }
        if( poses_database[i].loop_pose.position.x != poses_database[i].cam_pose.position.x
            || poses_database[i].loop_pose.position.y != poses_database[i].cam_pose.position.y
            || poses_database[i].loop_pose.position.z != poses_database[i].cam_pose.position.z)
        {
            loop_changed = true;
        }
    }

    /**********************************************************************************
     * !@brief 如果当前位姿数据库中的数据量大于orb_slam中传入的关键帧位姿序列,则进行更新
     * 相当于利用匀速模型，将最后一帧的位姿差作为匀速，对后面都做相同的更新
     * *********************************************************************************/
    if(poses_database.size() > loop_path_input->poses.size())
    {
        //!@brief 取出关键帧位姿序列的最后一帧作为最后更新帧
        int last_update_index = loop_path_input->poses.size() - 1;
        // 开始更新的位置是关键帧位姿序列的最后序列号,也就是pose_data中大于loop_path_input的部分
        int start_index = loop_path_input->poses.size();
        Eigen::Matrix4d warp_pose, pre_pose, after_pose;
        // 前一帧的cam_pose作为更新前的位姿
        pose_ros2eigen(poses_database[last_update_index].cam_pose, pre_pose);
        // 后一帧的after_pose保存下来作为更新后的位姿
        pose_ros2eigen(poses_database[last_update_index].loop_pose, after_pose);
        // 判断更新前后位姿变化差异
        warp_pose = after_pose * pre_pose.inverse();
        // 现在开始loop_path_input后面的部分进行更新
        for(start_index; start_index < poses_database.size(); start_index++)
        {
            Eigen::Matrix4d this_pose_pre, this_pose_after;
            pose_ros2eigen(poses_database[start_index].cam_pose, this_pose_pre);
            this_pose_after = warp_pose * this_pose_pre;
            geometry_msgs::Pose after_pose_ros;
            pose_eigen2ros(this_pose_after, after_pose_ros);
            poses_database[start_index].loop_pose = after_pose_ros;
        }
    }

    // warp the surfels

    printf("warp the surfels according to the loop!\n");
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();
    // 如果闭环检测进行了全局位姿更新了，则开始进行warp_surfels()操作
    // 这里warp_surfels是用线程池来操作的->这里我都可以直接看warp_surfels的操作，来做我的内容了
    if(loop_changed)
    {
        warp_surfels();
    }
    end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> used_time = end_time - start_time;
    double all_time = used_time.count() * 1000.0;
    printf("warp end! cost %f ms.\n", all_time);
    

    // add loop information
    // 开始添加闭环检测的信息
    int loop_num = loop_stamp_input->channels[0].values.size() / 2;
    for(int i = 0; i < loop_num; i++)
    {
        int loop_first = loop_stamp_input->channels[0].values[i*2];
        int loop_second = loop_stamp_input->channels[0].values[i*2+1];
        if(loop_first < poses_database.size() && loop_second < poses_database.size())
        {
            if(std::find(
                poses_database[loop_first].linked_pose_index.begin(),
                poses_database[loop_first].linked_pose_index.end(),
                loop_second) == poses_database[loop_first].linked_pose_index.end())
            {
                if(std::find(poses_database[loop_first].linked_pose_index.begin(),
                    poses_database[loop_first].linked_pose_index.end(),
                    loop_second) == poses_database[loop_first].linked_pose_index.end())
                    poses_database[loop_first].linked_pose_index.push_back(loop_second);
                if(std::find(poses_database[loop_second].linked_pose_index.begin(),
                    poses_database[loop_second].linked_pose_index.end(),
                    loop_first) == poses_database[loop_second].linked_pose_index.end())
                    poses_database[loop_second].linked_pose_index.push_back(loop_first);
            }
        }
        else
        {
            printf("cannot find loop pose %d and %d, we have %d poses!\n", loop_first, loop_second, poses_database.size());
        }
    }

    // if the current pose is new keyframe
    bool is_new_keyframe;
    // 这里判断是不是关键帧，如果是关键帧的话，则加入，通过同步消息来添加Surfel点云
    if(this_pose_input->pose.covariance[0] > 0)
        is_new_keyframe = true;
    else
        is_new_keyframe = false;
    // the corner case that the first frame of the system
    if(poses_database.size() == 0)
        is_new_keyframe = true;
    /******************************************************************************
     * !@brief 如果是关键帧的话，则先开始创建pose_database中对应的Pose Element元素
     * ***************************************************************************/
    if(is_new_keyframe){
        // add new pose
        PoseElement this_pose_element;
        int this_pose_index = poses_database.size();
        // 将读取到的input_pose直接赋值进去，不做改动
        this_pose_element.cam_pose = input_pose;                            // 初始对应的位姿
        this_pose_element.loop_pose = input_pose;                           // loop 变化后应该对应的修改位姿，这里先暂时直接将得到的放入进去不管
        this_pose_element.cam_stamp = this_pose_input->header.stamp;        // 这里是将变换的时间戳放入进去，暂时不动
        if(poses_database.size() > 0)
        {
            //relative_index 获得当前参考帧的索引
            int relative_index = this_pose_input->pose.covariance[1];
            this_pose_element.linked_pose_index.push_back(relative_index);
            poses_database[relative_index].linked_pose_index.push_back(this_pose_index);
        }
        // 现在考虑第一帧的情况，将这帧参数放入位姿库中
        poses_database.push_back(this_pose_element);
        // 然后这些本地的Surfels对应的位姿都放入进去
        local_surfels_indexs.insert(this_pose_index);
    }

    // push the msg into the buffer for fusion
    int relative_index = this_pose_input->pose.covariance[1];
    Eigen::Matrix4d reference_pose, fuse_pose, relative_pose;
    pose_ros2eigen(poses_database[relative_index].cam_pose, reference_pose);
    pose_ros2eigen(input_pose, fuse_pose);
    relative_pose = reference_pose.inverse() * fuse_pose;
    geometry_msgs::Pose relative_pose_ros;
    pose_eigen2ros(relative_pose, relative_pose_ros);
    pose_reference_buffer.push_back(std::make_tuple(loop_stamp_input->header.stamp, relative_pose_ros, relative_index));
    synchronize_msgs();
}

void SurfelMap::pose_ros2eigen(geometry_msgs::Pose &pose, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond rotation_q;
    rotation_q.w() = pose.orientation.w;
    rotation_q.x() = pose.orientation.x;
    rotation_q.y() = pose.orientation.y;
    rotation_q.z() = pose.orientation.z;
    T.block<3,3>(0,0) = rotation_q.toRotationMatrix();
    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;
}

void SurfelMap::pose_eigen2ros(Eigen::Matrix4d &T, geometry_msgs::Pose &pose)
{
    Eigen::Quaterniond rotation_q(T.block<3,3>(0,0));
    pose.orientation.w = rotation_q.w();
    pose.orientation.x = rotation_q.x();
    pose.orientation.y = rotation_q.y();
    pose.orientation.z = rotation_q.z();
    pose.position.x = T(0,3);
    pose.position.y = T(1,3);
    pose.position.z = T(2,3);
}

// this is a naive implementation
void SurfelMap::warp_inactive_surfels_cpu_kernel(int thread_i, int thread_num)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();
    int step = poses_database.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = poses_database.size();

    for(int i = begin_index; i < end_index; i ++)
    {
        if( poses_database[i].cam_pose.position.x == poses_database[i].loop_pose.position.x &&
            poses_database[i].cam_pose.position.y == poses_database[i].loop_pose.position.y &&
            poses_database[i].cam_pose.position.z == poses_database[i].loop_pose.position.z
            )
            continue;
        if(poses_database[i].attached_surfels.size() == 0)
        {
            poses_database[i].cam_pose = poses_database[i].loop_pose;
            continue;
        }

        PointCloud::Ptr warped_new_pointcloud(new PointCloud);

        Eigen::Matrix4d pre_pose, after_pose;
        Eigen::Matrix4f warp_matrix;
        pose_ros2eigen(poses_database[i].cam_pose, pre_pose);
        pose_ros2eigen(poses_database[i].loop_pose, after_pose);
        warp_matrix = (after_pose * pre_pose.inverse()).cast<float>();
        Eigen::MatrixXf point_positions(4, poses_database[i].attached_surfels.size());
        Eigen::MatrixXf point_norms(3, poses_database[i].attached_surfels.size());
        for(int surfel_i = 0; surfel_i < poses_database[i].attached_surfels.size(); surfel_i++)
        {
            point_positions(0,surfel_i) = poses_database[i].attached_surfels[surfel_i].px;
            point_positions(1,surfel_i) = poses_database[i].attached_surfels[surfel_i].py;
            point_positions(2,surfel_i) = poses_database[i].attached_surfels[surfel_i].pz;
            point_positions(3,surfel_i) = 1.0;
            point_norms(0,surfel_i) = poses_database[i].attached_surfels[surfel_i].nx;
            point_norms(1,surfel_i) = poses_database[i].attached_surfels[surfel_i].ny;
            point_norms(2,surfel_i) = poses_database[i].attached_surfels[surfel_i].nz;
        }
        point_positions = warp_matrix * point_positions;
        point_norms = warp_matrix.block<3,3>(0,0) * point_norms;
        for(int surfel_i = 0; surfel_i < poses_database[i].attached_surfels.size(); surfel_i++)
        {
            poses_database[i].attached_surfels[surfel_i].px = point_positions(0,surfel_i);
            poses_database[i].attached_surfels[surfel_i].py = point_positions(1,surfel_i);
            poses_database[i].attached_surfels[surfel_i].pz = point_positions(2,surfel_i);
            poses_database[i].attached_surfels[surfel_i].nx = point_norms(0,surfel_i);
            poses_database[i].attached_surfels[surfel_i].ny = point_norms(1,surfel_i);
            poses_database[i].attached_surfels[surfel_i].nz = point_norms(2,surfel_i);

            PointType new_point;
            new_point.x = poses_database[i].attached_surfels[surfel_i].px;
            new_point.y = poses_database[i].attached_surfels[surfel_i].py;
            new_point.z = poses_database[i].attached_surfels[surfel_i].pz;
            new_point.intensity = poses_database[i].attached_surfels[surfel_i].color;
            warped_new_pointcloud->push_back(new_point);
        }
        poses_database[i].cam_pose = poses_database[i].loop_pose;
        std::copy(&warped_new_pointcloud->front(), &warped_new_pointcloud->back(), &inactive_pointcloud->at(poses_database[i].points_begin_index));
    }
    end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> used_time = end_time - start_time;
    double all_time = used_time.count() * 1000.0;
    printf("warp kernel %d, cost %f ms.\n", thread_i, all_time);
}

void SurfelMap::warp_active_surfels_cpu_kernel(int thread_i, int thread_num, Eigen::Matrix4f transform_m)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    start_time = std::chrono::system_clock::now();
    int step = local_surfels.size() / thread_num;
    int begin_index = step * thread_i;
    int end_index = begin_index + step;
    if (thread_i == thread_num - 1)
        end_index = local_surfels.size();
    int surfel_num = end_index - begin_index;

    Eigen::MatrixXf point_positions(4, surfel_num);
    Eigen::MatrixXf point_norms(3, surfel_num);
    for(int i = 0; i < surfel_num; i++)
    {
        point_positions(0, i) = local_surfels[i + begin_index].px;
        point_positions(1, i) = local_surfels[i + begin_index].py;
        point_positions(2, i) = local_surfels[i + begin_index].pz;
        point_positions(3, i) = 1.0;
        point_norms(0, i) = local_surfels[i + begin_index].nx;
        point_norms(1, i) = local_surfels[i + begin_index].ny;
        point_norms(2, i) = local_surfels[i + begin_index].nz;
    }
    point_positions = transform_m * point_positions;
    point_norms = transform_m.block<3,3>(0,0) * point_norms;
    for(int i = 0; i < surfel_num; i++)
    {
        local_surfels[i + begin_index].px = point_positions(0, i);
        local_surfels[i + begin_index].py = point_positions(1, i);
        local_surfels[i + begin_index].pz = point_positions(2, i);
        local_surfels[i + begin_index].nx = point_norms(0, i);
        local_surfels[i + begin_index].ny = point_norms(1, i);
        local_surfels[i + begin_index].nz = point_norms(2, i);
    }

    end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> used_time = end_time - start_time;
    double all_time = used_time.count() * 1000.0;
    // printf("warp kernel %d, cost %f ms.\n", thread_i, all_time);
}

void SurfelMap::warp_surfels()
{
    // 先进行线程池清空， 然后说明要开启10个线程来做inactive surfels的操作
    warp_thread_pool.clear();
    warp_thread_num = 10;
    // warp inactive surfels
    for(int i = 0; i < warp_thread_num; i++)
    {
        std::thread this_thread(&SurfelMap::warp_inactive_surfels_cpu_kernel, this, i, warp_thread_num);
        warp_thread_pool.push_back(std::move(this_thread));
    }

    // warp active surfels
    int local_index = *local_surfels_indexs.begin();
    // 获得local_surfels对应的最后关键帧
    Eigen::Matrix4d pre_pose, loop_pose;
    Eigen::Matrix4f warp_pose;
    pose_ros2eigen(poses_database[local_index].cam_pose, pre_pose);
    pose_ros2eigen(poses_database[local_index].loop_pose, loop_pose);
    // 判断最后一帧对应的位姿差异情况，这种差异情况表示的是一种inconsistent。
    warp_pose = (loop_pose * pre_pose.inverse()).cast<float>();
    
    for(int i = 0; i < warp_thread_num; i++)
    {
        std::thread this_thread(&SurfelMap::warp_active_surfels_cpu_kernel, this, i, warp_thread_num, warp_pose);
        warp_thread_pool.push_back(std::move(this_thread));
    }

    // 这里是经典线程池的写法，join实际上不是线程的开启，而是线程的结束，这里是依次将线程结束
    for(int i = 0; i < warp_thread_pool.size(); i++)
        if(warp_thread_pool[i].joinable())
            warp_thread_pool[i].join();
}


void SurfelMap::calculate_memory_usage()
{
    double usgae_KB = 0;
    usgae_KB += local_surfels.size() * sizeof(SurfelElement)  / 1024.0;
    usgae_KB += poses_database.size() * sizeof(PoseElement) / 1024.0;
    usgae_KB += local_surfels_indexs.size() * sizeof(int) / 1024.0;
    // usgae_KB += inactive_pointcloud->size() * sizeof(PointType) / 1024.0;
    usgae_KB += inactive_pointcloud->size() * sizeof(SurfelElement)  / 1024.0;
    printf("the process comsumes %f KB\n", usgae_KB);
}

void SurfelMap::publish_camera_position(ros::Time pub_stamp, geometry_msgs::Pose &pose)
{
    CameraPoseVisualization current_frame_visual(1, 0, 0, 1);
    current_frame_visual.setScale(2.0);
    current_frame_visual.setLineWidth(0.6);

    Eigen::Vector3d current_position(
        pose.position.x,
        pose.position.y,
        pose.position.z);
    Eigen::Quaterniond current_pose(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    current_frame_visual.add_pose(current_position, current_pose);
    current_frame_visual.publish_by(camera_marker_publish, pub_stamp);
}
void SurfelMap::publish_pose_graph(ros::Time pub_stamp, int reference_index)
{
    nav_msgs::Path loop_path;
    loop_path.header.stamp = pub_stamp;
    loop_path.header.frame_id = "world";

    visualization_msgs::Marker loop_marker;
    loop_marker.header.frame_id = "world";
    loop_marker.header.stamp = pub_stamp;
    loop_marker.ns = "namespace";
    loop_marker.id = 0;
    loop_marker.type = visualization_msgs::Marker::LINE_LIST;
    loop_marker.action = visualization_msgs::Marker::ADD;
    loop_marker.scale.x = 0.01;
    loop_marker.scale.y = 0.01;
    loop_marker.scale.z = 0.01;
    loop_marker.color.a = 1.0; // Don't forget to set the alpha!
    loop_marker.color.r = 1.0;
    loop_marker.color.g = 0.0;
    loop_marker.color.b = 0.0;
    for(int i = 0; i < poses_database.size(); i++)
    {
        geometry_msgs::PoseStamped loop_pose;
        loop_pose.header.stamp = poses_database[i].cam_stamp;
        loop_pose.pose = poses_database[i].cam_pose;

        loop_path.poses.push_back(loop_pose);

        for(int j = 0; j < poses_database[i].linked_pose_index.size(); j++)
        {
            if(     poses_database[i].linked_pose_index[j] != i-1 
                &&  poses_database[i].linked_pose_index[j] != i+1
                &&  poses_database[i].linked_pose_index[j] > i
                )
            {
                geometry_msgs::Point one_point, another_point;
                one_point.x = poses_database[i].loop_pose.position.x;
                one_point.y = poses_database[i].loop_pose.position.y;
                one_point.z = poses_database[i].loop_pose.position.z;
                another_point.x = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.x;
                another_point.y = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.y;
                another_point.z = poses_database[poses_database[i].linked_pose_index[j]].loop_pose.position.z;
                loop_marker.points.push_back(one_point);
                loop_marker.points.push_back(another_point);
            }
        }
    }

    loop_path_publish.publish(loop_path);
    loop_marker_publish.publish(loop_marker);

    // publish driftfree poses
    visualization_msgs::Marker driftfree_marker;
    driftfree_marker.header.frame_id = "world";
    driftfree_marker.header.stamp = pub_stamp;
    driftfree_marker.ns = "driftfree";
    driftfree_marker.id = 0;
    driftfree_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    driftfree_marker.action = visualization_msgs::Marker::ADD;
    driftfree_marker.scale.x = 1.1;
    driftfree_marker.scale.y = 1.1;
    driftfree_marker.scale.z = 1.1;
    driftfree_marker.color.a = 1.0; // Don't forget to set the alpha!
    driftfree_marker.color.r = 1.0;
    driftfree_marker.color.g = 0.0;
    driftfree_marker.color.b = 0.0;
    vector<int> driftfree_indexs;
    get_driftfree_poses(reference_index, driftfree_indexs, drift_free_poses);
    for(int i = 0; i < driftfree_indexs.size(); i++)
    {
        geometry_msgs::Point one_point;
        one_point.x = poses_database[driftfree_indexs[i]].cam_pose.position.x;
        one_point.y = poses_database[driftfree_indexs[i]].cam_pose.position.y;
        one_point.z = poses_database[driftfree_indexs[i]].cam_pose.position.z;
        driftfree_marker.points.push_back(one_point);
    }
    visualization_msgs::Marker drift_marker;
    drift_marker.header.frame_id = "world";
    drift_marker.header.stamp = pub_stamp;
    drift_marker.ns = "drift";
    drift_marker.id = 0;
    drift_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    drift_marker.action = visualization_msgs::Marker::ADD;
    drift_marker.scale.x = 1.1;
    drift_marker.scale.y = 1.1;
    drift_marker.scale.z = 1.1;
    drift_marker.color.a = 1.0; // Don't forget to set the alpha!
    drift_marker.color.r = 0.0;
    drift_marker.color.g = 0.0;
    drift_marker.color.b = 0.0;
    for(int i = 0; i < poses_database.size(); i++)
    {
        if(std::find(driftfree_indexs.begin(), driftfree_indexs.end(), i) != driftfree_indexs.end())
            continue;
        geometry_msgs::Point one_point;
        one_point.x = poses_database[i].cam_pose.position.x;
        one_point.y = poses_database[i].cam_pose.position.y;
        one_point.z = poses_database[i].cam_pose.position.z;
        drift_marker.points.push_back(one_point);
    }
    driftfree_path_publish.publish(driftfree_marker);
    driftfree_path_publish.publish(drift_marker);
}

void SurfelMap::fuse_map(cv::Mat image, cv::Mat depth, Eigen::Matrix4f pose_input, int reference_index)
{
    printf("fuse surfels with reference index %d and %d surfels!\n", reference_index, local_surfels.size());    
    Timer fuse_timer("fusing");

    // 创建一个新的Surfel集合
    vector<SurfelElement> new_surfels;
    fusion_functions.fuse_initialize_map(
        reference_index,
        image,
        depth,
        pose_input,
        local_surfels,
        new_surfels
    );

    fuse_timer.middle("gpu part");

    // get the deleted surfel index
    vector<int> deleted_index;
    for(int i = 0; i < local_surfels.size(); i++)
    {
        if(local_surfels[i].update_times == 0)
            deleted_index.push_back(i);
    }
    fuse_timer.middle("delete index");

    // add new initialized surfels
    int add_surfel_num = 0;
    for(int i = 0; i < new_surfels.size(); i++)
    {
        if(new_surfels[i].update_times != 0)
        {
            SurfelElement this_surfel = new_surfels[i];
            if(deleted_index.size() > 0)
            {
                local_surfels[deleted_index.back()] = this_surfel;
                deleted_index.pop_back();
            }
            else
                local_surfels.push_back(this_surfel);
            add_surfel_num += 1;
        }
    }
    // remove deleted surfels
    while(deleted_index.size() > 0)
    {
        local_surfels[deleted_index.back()] = local_surfels.back();
        deleted_index.pop_back();
        local_surfels.pop_back();
    }
    fuse_timer.middle("cpu part");
    printf("add %d surfels, we now have %d local surfels.\n", add_surfel_num, local_surfels.size());
    fuse_timer.end();
}

void SurfelMap::publish_raw_pointcloud(cv::Mat &depth, cv::Mat &reference, geometry_msgs::Pose &pose)
{
    Eigen::Matrix3f rotation_R;
    Eigen::Vector3f translation_T;
    Eigen::Quaternionf rotation_q;
    rotation_q.w() = pose.orientation.w;
    rotation_q.x() = pose.orientation.x;
    rotation_q.y() = pose.orientation.y;
    rotation_q.z() = pose.orientation.z;
    rotation_R = rotation_q.toRotationMatrix();
    translation_T(0) = pose.position.x;
    translation_T(1) = pose.position.y;
    translation_T(2) = pose.position.z;

    PointCloud::Ptr pointcloud(new PointCloud);
    for(int i = 0; i < cam_width; i++)
    for(int j = 0; j < cam_height; j++)
    {
        float depth_value = depth.at<float>(j,i);
        Eigen::Vector3f cam_point;
        cam_point(0) = (i - cam_cx) * depth_value / cam_fx;
        cam_point(1) = (j - cam_cy) * depth_value / cam_fy;
        cam_point(2) = depth_value;
        Eigen::Vector3f world_point;
        world_point = rotation_R * cam_point + translation_T;

        PointType p;
        p.x = world_point(0);
        p.y = world_point(1);
        p.z = world_point(2);
        p.intensity = reference.at<uchar>(j,i);
        pointcloud->push_back(p);
    }
    pointcloud->header.frame_id = "world";
    raw_pointcloud_publish.publish(pointcloud);
    printf("publish raw point cloud with %d points.\n", pointcloud->size());
}

void SurfelMap::save_cloud(string save_path_name)
{
    printf("saving pointcloud ...\n");
    PointCloud::Ptr pointcloud(new PointCloud);
    for(int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if(local_surfels[surfel_it].update_times < 5)
            continue;
        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = local_surfels[surfel_it].pz;
        p.intensity = local_surfels[surfel_it].color;
        pointcloud->push_back(p);
    }
    
    (*pointcloud) += (*inactive_pointcloud);
    
    // pcl::io::savePLYFile(save_path_name.c_str(), *pointcloud);
    pcl::io::savePCDFile(save_path_name.c_str(), *pointcloud);
    printf("saving pointcloud done!\n");
}

void SurfelMap::push_a_surfel(vector<float> &vertexs, SurfelElement &this_surfel)
{
    int surfel_color = this_surfel.color;
    Eigen::Vector3f surfel_position;
    surfel_position(0) = this_surfel.px;
    surfel_position(1) = this_surfel.py;
    surfel_position(2) = this_surfel.pz;
    Eigen::Vector3f surfel_norm;
    surfel_norm(0) = this_surfel.nx;
    surfel_norm(1) = this_surfel.ny;
    surfel_norm(2) = this_surfel.nz;
    Eigen::Vector3f x_dir;
    x_dir(0) = -1 * this_surfel.ny;
    x_dir(1) = this_surfel.nx;
    x_dir(2) = 0;
    x_dir.normalize();
    Eigen::Vector3f y_dir;
    y_dir = surfel_norm.cross(x_dir);
    float radius = this_surfel.size;
    float h_r = radius * 0.5;
    float t_r = radius * 0.86603;
    Eigen::Vector3f point1, point2, point3, point4, point5, point6;
    point1 = surfel_position - x_dir * h_r - y_dir * t_r;
    point2 = surfel_position + x_dir * h_r - y_dir * t_r;
    point3 = surfel_position - x_dir * radius;
    point4 = surfel_position + x_dir * radius;
    point5 = surfel_position - x_dir * h_r + y_dir * t_r;
    point6 = surfel_position + x_dir * h_r + y_dir * t_r;
    vertexs.push_back(point1(0));vertexs.push_back(point1(1));vertexs.push_back(point1(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
    vertexs.push_back(point2(0));vertexs.push_back(point2(1));vertexs.push_back(point2(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
    vertexs.push_back(point3(0));vertexs.push_back(point3(1));vertexs.push_back(point3(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
    vertexs.push_back(point4(0));vertexs.push_back(point4(1));vertexs.push_back(point4(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
    vertexs.push_back(point5(0));vertexs.push_back(point5(1));vertexs.push_back(point5(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
    vertexs.push_back(point6(0));vertexs.push_back(point6(1));vertexs.push_back(point6(2));
    vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);vertexs.push_back(surfel_color);
}


void SurfelMap::save_mesh(string save_path_name)
{
    std::ofstream stream(save_path_name.c_str());
    if (!stream)
        return;
    std::vector<float> vertexs;
    for(int i = 0; i < poses_database.size(); i++)
    {
        for(int j = 0; j < poses_database[i].attached_surfels.size(); j++)
        {
            SurfelElement this_surfel = poses_database[i].attached_surfels[j];
            push_a_surfel(vertexs, this_surfel);
        }
    }

    for(int i = 0; i < local_surfels.size(); i++)
    {
        if(local_surfels[i].update_times < 5)
            continue;
        SurfelElement this_surfel = local_surfels[i];
        push_a_surfel(vertexs, this_surfel);
    }
    
    size_t numPoints = vertexs.size()/6;
    size_t numSurfels = numPoints/6;
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << numPoints << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "property uchar red" << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue" << std::endl;
    stream << "element face " << numSurfels * 4 <<  std::endl;
    stream << "property list uchar int vertex_index" << std::endl;
    stream << "end_header" << std::endl;

    for(int i = 0; i < numPoints; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            stream << vertexs[i*6+j] << " ";
        }
        stream << std::endl;
    }
    for(int i = 0; i < numSurfels; i++)
    {
        int p1, p2, p3, p4, p5, p6;
        p1 = i * 6 + 0;
        p2 = i * 6 + 1;
        p3 = i * 6 + 2;
        p4 = i * 6 + 3;
        p5 = i * 6 + 4;
        p6 = i * 6 + 5;
        stream << "3 " << p1 << " " << p2 << " " << p3 << std::endl;
        stream << "3 " << p2 << " " << p4 << " " << p3 << std::endl;
        stream << "3 " << p3 << " " << p4 << " " << p5 << std::endl;
        stream << "3 " << p5 << " " << p4 << " " << p6 << std::endl;
    }
    stream.close();
}


void SurfelMap::publish_neighbor_pointcloud(ros::Time pub_stamp, int reference_index)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> total_time;
    start_time = std::chrono::system_clock::now();

    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(local_surfels.size() + inactive_pointcloud->size());
    for(int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if(local_surfels[surfel_it].update_times == 0)
            continue;
        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = local_surfels[surfel_it].pz;
        p.intensity = local_surfels[surfel_it].color;
        pointcloud->push_back(p);
    }

    // add other pointcloud
    
    //METHOD 1, NAIVE ADD THE POINTS
    std::vector<int> neighbor_indexs;
    get_driftfree_poses(reference_index, neighbor_indexs, 2*drift_free_poses);
    for(int i = 0; i < neighbor_indexs.size(); i++)
    {
        int this_pose = neighbor_indexs[i];
        if(local_surfels_indexs.find(this_pose) != local_surfels_indexs.end())
            continue;
        int pointcloud_num = poses_database[this_pose].attached_surfels.size();
        int pointcloud_begin = poses_database[this_pose].points_begin_index;
        if(pointcloud_num <= 0)
            continue;
        pointcloud->insert(
            pointcloud->end(),
            inactive_pointcloud->begin()+pointcloud_begin,
            inactive_pointcloud->begin()+pointcloud_begin+pointcloud_num);
    }

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("construct point cloud cost %f ms.\n", total_time.count()*1000.0);
    start_time = std::chrono::system_clock::now();

    pointcloud->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud->header.stamp);
    pointcloud_publish.publish(pointcloud);
    printf("publish point cloud with %d points, in active %d points.\n", pointcloud->size(), inactive_pointcloud->size());

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("publish point cloud cost %f ms.\n", total_time.count()*1000.0);
}

void SurfelMap::publish_inactive_pointcloud(ros::Time pub_stamp)
{
    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(inactive_pointcloud->size());

    (*pointcloud) += (*inactive_pointcloud);

    pointcloud->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud->header.stamp);
    inactive_pointcloud_publish.publish(pointcloud);
    printf("publish point cloud with %d points, inactive %d points.\n", pointcloud->size(), inactive_pointcloud->size());

}
void SurfelMap::publish_active_pointcloud(ros::Time pub_stamp)
{
    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(local_surfels.size());
    for(int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if(local_surfels[surfel_it].update_times < 5)
            continue;
        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = local_surfels[surfel_it].pz;
        p.intensity = local_surfels[surfel_it].color;
        pointcloud->push_back(p);
    }
    pointcloud->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud->header.stamp);
    active_pointcloud_publish.publish(pointcloud);
    printf("publish point cloud with %d points, inactive %d points.\n", pointcloud->size(), inactive_pointcloud->size());
}

void SurfelMap::publish_all_pointcloud(ros::Time pub_stamp)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> total_time;
    start_time = std::chrono::system_clock::now();

    PointCloud::Ptr pointcloud(new PointCloud);
    pointcloud->reserve(local_surfels.size() + inactive_pointcloud->size());
    for(int surfel_it = 0; surfel_it < local_surfels.size(); surfel_it++)
    {
        if(local_surfels[surfel_it].update_times < 5)
            continue;
        PointType p;
        p.x = local_surfels[surfel_it].px;
        p.y = local_surfels[surfel_it].py;
        p.z = local_surfels[surfel_it].pz;
        p.intensity = local_surfels[surfel_it].color;
        pointcloud->push_back(p);
    }

    (*pointcloud) += (*inactive_pointcloud);

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("construct point cloud cost %f ms.\n", total_time.count()*1000.0);
    start_time = std::chrono::system_clock::now();

    pointcloud->header.frame_id = "world";
    pcl_conversions::toPCL(pub_stamp, pointcloud->header.stamp);
    pointcloud_publish.publish(pointcloud);
    printf("publish point cloud with %d points, inactive %d points.\n", pointcloud->size(), inactive_pointcloud->size());

    end_time = std::chrono::system_clock::now();
    total_time = end_time - start_time;
    printf("publish point cloud cost %f ms.\n", total_time.count()*1000.0);
}

void SurfelMap::move_add_surfels(int reference_index)
{
    // remove inactive surfels
    printf("get inactive surfels for pose %d.\n", reference_index);
    // vector<int> drift_poses;
    vector<int> poses_to_add;                                                               // active 对应的关键帧索引 
    vector<int> poses_to_remove;                                                            // inactive 对应的关键帧索引
    get_add_remove_poses(reference_index, poses_to_add, poses_to_remove);                   // 获得了active surfel对应的关键帧位姿和inactive Surfel对应的关键帧位姿
    std::chrono::time_point<std::chrono::system_clock> start_time, end_time;
    std::chrono::duration<double> move_pointcloud_time;
    /* ****************************************************************************************************
     * !@brief 这里是对inactive surfel的操作，通过poses_to_remove将active surfels中的位姿索引放到inactive中
     * ****************************************************************************************************/
    if(poses_to_remove.size() > 0)
    {
        start_time = std::chrono::system_clock::now();
        int added_surfel_num = 0;
        float sum_update_times = 0.0;
        for(int pi = 0; pi < poses_to_remove.size(); pi++)
        {
            // 将poses_to_remove中对应的inactive_index取出，然后在poses_database中进行搜索
            int inactive_index = poses_to_remove[pi];
            // 这帧即将沿着inactive_pointcloud继续往下添加，所以先将当前的inactive point cloud的size保存下来作为begin
            poses_database[inactive_index].points_begin_index = inactive_pointcloud->size();
            // 同样的，该帧也将被添加到inactive point cloud pose序列集pointcloud_pose_index中，所以这里会先得到当前inactive_pointcloud set对应的序列集个数
            poses_database[inactive_index].points_pose_index = pointcloud_pose_index.size();
            pointcloud_pose_index.push_back(inactive_index);
            // 对于当前local_surfes
            for(int i = 0; i < local_surfels.size(); i++)
            {
                // 如果该surfel对应的位姿是inactive_index的话
                if(local_surfels[i].update_times > 0 && local_surfels[i].last_update == inactive_index)
                {
                    // 添加到inactive_index对应的local_surfel中
                    poses_database[inactive_index].attached_surfels.push_back(local_surfels[i]);
                    PointType p;
                    p.x = local_surfels[i].px;
                    p.y = local_surfels[i].py;
                    p.z = local_surfels[i].pz;
                    p.intensity = local_surfels[i].color;
                    inactive_pointcloud->push_back(p);

                    added_surfel_num += 1;
                    sum_update_times += local_surfels[i].update_times;

                    // delete the surfel from the local point
                    // 这里通过update_times来进行更新的
                    local_surfels[i].update_times = 0;
                }
            }
            printf("remove pose %d from local poses, get %d surfels.\n", inactive_index, poses_database[inactive_index].attached_surfels.size());
            // active surfels对应的位姿索引集中删除这个index索引
            local_surfels_indexs.erase(inactive_index);
        }
        sum_update_times = sum_update_times / added_surfel_num;
        end_time = std::chrono::system_clock::now();
        move_pointcloud_time = end_time - start_time;
        printf("move surfels cost %f ms. the average update times is %f.\n", move_pointcloud_time.count()*1000.0, sum_update_times);
    }
    // 同理，这里对应的active 索引集合如果不为空，则应该添加新的进去
    if(poses_to_add.size() > 0)
    {
        // 1.0 add indexs
        local_surfels_indexs.insert(poses_to_add.begin(), poses_to_add.end());
        // 2.0 add surfels
        // 2.1 remove from inactive_pointcloud
        start_time = std::chrono::system_clock::now();
        std::vector<std::pair<int, int>> remove_info;//first, pointcloud start, pointcloud size, pointcloud pose index
        for(int add_i = 0; add_i < poses_to_add.size(); add_i++)
        {
            int add_index = poses_to_add[add_i];
            int pointcloud_pose_index = poses_database[add_index].points_pose_index;
            remove_info.push_back(std::make_pair(pointcloud_pose_index, add_index));
        }
        std::sort(
        remove_info.begin(),
        remove_info.end(),
        []( const std::pair<int, int >& first, const std::pair<int, int>& second)
        {
            return first.first < second.first;
        }
        );
        int remove_begin_index = remove_info[0].second;
        int remove_points_size = poses_database[remove_begin_index].attached_surfels.size();
        int remove_pose_size = 1;
        for(int remove_i = 1; remove_i <= remove_info.size(); remove_i++)
        {
            bool need_remove = false;
            if(remove_i == remove_info.size())
                need_remove = true;
            if(remove_i < remove_info.size())
            {
                if(remove_info[remove_i].first != (remove_info[remove_i-1].first + 1))
                    need_remove = true;
            }
            if(!need_remove)
            {
                int this_pose_index = remove_info[remove_i].second;
                remove_points_size += poses_database[this_pose_index].attached_surfels.size();
                remove_pose_size += 1;
                continue;
            }

            int remove_end_index = remove_info[remove_i - 1].second;
            printf("remove from pose %d -> %d, has %d points\n", remove_begin_index, remove_end_index, remove_points_size);

            PointCloud::iterator begin_ptr;
            PointCloud::iterator end_ptr;
            begin_ptr = inactive_pointcloud->begin() + poses_database[remove_begin_index].points_begin_index;
            end_ptr = begin_ptr + remove_points_size;
            inactive_pointcloud->erase(begin_ptr, end_ptr);
            
            for(int pi = poses_database[remove_end_index].points_pose_index + 1; pi < pointcloud_pose_index.size(); pi++)
            {
                poses_database[pointcloud_pose_index[pi]].points_begin_index -= remove_points_size;
                poses_database[pointcloud_pose_index[pi]].points_pose_index -= remove_pose_size; 
            }
    
            pointcloud_pose_index.erase(
                pointcloud_pose_index.begin() + poses_database[remove_begin_index].points_pose_index,
                pointcloud_pose_index.begin() + poses_database[remove_end_index].points_pose_index + 1
            );


            if(remove_i < remove_info.size())
            {
                remove_begin_index = remove_info[remove_i].second;;
                remove_points_size = poses_database[remove_begin_index].attached_surfels.size();
                remove_pose_size = 1;
            }
        }

        // 2.3 add the surfels into local
        for(int pi = 0; pi < poses_to_add.size(); pi++)
        {
            int pose_index = poses_to_add[pi];
            local_surfels.insert(
                local_surfels.end(),
                poses_database[pose_index].attached_surfels.begin(),
                poses_database[pose_index].attached_surfels.end());
            poses_database[pose_index].attached_surfels.clear();
            poses_database[pose_index].points_begin_index = -1;
            poses_database[pose_index].points_pose_index = -1;
        }
        end_time = std::chrono::system_clock::now();
        move_pointcloud_time = end_time - start_time;
        printf("add surfels cost %f ms.\n", move_pointcloud_time.count()*1000.0);
    }
}

void SurfelMap::get_add_remove_poses(int root_index, vector<int> &pose_to_add, vector<int> &pose_to_remove)
{
    vector<int> driftfree_poses;
    // root_index ->参考关键帧索引
    // driftfree_poses->对应的active 参考关键帧
    // drift_free_poses是做什么的暂时没看懂
    get_driftfree_poses(root_index, driftfree_poses, drift_free_poses);
    {
        printf("\ndriftfree poses: ");
        for(int i = 0; i < driftfree_poses.size(); i++)
        {
            printf("%d, ", driftfree_poses[i]);
        }
    }
    pose_to_add.clear();
    pose_to_remove.clear();
    // get to add
    for(int i = 0; i < driftfree_poses.size(); i++)
    {
        int temp_pose = driftfree_poses[i];
        // 如果这帧的pose在当前surfel对应的pose库中没有找到的话，则应该加上
        if(local_surfels_indexs.find(temp_pose) == local_surfels_indexs.end())
            pose_to_add.push_back(temp_pose);
    }
    {
        // 说明加上了几个pose
        printf("\nto add: ");
        for(int i = 0; i < pose_to_add.size(); i++)
        {
            printf("%d, ", pose_to_add[i]);
        }
    }
    // get to remove
    for(auto i = local_surfels_indexs.begin(); i != local_surfels_indexs.end(); i++)
    {
        // 如果对应的pose再里面是没有找到的话，则删除
        int temp_pose = *i;
        if( std::find(driftfree_poses.begin(), driftfree_poses.end(), temp_pose) ==  driftfree_poses.end() )
        {
            pose_to_remove.push_back(temp_pose);
        }
    }
    {
        printf("\nto remove: ");
        for(int i = 0; i < pose_to_remove.size(); i++)
        {
            printf("%d, ", pose_to_remove[i]);
        }
        printf("\n");
    }
}
/***************************************************************************************************
 *!@brief 不知道这里到底做什么的？按照意思看是在取出某个范围内的pose
 * **************************************************************************************************/
void SurfelMap::get_driftfree_poses(int root_index, vector<int> &driftfree_poses, int driftfree_range)
{``
    if(poses_database.size() < root_index + 1)
    {
        printf("get_driftfree_poses: pose database do not have the root index! This should only happen in initializaion!\n");
        return;
    }
    vector<int> this_level;
    vector<int> next_level;
    this_level.push_back(root_index);
    driftfree_poses.push_back(root_index);
    // get the drift
    for(int i = 1; i < driftfree_range; i++)
    {
        for(auto this_it = this_level.begin(); this_it != this_level.end(); this_it++)
        {
            for(auto linked_it = poses_database[*this_it].linked_pose_index.begin(); 
                linked_it != poses_database[*this_it].linked_pose_index.end();
                linked_it++)
            {
                bool already_saved = (find(driftfree_poses.begin(), driftfree_poses.end(), *linked_it) != driftfree_poses.end());
                if(!already_saved)
                {
                    next_level.push_back(*linked_it);
                    driftfree_poses.push_back(*linked_it);
                }
            }
        }
        this_level.swap(next_level);
        next_level.clear();
    }
}
