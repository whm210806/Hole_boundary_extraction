#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr averagePointCloud(const std::string& bag_file, const std::string& topic_name) {
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    // 初始化总和向量
    std::vector<float> x_sums, y_sums, z_sums;
    int num_messages = 0;

    // 读取bag文件中所有点云数据
    for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topic_name))) {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != nullptr) {
            // 读取每条消息中的点数
            size_t num_points = pc_msg->width * pc_msg->height;

            // 初始化第一次消息时的存储大小
            if (x_sums.empty()) {
                x_sums.resize(num_points, 0.0f);
                y_sums.resize(num_points, 0.0f);
                z_sums.resize(num_points, 0.0f);
            }

            // 检查每条消息是否有相同数量的点
            if (num_points != x_sums.size()) {
                ROS_WARN("Inconsistent point count in message; skipping message.");
                continue;
            }

            // 迭代每个点并累加到向量中
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_msg, "z");

            for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
                x_sums[i] += *iter_x;
                y_sums[i] += *iter_y;
                z_sums[i] += *iter_z;
            }
            num_messages++;
        }
    }
    bag.close();

    // 计算每个点的平均值
    pcl::PointCloud<pcl::PointXYZ>::Ptr avg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (num_messages > 0) {
        for (size_t i = 0; i < x_sums.size(); ++i) {
            pcl::PointXYZ avg_point;
            avg_point.x = x_sums[i] / num_messages;
            avg_point.y = y_sums[i] / num_messages;
            avg_point.z = z_sums[i] / num_messages;
            avg_cloud->push_back(avg_point);
        }
    } else {
        ROS_WARN("No valid point cloud data found in the topic: %s", topic_name.c_str());
    }

    ROS_INFO("Average Point Cloud Size: %zu", avg_cloud->size());

    return avg_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_ground(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointIndicesPtr ground (new pcl::PointIndices);
    // // Fill in the cloud data
    // // pcl::PCDReader reader;
    // // // Replace the path below with the path where you saved your file
    // // reader.read<pcl::PointXYZ> ("/home/mike/sick_Ws/cluster_0.pcd", *cloud);
    // std::cerr << "Cloud before filtering: " << std::endl;
    // std::cerr << *cloud << std::endl;
    // // Create the filtering object
    // pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    // pmf.setInputCloud (cloud);
    // pmf.setMaxWindowSize (20);
    // pmf.setSlope (1.0f);
    // pmf.setInitialDistance (0.8f);
    // pmf.setMaxDistance (3.0f);
    // pmf.extract (ground->indices);
    // // Create the filtering object
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (ground);
    // extract.filter (*cloud_filtered);
    // std::cerr << "Ground cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ> ("/home/mike/sick_Ws/cluster_ground.pcd", *cloud_filtered, false);
    // // Extract non-ground returns
    // extract.setNegative (true);
    // extract.filter (*cloud_filtered);
    // std::cerr << "Object cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    // writer.write<pcl::PointXYZ> ("/home/mike/sick_Ws/cluster_filtered.pcd", *cloud_filtered, false);
    // return cloud_filtered;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMaxIterations(1000);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0); //y axis
    seg.setAxis(axis);
    seg.setEpsAngle(60 * (M_PI / 180.0f)); // plane can be within 10.0 degrees of X-Z plane

    // Create pointcloud to publish inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr inversed_cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    // Fit a plane
    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);

    // Check result
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        // break;
    }

    // Extract inliers
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    // Let only the points that are not in the planar surface
    extract.setNegative(true);
    extract.filter(*inversed_cloud_plane);
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("/home/mike/sick_Ws/cluster_ground.pcd", *cloud_plane, false);
    writer.write<pcl::PointXYZ> ("/home/mike/sick_Ws/cluster_filtered.pcd", *inversed_cloud_plane, false);

    // pointClouds[0] = cloud_plane;
    // pointClouds[1] = inversed_cloud_plane;

    return inversed_cloud_plane;
}

// void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud) {
    // 创建 PCL 点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *cloud);

    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud(filtered_cloud);
    // sor.setLeafSize(0.01f, 0.01f, 0.01f); // 适当调整滤波粒度
    // sor.filter(*filtered_cloud);

    // 平面分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  // 距离阈值
    // seg.setOptimizeCoefficients(true);
    // // Mandatory
    // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    // seg.setMaxIterations(1000);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.01);

    // //because we want a specific plane (X-Z Plane) (In camera coordinates the ground plane is perpendicular to the y axis)
    // Eigen::Vector3f axis = Eigen::Vector3f(1.0, 0.0, 0.0); //y axis
    // seg.setAxis(axis);
    // seg.setEpsAngle(45 * (M_PI / 180.0f)); // plane can be within 10.0 degrees of X-Z plane

    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_INFO("未找到平面");
        return;
    }

    // 提取平面点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*planeCloud);
    pcl::io::savePCDFile("plane_cloud.pcd", *planeCloud);
    ROS_INFO("平面分割完成，已保存平面点云");

    // 欧氏聚类分割
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered_cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(clusterIndices);

    int j = 0;
    for (const auto& indices : clusterIndices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& index : indices.indices)
            cloudCluster->points.push_back(filtered_cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        std::string filename = "cluster_" + std::to_string(j) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *cloudCluster);
        ROS_INFO("保存聚类 %d 的点云，包含点数： %zu", j, cloudCluster->points.size());
        j++;
    }
}

int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "pointcloud_processing_node");
    ros::NodeHandle nh;

    // 读取ROS bag文件
    std::string bag_file = "/home/mike/sick_Ws/src/test/bag/camera_calibration.bag";  // 替换为你的ROS bag文件路径
    std::string topic_name = "/ifm3d_ros_driver/camera_3d/cloud";    // 设置你的点云话题
    std::string output_file = "average_output.pcd"; // 输出PCD文件名
    pcl::PointCloud<pcl::PointXYZ>::Ptr avg_cloud = averagePointCloud(bag_file, topic_name);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = extract_ground(avg_cloud);
    processPointCloud(filtered_cloud);

    // rosbag::Bag bag;
    // bag.open(bagFilePath, rosbag::bagmode::Read);
    // rosbag::View view(bag);

    // // 订阅点云话题
    // for (const auto& m : view) {
    //     sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    //     if (msg != nullptr) {
    //         processPointCloud(msg);
    //     }
    // }

    // bag.close();
    return 0;
}
