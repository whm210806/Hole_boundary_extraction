#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>



void circle_extraction() {
    // 读取边界点云的PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/mike/sick_Ws/boundary.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file \n");
        // return -1;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud);
    // sor.setMeanK(50); // 使用的邻域大小
    // sor.setStddevMulThresh(1.0); // 标准差乘数阈值
    // sor.filter(*cloud_filtered);

    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setInputCloud(cloud);
    // voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素大小
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // voxel_grid.filter(*cloud_filtered);

    // 设置分割对象的参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);  // 圆模型
    seg.setMethodType(pcl::SAC_RANSAC);         // 使用RANSAC方法
    seg.setDistanceThreshold(0.01);             // 点到圆的距离阈值，根据点云的噪声情况调整
    seg.setRadiusLimits(0.05,0.12);//添加拟合圆的半径限制，防止拟合过大或过小的圆
    seg.setInputCloud(cloud);

    // 执行分割
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        PCL_ERROR("Could not estimate a circular model for the given dataset.\n");
        // return -1;
    }

    // 输出拟合圆的参数
    std::cout << "Circle center (x, y): (" << coefficients->values[0] << ", " << coefficients->values[1] << ")\n";
    std::cout << "Circle radius: " << coefficients->values[2] << std::endl;

    // 可视化或保存圆的内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : inliers->indices) {
        circle_points->points.push_back(cloud->points[idx]);
    }
    circle_points->width = circle_points->points.size();
    circle_points->height = 1;
    circle_points->is_dense = true;

    pcl::io::savePCDFileASCII("circle_inliers.pcd", *circle_points);
    std::cout << "Circle inliers saved to circle_inliers.pcd" << std::endl;
}


int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "circle_extraction_node");
    ros::NodeHandle nh;
    circle_extraction();
    return 0;
}
