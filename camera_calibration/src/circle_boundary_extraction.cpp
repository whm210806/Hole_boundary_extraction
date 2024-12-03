#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>


void extractCircleOnPlaneWithNormal() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::PCDReader reader;
    reader.read("/home/mike/sick_Ws/cluster_0.pcd", *cloud);
    // Step 1: 平面分割
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.01);
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*plane_inliers, *plane_coefficients);

    if (plane_inliers->indices.empty()) {
        PCL_ERROR("未找到平面。\n");
        return;
    }

    // 获取平面法向量
    Eigen::Vector3f normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
    normal.normalize();

    std::cout << "normal 的维度: " << normal.size() << std::endl;

    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_i;
    // ne_i.setInputCloud(cloud);

    // // Use a KD-Tree to find the nearest neighbors
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_i(new pcl::search::KdTree<pcl::PointXYZ>());
    // ne_i.setSearchMethod(tree_i);

    // // Set the radius or K neighbors for normal estimation
    // ne_i.setRadiusSearch(0.03); // You can adjust this value based on your point cloud density

    // // Output data structure to hold normals and curvature
    // pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

    // // Compute the normals and curvature
    // ne_i.compute(*normal);

    // Eigen::Vector3f normal_vec(normal->points[0].normal_x, normal->points[0].normal_y, normal->points[0].normal_z);

    // 计算旋转矩阵，使平面法向量与z轴对齐
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(normal, z_axis);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation.toRotationMatrix();

    // 将平面内的点云转换到新的坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    // extract.setIndices(plane_inliers);
    extract.setNegative(false);  // 保留平面上的点
    extract.filter(*plane_cloud);
    pcl::transformPointCloud(*plane_cloud, *plane_cloud, transform);
    pcl::io::savePCDFileASCII("transformed_cloud.pcd", *plane_cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(plane_cloud);

    // Use a KD-Tree to find the nearest neighbors
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Set the radius or K neighbors for normal estimation
    ne.setRadiusSearch(0.03); // You can adjust this value based on your point cloud density

    // Output data structure to hold normals and curvature
    pcl::PointCloud<pcl::Normal>::Ptr normal_b(new pcl::PointCloud<pcl::Normal>);

    // Compute the normals and curvature
    ne.compute(*normal_b);
    
    /*pcl计算边界*/
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
    boundaries->resize(cloud->size()); //初始化大小
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
    boundary_estimation.setInputCloud(plane_cloud); //设置输入点云
    boundary_estimation.setInputNormals(normal_b); //设置输入法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>); 
    boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
    boundary_estimation.setKSearch(30); //设置k近邻数量
    boundary_estimation.setAngleThreshold(M_PI * 0.6); //设置角度阈值，大于阈值为边界
    boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    cloud_visual->resize(plane_cloud->size());
    for(size_t i = 0; i < plane_cloud->size(); i++)
    {
        cloud_visual->points[i].x = plane_cloud->points[i].x;
        cloud_visual->points[i].y = plane_cloud->points[i].y;
        cloud_visual->points[i].z = plane_cloud->points[i].z;
        if(boundaries->points[i].boundary_point != 0)
        {
            cloud_visual->points[i].r = 255;
            cloud_visual->points[i].g = 0;
            cloud_visual->points[i].b = 0;
            boundary_cloud->points.push_back(plane_cloud->points[i]);
        }
        else
        {
            cloud_visual->points[i].r = 255;
            cloud_visual->points[i].g = 255;
            cloud_visual->points[i].b = 255;
        }
    }

    boundary_cloud->width = boundary_cloud->points.size();
    boundary_cloud->height = 1;
    boundary_cloud->is_dense = true;

    pcl::io::savePCDFileASCII("transformed_boundary.pcd", *boundary_cloud);
    pcl::visualization::CloudViewer viewer("view");
    viewer.showCloud(cloud_visual);
    // system("PAUSE");
    std::cout << "Press Enter to continue...";
    std::cin.get();

    // cloud_visual->resize(cloud->size());
    // for(size_t i = 0; i < cloud->size(); i++)
    // {
    //     if(boundaries->points[i].boundary_point != 0)
    //     {
    //         boundary_cloud->points.push_back(cloud->points[i]);
    //     }
    // }



    // Step 2: 在平面上进行圆形边界提取
    pcl::SACSegmentation<pcl::PointXYZ> circle_seg;
    pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr circle_coefficients(new pcl::ModelCoefficients);
    circle_seg.setOptimizeCoefficients(true);
    circle_seg.setModelType(pcl::SACMODEL_CIRCLE2D);  // 使用2D圆模型
    circle_seg.setMethodType(pcl::SAC_RANSAC);
    circle_seg.setDistanceThreshold(0.01);
    circle_seg.setMaxIterations(2000); // 最大迭代次数
    circle_seg.setRadiusLimits(0.05, 0.1);  // 设置圆半径的限制
    circle_seg.setInputCloud(boundary_cloud);
    circle_seg.segment(*circle_inliers, *circle_coefficients);

    if (circle_inliers->indices.empty()) {
        PCL_ERROR("未找到圆形边界。\n");
        return;
    }

    // 输出圆的参数
    std::cout << "圆心 (x, y): (" << circle_coefficients->values[0] << ", "
              << circle_coefficients->values[1] << ")\n";
    std::cout << "半径: " << circle_coefficients->values[2] << std::endl;

    // Step 3: 将圆的索引映射回原始点云
    // pcl::PointIndices::Ptr original_circle_indices(new pcl::PointIndices);
    // for (const auto& idx : circle_inliers->indices) {
    //     original_circle_indices->indices.push_back(plane_inliers->indices[idx]);
    // }

    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : circle_inliers->indices) {
        circle_points->points.push_back(boundary_cloud->points[idx]);
    }
    circle_points->width = circle_points->points.size();
    circle_points->height = 1;
    circle_points->is_dense = true;

    Eigen::Matrix4f inverse_transform = transform.inverse();
    pcl::transformPointCloud(*circle_points, *circle_points, inverse_transform);
    pcl::io::savePCDFileASCII("circle_3.pcd", *circle_points);

    // 提取并保存圆的点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr circle_points(new pcl::PointCloud<pcl::PointXYZ>);
    // extract.setInputCloud(cloud);
    // extract.setIndices(original_circle_indices);
    // extract.setNegative(false);
    // extract.filter(*circle_points);
    // pcl::io::savePCDFileASCII("circle_inliers_with_normal.pcd", *circle_points);
    // std::cout << "圆形边界点已保存到文件: circle_inliers_with_normal.pcd" << std::endl;
}

int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "circle_hole_extraction_node");
    ros::NodeHandle nh;
    extractCircleOnPlaneWithNormal();

    return 0;
}
