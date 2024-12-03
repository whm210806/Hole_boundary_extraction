#include <ros/ros.h>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>


void hole_extraction() {
		/*输入点云和法线*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
        pcl::PCDReader reader;
        reader.read("/home/mike/sick_Ws/cluster_0.pcd", *cloud);
        // reader.read("/home/mike/sick_Ws/cluster_0.pcd", *normal);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Use a KD-Tree to find the nearest neighbors
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        // Set the radius or K neighbors for normal estimation
        ne.setRadiusSearch(0.03); // You can adjust this value based on your point cloud density

        // Output data structure to hold normals and curvature
        pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

        // Compute the normals and curvature
        ne.compute(*normal);
		
		/*pcl计算边界*/
		pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
		boundaries->resize(cloud->size()); //初始化大小
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
		boundary_estimation.setInputCloud(cloud); //设置输入点云
		boundary_estimation.setInputNormals(normal); //设置输入法线
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>); 
		boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
		boundary_estimation.setKSearch(30); //设置k近邻数量
		boundary_estimation.setAngleThreshold(M_PI * 0.6); //设置角度阈值，大于阈值为边界
		boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中

		/*可视化*/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud_visual->resize(cloud->size());
		for(size_t i = 0; i < cloud->size(); i++)
		{
			cloud_visual->points[i].x = cloud->points[i].x;
			cloud_visual->points[i].y = cloud->points[i].y;
			cloud_visual->points[i].z = cloud->points[i].z;
			if(boundaries->points[i].boundary_point != 0)
			{
				cloud_visual->points[i].r = 255;
				cloud_visual->points[i].g = 0;
				cloud_visual->points[i].b = 0;
				boundary_cloud->points.push_back(cloud->points[i]);
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

		std::string filename = "boundary.pcd";
        pcl::io::savePCDFileASCII(filename, *boundary_cloud);
        ROS_INFO("保存边界点云，包含点数： %zu", boundary_cloud->points.size());

		pcl::visualization::CloudViewer viewer("view");
		viewer.showCloud(cloud_visual);
		// system("PAUSE");
        std::cout << "Press Enter to continue...";
        std::cin.get();
}

int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "hole_extraction_node");
    ros::NodeHandle nh;
    hole_extraction();

    return 0;
}
