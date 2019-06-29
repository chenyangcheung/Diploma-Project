/******************************************
遍历输入点云文件并输出维度所需坐标
2019.05.05ChenyangCheung
*******************************************/

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>

int main()
{
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	char dir[200];
	sprintf(dir, "60607_1.pcd");
	reader.read(dir, *cloud);

	std::cerr << "PointCloud amount: " << cloud->width * cloud->height
		<< " data points." << std::endl;

	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;
	searchPoint.x = 0;
	searchPoint.y = 0;
	searchPoint.z = 0;

	int K = cloud->width * cloud->height;
	std::vector<int>pointIdxNKNSearch(K);
	std::vector<float>pointNKNSquaredDistance(K);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0)
	{
		for (size_t i = 0; i<pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
	system("PAUSE");
	return 0;
}
